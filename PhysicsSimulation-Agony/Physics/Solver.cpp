#include "Solver.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <numeric>
#include <iostream>

#if defined(_MSC_VER) || defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define SPIN_PAUSE() _mm_pause()
#else
#include <thread>
#define SPIN_PAUSE() std::this_thread::yield()
#endif

namespace PS_AGONY
{
    void Solver::setDataViewers(
        BodySoA& bodies,
        const std::vector<Material>& materials
    )
    {
        this->bodies = &bodies;
        this->materials = &materials;
    }

    size_t Solver::getMemoryUsage() const
    {
        size_t total = sizeof(Solver);

        // Threaded plan.
        total += getVectorMemoryUsage(threadedPlan.remainingIndices);
        total += getVectorMemoryUsage(threadedPlan.nextRemainingIndices);
        total += getVectorMemoryUsage(threadedPlan.usedBodies);
        total += getVectorMemoryUsage(threadedPlan.waves);

        for (auto& wave : threadedPlan.waves)
        {
            total += getVectorMemoryUsage(wave.passes);
            for (auto& pass : wave.passes)
            {
                total += getVectorMemoryUsage(pass.indices);
            }
        }

        total += getVectorMemoryUsage(positionAnchors);

        return total;
    }

    void Solver::solve(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        TRACY_SCOPE_NC("Solve constraints", Ecstasy::Color::OliveDrab);

        computeAnchorPoints(positionAnchors, narrowPhaseCollisions);

        solveVelocityConstraints(narrowPhaseCollisions, velocityIterations);
        solvePositionConstraints(narrowPhaseCollisions, positionAnchors, positionIterations);
    }

    void Solver::solveThreaded(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        TRACY_SCOPE_NC("Solve constraints", Ecstasy::Color::OliveDrab);

        planWorkerCount(narrowPhaseCollisions.size());

        // Single-threaded path.
        if (threadedPlan.workerCount <= 1)
        {
            solve(narrowPhaseCollisions, velocityIterations, positionIterations);
            return;
        }

        // Multi-threading path.
        computeAnchorPoints(positionAnchors, narrowPhaseCollisions);

        planExecutionWithGraphColoring(narrowPhaseCollisions);

        solveConstraintsThreaded(narrowPhaseCollisions, velocityIterations, positionIterations);
    }

    void Solver::planWorkerCount(size_t collisionCount)
    {
        // Single-threaded path.
        if (collisionCount < ThreadedConstraintSolvingPlan::MIN_COLLISION_COUNT_FOR_THREADING)
        {
            threadedPlan.workerCount = 0;
            return;
        }

        // Multi-threading path.
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t availableWorkerCount = threadPool.getThreadCount();
        const size_t neededWorkerCount = collisionCount / ThreadedConstraintSolvingPlan::COLLISION_COUNT_PER_WORKER;

        threadedPlan.workerCount = std::min(availableWorkerCount, neededWorkerCount);
    }

    void Solver::planExecutionWithGraphColoring(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        TRACY_SCOPE_NC("Plan execution", Ecstasy::Color::Blue);

        // Prepare containers.
        threadedPlan.nextRemainingIndices.clear();
        threadedPlan.waves.clear();
        threadedPlan.usedBodies.resize(bodies->getCount());

        // Fill remaining indices.
        const size_t collisionCount = narrowPhaseCollisions.size();
        threadedPlan.remainingIndices.resize(collisionCount);
        std::iota(
            threadedPlan.remainingIndices.begin(),
            threadedPlan.remainingIndices.end(),
            0ull
        );

        // Get pointers.
        const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

        // Coloring loop.
        while (true)
        {
            // Clear body-using history.
            std::fill(
                threadedPlan.usedBodies.begin(),
                threadedPlan.usedBodies.end(),
                ThreadedConstraintSolvingPlan::UsedSlot(-1) // Unsigned.
            );

            // Coloring.
            auto& currentWave = threadedPlan.waves.emplace_back(threadedPlan.workerCount);
            {
                // TODO: Make first grab take whole range for stage 0.

                const size_t readSize = threadedPlan.remainingIndices.size();
                size_t currentStageIndex = 0;
                for (size_t readPos = 0; readPos < readSize; readPos++)
                {
                    // Get collision data at index.
                    const size_t collisionIndex = threadedPlan.remainingIndices[readPos];
                    const auto& collisionData = narrowPhaseCollisions[collisionIndex];

                    // Get body indices.
                    const BodyIndex bodyIndexA = collisionData.bodyA;
                    const BodyIndex bodyIndexB = collisionData.bodyB;

                    // If body is used by previous stages, skip.
                    if (
                        threadedPlan.usedBodies[bodyIndexA] < currentStageIndex ||
                        threadedPlan.usedBodies[bodyIndexB] < currentStageIndex
                        )
                    {
                        threadedPlan.nextRemainingIndices.push_back(collisionIndex);
                        continue;
                    }

                    // Mark bodies as used by current stage.
                    if constexpr (ThreadedConstraintSolvingPlan::DO_NOT_MARK_STATIC_BODIES_AS_USED)
                    {
                        // If body is static, it won't get modified anyway, so there can't be any data race.
                        const size_t isStaticA = isStaticPtr[bodyIndexA];
                        const size_t isStaticB = isStaticPtr[bodyIndexB];

                        threadedPlan.usedBodies[bodyIndexA] = (-isStaticA) | (currentStageIndex & (~isStaticA));
                        threadedPlan.usedBodies[bodyIndexB] = (-isStaticB) | (currentStageIndex & (~isStaticB));
                    }
                    else
                    {
                        threadedPlan.usedBodies[bodyIndexA] = currentStageIndex;
                        threadedPlan.usedBodies[bodyIndexB] = currentStageIndex;
                    }

                    // Push index.
                    auto& pass = currentWave.passes[currentStageIndex].indices;
                    pass.push_back(collisionIndex);

                    // Advance to next stage or stop.
                    if (pass.size() >= ThreadedConstraintSolvingPlan::MAX_VALID_INDICES_PER_WORKER)
                    {
                        currentStageIndex++;
                        if (currentStageIndex >= threadedPlan.workerCount)
                        {
                            threadedPlan.nextRemainingIndices.insert(
                                threadedPlan.nextRemainingIndices.end(),
                                threadedPlan.remainingIndices.begin() + (readPos + 1),
                                threadedPlan.remainingIndices.end()
                            );
                            break;
                        }
                    }
                }

                threadedPlan.remainingIndices.clear();
                threadedPlan.remainingIndices.swap(threadedPlan.nextRemainingIndices);
            }
            if (currentWave.passes[0].indices.empty()) [[unlikely]]
            {
                break;
            }
            else if (threadedPlan.remainingIndices.empty())
            {
                break;
            }
        }
    }

    void Solver::computeAnchorPoints(
        std::vector<PositionAnchor>& outPositionAnchors,
        const std::vector<BodyCollisionData>& narrowPhaseCollisions
    )
    {
        TRACY_SCOPE_NC("Compute anchor points", Ecstasy::Color::Chocolate);

        Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();

        auto getCenterOfMass = [&](BodyIndex bodyIndex) -> Vec2
            {
                return {
                    positionXPtr[bodyIndex] + localCenterOfMassXPtr[bodyIndex],
                    positionYPtr[bodyIndex] + localCenterOfMassYPtr[bodyIndex]
                };
            };

        auto invRotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
            {
                return { v.x * cos + v.y * sin, -v.x * sin + v.y * cos };
            };


        const size_t collisionCount = narrowPhaseCollisions.size();
        outPositionAnchors.resize(collisionCount);
        for (size_t c = 0; c < collisionCount; c++)
        {
            const auto& data = narrowPhaseCollisions[c];
            const Vec2 contactPoint = data.contacts[0]; // Only the first point is used.

            const Vec2 centerOfMassA = getCenterOfMass(data.bodyA);
            const Vec2 centerOfMassB = getCenterOfMass(data.bodyB);

            outPositionAnchors[c].localAnchorA = invRotate(contactPoint - centerOfMassA, rotationCosPtr[data.bodyA], rotationSinPtr[data.bodyA]);
            outPositionAnchors[c].localAnchorB = invRotate(contactPoint - centerOfMassB, rotationCosPtr[data.bodyB], rotationSinPtr[data.bodyB]);
        }
    }

    void Solver::solveVelocityConstraints(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t solverIterations
    )
    {
        TRACY_SCOPE_NC("Solve velocity constraints", Ecstasy::Color::Violet);

        constexpr Real frictionEpsilonSq = Real(1e-3 * 1e-3);

        // Get pointers.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();

        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();

        Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

        const MaterialIndex* ECSTASY_RESTRICT materialIndexPtr = bodies->materialIndex.data();
        const Material* ECSTASY_RESTRICT materialPtr = materials->data();

        // Lambdas.
        auto getLinearVelocity = [&](const BodyIndex& bodyIndex) -> Vec2
            {
                const Real velocityX = velocityXPtr[bodyIndex];
                const Real velocityY = velocityYPtr[bodyIndex];
                return { velocityX, velocityY };
            };

        auto getCenterOfMass = [&](BodyIndex bodyIndex) -> Vec2
            {
                const Real positionX = positionXPtr[bodyIndex];
                const Real positionY = positionYPtr[bodyIndex];

                const Real localCOMX = localCenterOfMassXPtr[bodyIndex];
                const Real localCOMY = localCenterOfMassYPtr[bodyIndex];

                return { positionX + localCOMX, positionY + localCOMY };
            };

        // Main loop.
        for (uint32_t solverIteration = 0; solverIteration < solverIterations; solverIteration++)
        {
            for (const auto& data : narrowPhaseCollisions)
            {
                // Get body indices.
                const BodyIndex bodyIndexA = data.bodyA;
                const BodyIndex bodyIndexB = data.bodyB;

                // Get inv masses.
                const Real invMassA = invMassPtr[bodyIndexA];
                const Real invMassB = invMassPtr[bodyIndexB];

                const Real totalInvMass = invMassA + invMassB;
                if (totalInvMass <= Real(0)) [[unlikely]]
                {
                    continue;
                }

                // Get materials.
                const MaterialIndex materialIndexA = materialIndexPtr[bodyIndexA];
                const MaterialIndex materialIndexB = materialIndexPtr[bodyIndexB];

                const Material* materialA = materialPtr + materialIndexA;
                const Material* materialB = materialPtr + materialIndexB;

                const Real elasticityPlusOne = (materialA->elasticity + materialB->elasticity) * Real(0.5) + Real(1.0); // Hoping for fused multiply-add. Adding here instead of adding in impulse calculation.

                const Real staticFriction = std::sqrt(std::fmax(Real(0), materialA->staticFriction * materialB->staticFriction));
                const Real dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

                // Compute world centers of mass.
                const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
                const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

                //
                const Real invInertiaA = invInertiaPtr[bodyIndexA];
                const Real invInertiaB = invInertiaPtr[bodyIndexB];

                Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                Real angularVelocityA = angularVelocityPtr[bodyIndexA];
                Real angularVelocityB = angularVelocityPtr[bodyIndexB];

                const Vec2 normal = data.normal;
                const Real depth = data.depth;

                // Calculate collision impulses.
                std::array<Vec2, 2> impulseArray{};
                std::array<Vec2, 2> rAPerpArray{};
                std::array<Vec2, 2> rBPerpArray{};
                std::array<Real, 2> jnArray{};
                const uint32_t contactCount = data.contactCount; // std::min(data.contactCount, 2u);

                const Real impulseScale = Real(1.0) / Real(data.contactCount);
                {
                    bool noContacts = true;

                    for (uint32_t i = 0; i < contactCount; i++)
                    {
                        const Vec2 contactPoint = data.contacts[i];

                        const Vec2 rA = contactPoint - centerOfMassA;
                        const Vec2 rB = contactPoint - centerOfMassB;

                        const Vec2 rAPerp = { -rA.y, rA.x };
                        const Vec2 rBPerp = { -rB.y, rB.x };

                        const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
                        const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

                        const Vec2 relativeVelocity =
                            (linearVelocityB + angularLinearVelB) -
                            (linearVelocityA + angularLinearVelA);

                        const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

                        if (velocityAlongNormal > Real(0)) continue;

                        const Real rAPerpDotN = glm::dot(rAPerp, normal);
                        const Real rBPerpDotN = glm::dot(rBPerp, normal);

                        const Real inertiaTermA = rAPerpDotN * rAPerpDotN * invInertiaA;
                        const Real inertiaTermB = rBPerpDotN * rBPerpDotN * invInertiaB;

                        const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
                        const Real jn = -elasticityPlusOne * velocityAlongNormal / denom * impulseScale;

                        impulseArray[i] = jn * normal;
                        rAPerpArray[i] = rAPerp;
                        rBPerpArray[i] = rBPerp;
                        jnArray[i] = jn;

                        noContacts = false;
                    }

                    // Check if there is at least one valid contact.
                    if (noContacts) continue;
                }

                // Apply collision impulses.
                {
                    const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

                    linearVelocityA -= impulseSum * invMassA;
                    angularVelocityA -= (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;

                    linearVelocityB += impulseSum * invMassB;
                    angularVelocityB += (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                }

                // Calculate friction impulses.
                {
                    for (uint32_t i = 0; i < contactCount; i++)
                    {
                        const Vec2 rAPerp = rAPerpArray[i];
                        const Vec2 rBPerp = rBPerpArray[i];

                        const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
                        const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

                        const Vec2 relativeVelocity =
                            (linearVelocityB + angularLinearVelB) -
                            (linearVelocityA + angularLinearVelA);

                        Vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
                        const Real tangentLengthSq = glm::dot(tangent, tangent);
                        if (tangentLengthSq < frictionEpsilonSq)
                        {
                            impulseArray[i] = Vec2(0.0, 0.0);
                            continue;
                        }

                        tangent /= std::sqrt(tangentLengthSq);

                        const Real rAPerpDotT = glm::dot(rAPerp, tangent);
                        const Real rBPerpDotT = glm::dot(rBPerp, tangent);

                        const Real inertiaTermA = rAPerpDotT * rAPerpDotT * invInertiaA;
                        const Real inertiaTermB = rBPerpDotT * rBPerpDotT * invInertiaB;

                        const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
                        const Real jt = glm::dot(relativeVelocity, tangent) / denom * impulseScale;

                        const Real jn = jnArray[i];
                        if (std::fabs(jt) <= jn * staticFriction)
                        {
                            impulseArray[i] = -jt * tangent; // Static friction.
                        }
                        else
                        {
                            const Real maxDynamic = jn * dynamicFriction;
                            const Real f = -std::clamp(jt, -maxDynamic, maxDynamic);
                            impulseArray[i] = f * tangent; // Dynamic friction.
                        }
                    }
                }

                // Apply friction impulses.
                {
                    const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

                    linearVelocityA -= impulseSum * invMassA;
                    angularVelocityA -= (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;

                    linearVelocityB += impulseSum * invMassB;
                    angularVelocityB += (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                }

                // Store velocities.
                velocityXPtr[bodyIndexA] = linearVelocityA.x;
                velocityYPtr[bodyIndexA] = linearVelocityA.y;
                velocityXPtr[bodyIndexB] = linearVelocityB.x;
                velocityYPtr[bodyIndexB] = linearVelocityB.y;

                angularVelocityPtr[bodyIndexA] = angularVelocityA;
                angularVelocityPtr[bodyIndexB] = angularVelocityB;
            }
        }
    }

    void Solver::solvePositionConstraints(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        const std::vector<PositionAnchor>& positionAnchors,
        uint32_t solverIterations
    )
    {
        TRACY_SCOPE_NC("Solve position constraints", Ecstasy::Color::Indigo);

        Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();

        auto getCenterOfMass = [&](BodyIndex bodyIndex) -> Vec2
            {
                return {
                    positionXPtr[bodyIndex] + localCenterOfMassXPtr[bodyIndex],
                    positionYPtr[bodyIndex] + localCenterOfMassYPtr[bodyIndex]
                };
            };

        auto rotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
            {
                return { v.x * cos - v.y * sin, v.x * sin + v.y * cos };
            };

        // Cache each contact's anchor in local (unrotated) space, relative to each body's
        // center of mass, at the moment the contact was generated (both anchors coincide
        // with data.contacts[0] right now, before any correction has moved anything).
        const size_t collisionCount = narrowPhaseCollisions.size();

        for (uint32_t iteration = 0; iteration < solverIterations; iteration++)
        {
            for (size_t c = 0; c < collisionCount; c++)
            {
                const auto& data = narrowPhaseCollisions[c];
                const BodyIndex bodyIndexA = data.bodyA;
                const BodyIndex bodyIndexB = data.bodyB;

                const Real invMassA = invMassPtr[bodyIndexA];
                const Real invMassB = invMassPtr[bodyIndexB];
                const Real totalInvMass = invMassA + invMassB;
                if (totalInvMass <= Real(0)) continue;

                // Re-derive the world anchors from the current (possibly already corrected) transforms.
                const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
                const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

                const Vec2 worldAnchorA = centerOfMassA + rotate(positionAnchors[c].localAnchorA, rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
                const Vec2 worldAnchorB = centerOfMassB + rotate(positionAnchors[c].localAnchorB, rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

                // The anchors coincided (drift = 0) when depth was measured, so drift is exactly
                // how much penetration has already been resolved since then.
                const Real drift = glm::dot(worldAnchorB - worldAnchorA, data.normal);
                const Real currentSeparation = data.depth - drift;

                const Real correctionDepth = currentSeparation - simulationSettings.positionCorrectionSlop;
                if (correctionDepth <= Real(0)) continue;

                const Real invTotalInvMass_x_Depth = correctionDepth / totalInvMass;
                const Real correctionA = invMassA * invTotalInvMass_x_Depth * simulationSettings.positionCorrectionPercent;
                const Real correctionB = invMassB * invTotalInvMass_x_Depth * simulationSettings.positionCorrectionPercent;

                const Vec2 correctionAVec = data.normal * correctionA;
                const Vec2 correctionBVec = data.normal * correctionB;

                positionXPtr[bodyIndexA] -= correctionAVec.x;
                positionYPtr[bodyIndexA] -= correctionAVec.y;
                positionXPtr[bodyIndexB] += correctionBVec.x;
                positionYPtr[bodyIndexB] += correctionBVec.y;
            }
        }
    }

    void Solver::solveConstraintsThreaded(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        auto& threadPool = Threading::getGlobalThreadPool();

        TRACY_SCOPE_NC("Solve constraints (Multi-threaded)", Ecstasy::Color::Purple);

        const size_t workerCount = threadedPlan.workerCount;
        const size_t waveCount = threadedPlan.waves.size();

        if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0))
        {
            return;
        }

        const uint32_t positionSolvingStartTick = static_cast<uint32_t>(waveCount) * velocityIterations;
        const uint32_t totalTicks = positionSolvingStartTick + static_cast<uint32_t>(waveCount) * positionIterations;

        // Worker data.
        workerResources.workerData.resize(workerCount);
        for (auto& w : workerResources.workerData)
        {
            w.isDestroyed.store(false, std::memory_order_release);
        }

        workerResources.currentWaveTicket.store(0, std::memory_order_release);
        workerResources.workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);

        // Lambdas.
        auto workerFunc = [&](size_t workerIndex)
            {
                auto& wData = workerResources.workerData[workerIndex];

                uint32_t localTicket = 0;
                while (true)
                {
                    // All waves x all iterations done -> stop.
                    if (localTicket >= totalTicks) break;

                    const size_t waveIndex = localTicket % waveCount;
                    const auto& wave = threadedPlan.waves[waveIndex];
                    const auto& indices = wave.passes[workerIndex].indices;

                    // Get work from current wave and execute it. If empty, skip.
                    if (!indices.empty())
                    {
                        if (localTicket >= positionSolvingStartTick)
                        {
                            solvePositionConstraintsIndirect(narrowPhaseCollisions, indices);
                        }
                        else
                        {
                            solveVelocityConstraintsIndirect(narrowPhaseCollisions, indices);
                        }
                    }

                    // Decrease atomic counter; last one to finish advances the wave and wakes everyone.
                    const uint32_t remaining = workerResources.workNotDone.fetch_sub(1, std::memory_order_acq_rel) - 1;
                    if (remaining == 0)
                    {
                        workerResources.workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);
                        workerResources.currentWaveTicket.fetch_add(1, std::memory_order_release);
                    }

                    // Wait for new wave.
                    {
                        TRACY_SCOPE_NC("Spin", Ecstasy::Color::Black);
                        while (workerResources.currentWaveTicket.load(std::memory_order_acquire) == localTicket)
                        {
                            SPIN_PAUSE();
                        }
                    }
                    localTicket = workerResources.currentWaveTicket.load(std::memory_order_acquire);

                    // Loop naturally wraps back to wave 0 via (localTicket % waveCount) until totalTicks is hit.
                }

                wData.isDestroyed.store(true, std::memory_order_release);
                wData.isDestroyed.notify_one();
            };

        for (size_t i = 1; i < workerCount; i++)
        {
            threadPool.enqueue(workerFunc, i);
        }
        workerFunc(0);

        // Wait for all workers to finish every iteration and terminate.
        {
            TRACY_SCOPE_NC("Wait for workers to finish", Ecstasy::Color::Brown);
            for (size_t i = 1; i < workerResources.workerData.size(); i++)
            {
                workerResources.workerData[i].isDestroyed.wait(false, std::memory_order_acquire);
            }
        }
    }

    void Solver::solveVelocityConstraintsIndirect(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        const std::vector<size_t>& collisionIndices
    )
    {
        TRACY_SCOPE_NC("Solve velocity constraints (Indirect)", Ecstasy::Color::HotPink);

        // My tests show that copying data to make it sequantial is a little faster than doing indirect loads.
        // Plus it allows for having single source of truth for collision resolution.

        static thread_local std::vector<BodyCollisionData> tempData; // TODO: Add to memory usage.

        const size_t collisionCount = collisionIndices.size();

        {
            TRACY_SCOPE_N("Allocate and copy");

            tempData.resize(collisionCount);
            for (size_t i = 0; i < collisionCount; i++)
            {
                tempData[i] = narrowPhaseCollisions[collisionIndices[i]];
            }
        }

        solveVelocityConstraints(tempData, 1);
    }

    void Solver::solvePositionConstraintsIndirect(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        const std::vector<size_t>& collisionIndices
    )
    {
        TRACY_SCOPE_NC("Solve position constraints (Indirect)", Ecstasy::Color::HotPink);

        // My tests show that copying data to make it sequantial is a little faster than doing indirect loads.
        // Plus it allows for having single source of truth for collision resolution.

        static thread_local std::vector<BodyCollisionData> tempCollisionData; // TODO: Add to memory usage.
        static thread_local std::vector<PositionAnchor> tempPositionAnchorData;
        // TODO: Better create pool for these.

        const size_t collisionCount = collisionIndices.size();

        {
            TRACY_SCOPE_N("Allocate and copy");

            tempCollisionData.resize(collisionCount);
            for (size_t i = 0; i < collisionCount; i++)
            {
                tempCollisionData[i] = narrowPhaseCollisions[collisionIndices[i]];
            }

            tempPositionAnchorData.resize(collisionCount);
            for (size_t i = 0; i < collisionCount; i++)
            {
                tempPositionAnchorData[i] = positionAnchors[collisionIndices[i]];
            }
        }

        solvePositionConstraints(tempCollisionData, tempPositionAnchorData, 1);
    }
}