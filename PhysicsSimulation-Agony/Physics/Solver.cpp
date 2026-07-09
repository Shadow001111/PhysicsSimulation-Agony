#include "Solver.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

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

        total += solvingPlanner.getMemoryUsage();

        total += getVectorMemoryUsage(positionAnchors);

        total += getVectorMemoryUsage(indirectCollisionData);
        total += getVectorMemoryUsage(indirectPositionAnchorData);

        return total;
    }

    void Solver::solve(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        TRACY_SCOPE_NC("Solve constraints (Single-threaded)", Ecstasy::Color::OliveDrab);

        computeAnchorPoints(positionAnchors, narrowPhaseCollisions);

        for (uint32_t i = 0; i < velocityIterations; i++)
        {
            solveVelocityConstraints(narrowPhaseCollisions);
        }
        for (uint32_t i = 0; i < positionIterations; i++)
        {
            solvePositionConstraints(narrowPhaseCollisions, positionAnchors);
        }
    }

    void Solver::solveThreaded(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        TRACY_SCOPE_NC("Solve constraints (Multi-threaded)", Ecstasy::Color::OliveDrab);

        const size_t workerCount = planWorkerCount(narrowPhaseCollisions.size());

        // Single-threaded path.
        if (workerCount <= 1)
        {
            solve(narrowPhaseCollisions, velocityIterations, positionIterations);
            return;
        }

        // Multi-threading path.
        computeAnchorPoints(positionAnchors, narrowPhaseCollisions);

        {
            TRACY_SCOPE_NC("Collect colliding body pairs from collision data", Ecstasy::Color::Red);

            const size_t collisionCount = narrowPhaseCollisions.size();

            collidingBodyPairs.resize(collisionCount);
            for (size_t i = 0; i < collisionCount; i++)
            {
                const BodyCollisionData& collData = narrowPhaseCollisions[i];
                collidingBodyPairs[i] = { collData.bodyA, collData.bodyB };
            }
        }

        {
            TRACY_SCOPE_NC("Plan execution", Ecstasy::Color::Blue);
            solvingPlanner.setWorkerCount(workerCount);
            solvingPlanner.planExecution(collidingBodyPairs, bodies->getCount());
        }

        solveConstraintsThreaded(narrowPhaseCollisions, velocityIterations, positionIterations);

        // Note: I tried to 'computeAnchorPoints'in parallel with 'solvingPlanner.planExecution'. It caused slowdown in second.
        // Maybe app is memory bound.
    }

    size_t Solver::planWorkerCount(size_t collisionCount)
    {
        static constexpr size_t COLLISION_COUNT_PER_WORKER = 830;
        static constexpr size_t MIN_COLLISION_COUNT_FOR_THREADING = COLLISION_COUNT_PER_WORKER * 3;

        // Single-threaded path.
        if (collisionCount < MIN_COLLISION_COUNT_FOR_THREADING)
        {
            return 0;
        }

        // Multi-threading path.
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t availableWorkerCount = threadPool.getThreadCount();
        const size_t neededWorkerCount = collisionCount / COLLISION_COUNT_PER_WORKER;

        return std::min(availableWorkerCount, neededWorkerCount);
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
        std::span<const BodyCollisionData> narrowPhaseCollisions
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
        for (const auto& data : narrowPhaseCollisions)
        {
            // Get body indices.
            const BodyIndex bodyIndexA = data.bodyA;
            const BodyIndex bodyIndexB = data.bodyB;

            // Get inv masses.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            const Real totalInvMass = invMassA + invMassB;

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

            std::array<Vec2, 2> rAPerpArray{};
            std::array<Vec2, 2> rBPerpArray{};
            std::array<Real, 2> jnArray{};
            const uint32_t contactCount = data.contactCount; // std::min(data.contactCount, 2u);

            // Calculate collision impulses and apply them.
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
                    const Real jn = -elasticityPlusOne * velocityAlongNormal / denom;

                    const Vec2 impulse = jn * normal;
                    rAPerpArray[i] = rAPerp;
                    rBPerpArray[i] = rBPerp;
                    jnArray[i] = jn;
                    noContacts = false;

                    // Apply impulse.
                    linearVelocityA -= impulse * invMassA;
                    angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

                    linearVelocityB += impulse * invMassB;
                    angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
                }

                // Check if there is at least one valid contact.
                if (noContacts) continue;
            }

            // Calculate friction impulses and apply them.
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
                    if (tangentLengthSq < frictionEpsilonSq) continue;

                    tangent /= std::sqrt(tangentLengthSq);

                    const Real rAPerpDotT = glm::dot(rAPerp, tangent);
                    const Real rBPerpDotT = glm::dot(rBPerp, tangent);

                    const Real inertiaTermA = rAPerpDotT * rAPerpDotT * invInertiaA;
                    const Real inertiaTermB = rBPerpDotT * rBPerpDotT * invInertiaB;

                    const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
                    const Real jt = glm::dot(relativeVelocity, tangent) / denom;

                    const Real jn = jnArray[i];

                    Vec2 impulse;
                    if (std::fabs(jt) <= jn * staticFriction)
                    {
                        impulse = -jt * tangent; // Static friction.
                    }
                    else
                    {
                        const Real maxDynamic = jn * dynamicFriction;
                        const Real f = -std::clamp(jt, -maxDynamic, maxDynamic);
                        impulse = f * tangent; // Dynamic friction.
                    }

                    // Apply impulse.
                    linearVelocityA -= impulse * invMassA;
                    angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

                    linearVelocityB += impulse * invMassB;
                    angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
                }
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

    void Solver::solvePositionConstraints(
        std::span<const BodyCollisionData> narrowPhaseCollisions,
        std::span<const PositionAnchor> positionAnchors
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

        const size_t collisionCount = narrowPhaseCollisions.size();

        // Note: I tried to use Simd, it was slower, probably because of the gather-scatter, or just memory intensive.
        //       plus it was invalid because same body index could appear multiple times in simd batch (on same worker).
        //       remaking solving planner to account for that would be hell!

        for (size_t c = 0; c < collisionCount; c++)
        {
            const auto& data = narrowPhaseCollisions[c];
            const BodyIndex bodyIndexA = data.bodyA;
            const BodyIndex bodyIndexB = data.bodyB;

            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];
            const Real totalInvMass = invMassA + invMassB;

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

            const Real totalCorrection = correctionDepth / totalInvMass * simulationSettings.positionCorrectionPercent;
            const Real correctionA = invMassA * totalCorrection;
            const Real correctionB = invMassB * totalCorrection;

            const Vec2 correctionAVec = data.normal * correctionA;
            const Vec2 correctionBVec = data.normal * correctionB;

            positionXPtr[bodyIndexA] -= correctionAVec.x;
            positionYPtr[bodyIndexA] -= correctionAVec.y;
            positionXPtr[bodyIndexB] += correctionBVec.x;
            positionYPtr[bodyIndexB] += correctionBVec.y;
        }
    }

    void Solver::solveConstraintsThreaded(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t workerCount = solvingPlanner.getWorkerCount();

        const auto& passOffsets = solvingPlanner.getPassOffsets();
        const size_t waveCount = passOffsets.size() / workerCount;
        const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

        if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0))
        {
            return;
        }

        // Reorder indirect data.
        {
            TRACY_SCOPE_N("Reorder indirect data");

            const auto& flatIndices = solvingPlanner.getFlatIndices();
            const size_t mappedCount = flatIndices.size();
            const size_t* ECSTASY_RESTRICT flatIndicesPtr = flatIndices.data();

            indirectCollisionData.resize(mappedCount);
            indirectPositionAnchorData.resize(mappedCount);
            for (size_t i = 0; i < mappedCount; i++)
            {
                const size_t originalIndex = flatIndicesPtr[i];
                indirectCollisionData[i] = narrowPhaseCollisions[originalIndex];
                indirectPositionAnchorData[i] = positionAnchors[originalIndex];
            }
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

                    const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
                    const auto& passOffset = passOffsetsPtr[passGlobalIndex];

                    // Get work from current wave and execute it. If empty, skip.
                    if (passOffset.size > 0)
                    {
                        std::span<const BodyCollisionData> collisionSlice(indirectCollisionData.data() + passOffset.start, passOffset.size);

                        if (localTicket >= positionSolvingStartTick)
                        {
                            std::span<const PositionAnchor> anchorSlice(indirectPositionAnchorData.data() + passOffset.start, passOffset.size);
                            solvePositionConstraints(collisionSlice, anchorSlice);
                        }
                        else
                        {
                            solveVelocityConstraints(collisionSlice);
                        }
                    }

                    // Decrease atomic counter; last one to finish advances the wave and wakes everyone.
                    const uint32_t remaining = workerResources.workNotDone.fetch_sub(1, std::memory_order_acq_rel) - 1;
                    if (remaining == 0)
                    {
                        workerResources.workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);
                        workerResources.currentWaveTicket.fetch_add(1, std::memory_order_release);
                        workerResources.currentWaveTicket.notify_all();
                        localTicket++;
                    }
                    else
                    {
                        // Wait for new wave.
                        for (int i = 0; i < 2000; i++)
                        {
                            SPIN_PAUSE();
                            if (workerResources.currentWaveTicket.load(std::memory_order_acquire) != localTicket)
                                break;
                        }
                        if (workerResources.currentWaveTicket.load(std::memory_order_acquire) == localTicket)
                        {
                            workerResources.currentWaveTicket.wait(localTicket, std::memory_order_acquire);
                        }
                        localTicket = workerResources.currentWaveTicket.load(std::memory_order_acquire);
                    }

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
}