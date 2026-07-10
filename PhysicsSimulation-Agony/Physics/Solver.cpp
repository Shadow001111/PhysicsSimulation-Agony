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

        total += getVectorMemoryUsage(positionConstraintContainer);
        total += getVectorMemoryUsage(velocityConstraintContainer);
        total += getVectorMemoryUsage(frictionDataContainer);

        total += getVectorMemoryUsage(orderedCollisionData);

        return total;
    }

    void Solver::solve(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        TRACY_SCOPE_NC("Solve constraints (Single-threaded)", Ecstasy::Color::OliveDrab);

        computeConstantData(narrowPhaseCollisions);

        for (uint32_t i = 0; i < velocityIterations; i++)
        {
            solveVelocityConstraints(narrowPhaseCollisions, velocityConstraintContainer, frictionDataContainer);
        }
        for (uint32_t i = 0; i < positionIterations; i++)
        {
            solvePositionConstraints(narrowPhaseCollisions, positionConstraintContainer);
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
        {
            TRACY_SCOPE_N("Reorder collision data");

            const auto& flatIndices = solvingPlanner.getFlatIndices();
            const size_t mappedCount = flatIndices.size();
            const size_t* ECSTASY_RESTRICT flatIndicesPtr = flatIndices.data();

            orderedCollisionData.resize(mappedCount);
            for (size_t i = 0; i < mappedCount; i++)
            {
                const size_t originalIndex = flatIndicesPtr[i];
                orderedCollisionData[i] = narrowPhaseCollisions[originalIndex];
            }
        }
        computeConstantData(orderedCollisionData);

        solveConstraintsThreaded(orderedCollisionData, velocityIterations, positionIterations);
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

    void Solver::computeConstantData(const std::vector<BodyCollisionData>& collisionDataContainer)
    {
        TRACY_SCOPE_NC("Compute constant data", Ecstasy::Color::Chocolate);

        Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
        const MaterialIndex* ECSTASY_RESTRICT materialIndexPtr = bodies->materialIndex.data();
        const Material* ECSTASY_RESTRICT materialPtr = materials->data();

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

        // Resize containers.
        const size_t collisionCount = collisionDataContainer.size();
        positionConstraintContainer.resize(collisionCount);
        velocityConstraintContainer.resize(collisionCount);
        frictionDataContainer.resize(collisionCount);

        const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

        // Compute constant data.
        for (size_t c = 0; c < collisionCount; c++)
        {
            const auto& data = collisionDataPtr[c];
            const Vec2 centerOfMassA = getCenterOfMass(data.bodyA);
            const Vec2 centerOfMassB = getCenterOfMass(data.bodyB);
            {
                const Vec2 contactPoint = data.contactPoints[0]; // Only the first point is used.

                positionConstraintContainer[c].localAnchorA = invRotate(contactPoint - centerOfMassA, rotationCosPtr[data.bodyA], rotationSinPtr[data.bodyA]);
                positionConstraintContainer[c].localAnchorB = invRotate(contactPoint - centerOfMassB, rotationCosPtr[data.bodyB], rotationSinPtr[data.bodyB]);
            }
            {
                auto& velocityConstraint = velocityConstraintContainer[c];
                auto& frictionData = frictionDataContainer[c];

                const MaterialIndex materialIndexA = materialIndexPtr[data.bodyA];
                const MaterialIndex materialIndexB = materialIndexPtr[data.bodyB];
                const Material* materialA = materialPtr + materialIndexA;
                const Material* materialB = materialPtr + materialIndexB;

                const Real elasticityPlusOne = (materialA->elasticity + materialB->elasticity) * Real(0.5) + Real(1.0);
                frictionData.staticFriction  = std::sqrt(std::fmax(Real(0), materialA->staticFriction  * materialB->staticFriction));
                frictionData.dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

                const Real invMassA = invMassPtr[data.bodyA];
                const Real invMassB = invMassPtr[data.bodyB];
                const Real invInertiaA = invInertiaPtr[data.bodyA];
                const Real invInertiaB = invInertiaPtr[data.bodyB];
                const Real totalInvMass = invMassA + invMassB;

                const Vec2 normal = data.normal;
                const Vec2 tangent{ -normal.y, normal.x };

                for (uint32_t i = 0; i < data.contactCount; i++)
                {
                    const Vec2 contactPoint = data.contactPoints[i];
                    auto& point = velocityConstraint.points[i];

                    const Vec2 rA = contactPoint - centerOfMassA;
                    const Vec2 rB = contactPoint - centerOfMassB;
                    point.rAPerp = { -rA.y, rA.x };
                    point.rBPerp = { -rB.y, rB.x };

                    const Real rAPerpDotN = glm::dot(point.rAPerp, normal);
                    const Real rBPerpDotN = glm::dot(point.rBPerp, normal);
                    const Real normalDenom = totalInvMass
                        + rAPerpDotN * rAPerpDotN * invInertiaA
                        + rBPerpDotN * rBPerpDotN * invInertiaB;
                    point.normalMassXElasticityFactor = normalDenom > Real(0) ? elasticityPlusOne / normalDenom : Real(0);

                    const Real rAPerpDotT = glm::dot(point.rAPerp, tangent);
                    const Real rBPerpDotT = glm::dot(point.rBPerp, tangent);
                    const Real tangentDenom = totalInvMass
                        + rAPerpDotT * rAPerpDotT * invInertiaA
                        + rBPerpDotT * rBPerpDotT * invInertiaB;
                    point.tangentMass = tangentDenom > Real(0) ? Real(1) / tangentDenom : Real(0);
                }
            }
        }
    }

    void Solver::solveVelocityConstraints(
        std::span<const BodyCollisionData> collisionDataContainer,
        std::span<const VelocityConstraintData> constraintDataContainer,
        std::span<const FrictionData> frictionDataContainer
    )
    {
        TRACY_SCOPE_NC("Solve velocity constraints", Ecstasy::Color::Violet);

        constexpr Real frictionEpsilonSq = Real(1e-3 * 1e-3);

        // Get pointers.
        Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

        // Lambdas.
        auto getLinearVelocity = [&](const BodyIndex& bodyIndex) -> Vec2
            {
                const Real velocityX = velocityXPtr[bodyIndex];
                const Real velocityY = velocityYPtr[bodyIndex];
                return { velocityX, velocityY };
            };

        // Main loop.
        const size_t collisionCount = collisionDataContainer.size();
        for (size_t c = 0; c < collisionCount; c++)
        {
            const BodyCollisionData& collisionData = collisionDataContainer[c];
            const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

            // Get body indices.
            const BodyIndex bodyIndexA = collisionData.bodyA;
            const BodyIndex bodyIndexB = collisionData.bodyB;

            // Get inv masses.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            //
            const Real invInertiaA = invInertiaPtr[bodyIndexA];
            const Real invInertiaB = invInertiaPtr[bodyIndexB];

            Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
            Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
            Real angularVelocityA = angularVelocityPtr[bodyIndexA];
            Real angularVelocityB = angularVelocityPtr[bodyIndexB];

            const Vec2 normal = collisionData.normal;

            std::array<Real, 2> jnArray{};
            const uint32_t contactCount = collisionData.contactCount; // std::min(data.contactCount, 2u);

            // Calculate collision impulses and apply them.
            bool noContacts = true;
            for (uint32_t i = 0; i < contactCount; i++)
            {
                const auto& contactData = velocityConstraintData.points[i];

                const Vec2 rAPerp = contactData.rAPerp;
                const Vec2 rBPerp = contactData.rBPerp;

                const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
                const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

                const Vec2 relativeVelocity =
                    (linearVelocityB + angularLinearVelB) -
                    (linearVelocityA + angularLinearVelA);

                const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

                if (velocityAlongNormal > Real(0)) continue;

                const Real jn = -velocityAlongNormal * contactData.normalMassXElasticityFactor;

                const Vec2 impulse = jn * normal;
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

            // Calculate friction impulses and apply them.
            const FrictionData& frictionData = frictionDataContainer[c];
            const Real staticFriction  = frictionData.staticFriction;
            const Real dynamicFriction = frictionData.dynamicFriction;
            for (uint32_t i = 0; i < contactCount; i++)
            {
                const auto& contactData = velocityConstraintData.points[i];

                const Vec2 rAPerp = contactData.rAPerp;
                const Vec2 rBPerp = contactData.rBPerp;

                const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
                const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

                const Vec2 relativeVelocity =
                    (linearVelocityB + angularLinearVelB) -
                    (linearVelocityA + angularLinearVelA);

                Vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
                const Real tangentLengthSq = glm::dot(tangent, tangent);
                if (tangentLengthSq < frictionEpsilonSq) continue;

                tangent /= std::sqrt(tangentLengthSq);

                const Real jt = glm::dot(relativeVelocity, tangent) * contactData.tangentMass;

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
        std::span<const BodyCollisionData> collisionDataContainer,
        std::span<const PositionConstraintData> constraintDataContainer
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

        const size_t collisionCount = collisionDataContainer.size();

        // Note: I tried to use Simd, it was slower, probably because of the gather-scatter, or just memory intensive.
        //       plus it was invalid because same body index could appear multiple times in simd batch (on same worker).
        //       remaking solving planner to account for that would be hell!

        for (size_t c = 0; c < collisionCount; c++)
        {
            const auto& data = collisionDataContainer[c];
            const BodyIndex bodyIndexA = data.bodyA;
            const BodyIndex bodyIndexB = data.bodyB;

            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];
            const Real totalInvMass = invMassA + invMassB;

            // Re-derive the world anchors from the current (possibly already corrected) transforms.
            const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
            const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

            const PositionConstraintData& positionConstraintData = constraintDataContainer[c];
            const Vec2 worldAnchorA = centerOfMassA + rotate(positionConstraintData.localAnchorA, rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
            const Vec2 worldAnchorB = centerOfMassB + rotate(positionConstraintData.localAnchorB, rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

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
        const std::vector<BodyCollisionData>& collisionDataContainer,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t workerCount = solvingPlanner.getWorkerCount();

        const auto& passOffsets = solvingPlanner.getPassOffsets();
        const size_t waveCount = passOffsets.size() / workerCount;
        const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

        if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0)) [[unlikely]]
        {
            return;
        }

        const uint32_t positionSolvingStartTick = static_cast<uint32_t>(waveCount) * velocityIterations;
        const uint32_t totalTicks = positionSolvingStartTick + static_cast<uint32_t>(waveCount) * positionIterations;

        const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

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
                        std::span<const BodyCollisionData> collisionSlice(collisionDataPtr + passOffset.start, passOffset.size);

                        if (localTicket >= positionSolvingStartTick)
                        {
                            std::span<const PositionConstraintData> constraintSlice(positionConstraintContainer.data() + passOffset.start, passOffset.size);
                            solvePositionConstraints(collisionSlice, constraintSlice);
                        }
                        else
                        {
                            std::span<const VelocityConstraintData> constraintSlice(velocityConstraintContainer.data() + passOffset.start, passOffset.size);
                            std::span<const FrictionData> frictionDataSlice(frictionDataContainer.data() + passOffset.start, passOffset.size);
                            solveVelocityConstraints(collisionSlice, constraintSlice, frictionDataSlice);
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