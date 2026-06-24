#include "Solver.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <numeric>
#include <iostream>

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

        total += getVectorMemoryUsage(solverResources.remainingIndices);
        for (auto& pass : solverResources.stagingPasses)
        {
            total += getVectorMemoryUsage(pass);
        }
        total += getVectorMemoryUsage(solverResources.usedBodies);
        for (const auto& wData : solverResources.workerData)
        {
            total += getVectorMemoryUsage(wData.indices);
        }

        return total;
    }

    void Solver::resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        TRACY_SCOPE_NC("Resolve collisions", Ecstasy::Color::Violet);

        constexpr Real frictionEpsilonSq = Real(1e-3 * 1e-3);

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies->positionX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->positionY.data();

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
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];

                bool noContacts = true;

                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 contactPoint = data.contacts[i];

                    const Vec2 rA = contactPoint - centerOfMassA;
                    const Vec2 rB = contactPoint - centerOfMassB;

                    const Vec2 rAPerp = { -rA.y, rA.x };
                    const Vec2 rBPerp = { -rB.y, rB.x };

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

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
                {
                    const Vec2 linearVelocityChangeA = impulseSum * invMassA;
                    velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
                    velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;

                    const Real angularVelocityChangeA = (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;
                    angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
                }

                {
                    const Vec2 linearVelocityChangeB = impulseSum * invMassB;
                    velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
                    velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;

                    const Real angularVelocityChangeB = (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                    angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
                }
            }

            // Calculate friction impulses.
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];
                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 rAPerp = rAPerpArray[i];
                    const Vec2 rBPerp = rBPerpArray[i];

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

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
            //{
            //    const Vec2 impulseSum = impulseArray[0] + impulseArray[1];
            //    {
            //        const Vec2 linearVelocityChangeA = impulseSum * invMassA;
            //        velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
            //        velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;
            //
            //        const Real angularVelocityChangeA = (
            //            glm::dot(rAPerpArray[0], impulseArray[0]) +
            //            glm::dot(rAPerpArray[1], impulseArray[1])
            //            ) * invInertiaA;
            //        angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
            //    }
            //
            //    {
            //        const Vec2 linearVelocityChangeB = impulseSum * invMassB;
            //        velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
            //        velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;
            //
            //        const Real angularVelocityChangeB = (
            //            glm::dot(rBPerpArray[0], impulseArray[0]) +
            //            glm::dot(rBPerpArray[1], impulseArray[1])
            //            ) * invInertiaB;
            //        angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
            //    }
            //}

            // Position and velocity correction.
            const Real invTotalInvMass_x_Depth = depth / totalInvMass;
            const Real correctionStrengthA = invMassA * invTotalInvMass_x_Depth;
            const Real correctionStrengthB = invMassB * invTotalInvMass_x_Depth;
            {
                const Real correctionA = correctionStrengthA * simulationSettings.positionCorrectionPercent;
                const Real correctionB = correctionStrengthB * simulationSettings.positionCorrectionPercent;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                positionXPtr[bodyIndexA] -= correctionAVec.x;
                positionYPtr[bodyIndexA] -= correctionAVec.y;
                positionXPtr[bodyIndexB] += correctionBVec.x;
                positionYPtr[bodyIndexB] += correctionBVec.y;
            }
            if constexpr (SimulationSettings::ENABLE_VELOCITY_CORRECTION)
            {
                const Real correctionA = correctionStrengthA * simulationSettings.velocityCorrectionStrength;
                const Real correctionB = correctionStrengthB * simulationSettings.velocityCorrectionStrength;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                velocityXPtr[bodyIndexA] -= correctionAVec.x;
                velocityYPtr[bodyIndexA] -= correctionAVec.y;
                velocityXPtr[bodyIndexB] += correctionBVec.x;
                velocityYPtr[bodyIndexB] += correctionBVec.y;
            }
        }
    }

    void Solver::resolveCollisionsThreadedGraphColoring(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        constexpr size_t COLLISION_COUNT_PER_WORKER = 830;
        constexpr size_t MIN_COLLISION_COUNT_FOR_THREADING = COLLISION_COUNT_PER_WORKER * 3;
        constexpr size_t MAX_VALID_INDICES_PER_WORKER = 256;

        static_assert(MIN_COLLISION_COUNT_FOR_THREADING >= MAX_VALID_INDICES_PER_WORKER);

        // Single-threaded path.
        if (narrowPhaseCollisions.size() < MIN_COLLISION_COUNT_FOR_THREADING)
        {
            resolveCollisions(narrowPhaseCollisions);
            return;
        }

        // Multi-threading path.
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t availableWorkerCount = threadPool.getThreadCount() - 1; // One less to not share logical core with main thread (depends on scheduler).
        const size_t neededWorkerCount = narrowPhaseCollisions.size() / COLLISION_COUNT_PER_WORKER;

        const size_t workerCount = std::min(availableWorkerCount, neededWorkerCount);

        TRACY_SCOPE_NC("Resolve collisions (Threaded)", Ecstasy::Color::Purple);

        // Fill remaining indices.
        const size_t collisionCount = narrowPhaseCollisions.size();
        solverResources.remainingIndices.resize(collisionCount);
        {
            TRACY_SCOPE_NC("Fill indices", Ecstasy::Color::Red);
            std::iota(
                solverResources.remainingIndices.begin(),
                solverResources.remainingIndices.end(),
                0ull
            );
        }

        // Worker data.
        {
            TRACY_SCOPE_NC("Wait for workers to get destroyed", Ecstasy::Color::Gray);
            for (auto& wData : solverResources.workerData)
            {
                wData.isDestroyed.wait(false, std::memory_order_acquire);
            }
        }
        solverResources.workerData.resize(workerCount);
        for (auto& w : solverResources.workerData)
        {
            w.indices.clear();
            w.isDestroyed.store(false, std::memory_order_release);
            w.workWave.store(0, std::memory_order_release);
        }

        //
        solverResources.stagingPasses.resize(workerCount + 1);
        for (auto& pass : solverResources.stagingPasses)
        {
            pass.clear();
            pass.reserve(MAX_VALID_INDICES_PER_WORKER);
        }
        solverResources.usedBodies.resize(bodies->getCount());

        solverResources.workNotDone.store(0, std::memory_order_release);

        // Lambdas.
        auto workerFunc = [&](size_t workerIndex)
            {
                auto& wData = solverResources.workerData[workerIndex];
                
                uint32_t previousWave = 0;
                while (true)
                {
                    // Stop.
                    if (previousWave == ResolveCollisionsThreadedResources::STOP_WAVE) break;

                    // Wait for data next wave.
                    wData.workWave.wait(previousWave, std::memory_order_acquire);
                    previousWave = wData.workWave.load(std::memory_order_acquire);

                    // If workers will receive stop signal, they still must to execute their work and only then stop.

                    // Check for stop.
                    if (wData.indices.empty()) continue;

                    // Execute.
                    resolveCollisionsIndirect(narrowPhaseCollisions, wData.indices);

                    wData.indices.clear();

                    auto wND = solverResources.workNotDone.fetch_sub(1, std::memory_order_release) - 1;
                    if (wND == 0)
                    {
                        solverResources.workNotDone.notify_one();
                    }
                }

                wData.isDestroyed.store(true, std::memory_order_release);
                wData.isDestroyed.notify_one();
            };

        for (size_t i = 0; i < workerCount; i++)
        {
            threadPool.enqueue(workerFunc, i);
        }

        // Main loop.
        auto& mainThreadPass = solverResources.stagingPasses[0];
        while (true)
        {
            // Clear body-using history.
            std::fill(
                solverResources.usedBodies.begin(),
                solverResources.usedBodies.end(),
                ResolveCollisionsThreadedResources::UsedSlot(-1) // Unsigned.
            );

            // Coloring.
            size_t workerEnableCount = 0;
            {
                TRACY_SCOPE_NC("Coloring", Ecstasy::Color::Blue);

                size_t readSize = solverResources.remainingIndices.size();
                size_t currentStageIndex = 0;
                for (size_t readPos = 0; readPos < readSize;)
                {
                    // Get collision data at index.
                    const size_t collisionIndex = solverResources.remainingIndices[readPos];
                    const auto& collisionData = narrowPhaseCollisions[collisionIndex];

                    // Get body indices.
                    const BodyIndex bodyIndexA = collisionData.bodyA;
                    const BodyIndex bodyIndexB = collisionData.bodyB;

                    // If body is used by previous stages, skip.
                    if (
                        solverResources.usedBodies[bodyIndexA] < currentStageIndex ||
                        solverResources.usedBodies[bodyIndexB] < currentStageIndex
                        )
                    {
                        readPos++;
                        continue;
                    }

                    // Mark bodies as used by current stage.
                    solverResources.usedBodies[bodyIndexA] = currentStageIndex;
                    solverResources.usedBodies[bodyIndexB] = currentStageIndex;

                    // Push index to staging pass.
                    auto& stagingPass = solverResources.stagingPasses[currentStageIndex];
                    stagingPass.push_back(collisionIndex);

                    // Remove index from remaining indices.
                    solverResources.remainingIndices[readPos] = solverResources.remainingIndices.back();
                    solverResources.remainingIndices.pop_back();
                    readSize--;

                    // Advance to next stage or stop.
                    if (stagingPass.size() >= MAX_VALID_INDICES_PER_WORKER)
                    {
                        currentStageIndex++;
                        if (currentStageIndex >= workerCount + 1)
                        {
                            break;
                        }
                    }
                }

                // Count valid passes.
                for (size_t i = 0; i <= workerCount; i++)
                {
                    auto& stagingPass = solverResources.stagingPasses[i];
                    if (stagingPass.empty()) break;
                    workerEnableCount++;
                }
            }
            if (workerEnableCount == 0) [[unlikely]]
            {
                for (size_t stageIndex = 1; stageIndex <= workerCount; stageIndex++)
                {
                    auto& wData = solverResources.workerData[stageIndex - 1];

                    wData.workWave.store(ResolveCollisionsThreadedResources::STOP_WAVE, std::memory_order_release);
                    wData.workWave.notify_one();
                }
                break; // Exit main loop.
            }

            // Resolve collisions on main thread, while wave is being executed.
            resolveCollisionsIndirect(narrowPhaseCollisions, mainThreadPass);
            mainThreadPass.clear();

            // Wait for previous wave end.
            {
                TRACY_SCOPE_NC("Wait for wave end", Ecstasy::Color::Brown);

                while (true)
                {
                    uint32_t val = solverResources.workNotDone.load(std::memory_order_acquire);
                    if (val == 0) break;
                    solverResources.workNotDone.wait(val, std::memory_order_acquire);
                }
            }

            // Push staging passes.
            const bool stop = solverResources.remainingIndices.size() <= MIN_COLLISION_COUNT_FOR_THREADING;
            if (stop)
            {
                TRACY_SCOPE_NC("Push staging passes", Ecstasy::Color::Gold);

                solverResources.workNotDone.fetch_add(workerEnableCount - 1, std::memory_order_release);
                for (size_t stageIndex = 1; stageIndex <= workerCount; stageIndex++)
                {
                    auto& wData = solverResources.workerData[stageIndex - 1];
                    auto& stagingPass = solverResources.stagingPasses[stageIndex];
                    wData.indices.swap(stagingPass);

                    wData.workWave.store(ResolveCollisionsThreadedResources::STOP_WAVE, std::memory_order_release);
                    wData.workWave.notify_one();
                }
                break; // Exit main loop.
            }
            else
            {
                TRACY_SCOPE_NC("Push staging passes", Ecstasy::Color::Gold);

                solverResources.workNotDone.fetch_add(workerEnableCount - 1, std::memory_order_release);
                for (size_t stageIndex = 1; stageIndex < workerEnableCount; stageIndex++)
                {
                    auto& wData = solverResources.workerData[stageIndex - 1];
                    auto& stagingPass = solverResources.stagingPasses[stageIndex];
                    wData.indices.swap(stagingPass); // Staging pass is cleared in worker thread.

                    wData.workWave.fetch_add(1, std::memory_order_release);
                    wData.workWave.notify_one();
                }
            }
        }

        // Wait for last wave end.
        {
            TRACY_SCOPE_NC("Wait for wave end", Ecstasy::Color::Brown);
            while (true)
            {
                uint32_t val = solverResources.workNotDone.load(std::memory_order_acquire);
                if (val == 0) break;
                solverResources.workNotDone.wait(val, std::memory_order_acquire);
            }
        }

        // Execute remaining on main thread.
        if (!solverResources.remainingIndices.empty())
        {
            resolveCollisionsIndirect(narrowPhaseCollisions, solverResources.remainingIndices);
        }
    }

    void Solver::resolveCollisionsIndirect(const std::vector<BodyCollisionData>& narrowPhaseCollisions, const std::vector<size_t>& collisionIndices)
    {
        TRACY_SCOPE_NC("Resolve collisions (Indirect)", Ecstasy::Color::HotPink);

        constexpr Real frictionEpsilonSq = Real(1e-3 * 1e-3);

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies->positionX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->positionY.data();

        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();

        Real* ECSTASY_RESTRICT velocityXPtr =        bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr =        bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr =  bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr =    bodies->invMass.data();
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
        for (const size_t dataIndex : collisionIndices)
        {
            const auto& data = narrowPhaseCollisions[dataIndex];

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
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];

                bool noContacts = true;

                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 contactPoint = data.contacts[i];

                    const Vec2 rA = contactPoint - centerOfMassA;
                    const Vec2 rB = contactPoint - centerOfMassB;

                    const Vec2 rAPerp = { -rA.y, rA.x };
                    const Vec2 rBPerp = { -rB.y, rB.x };

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

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
                {
                    const Vec2 linearVelocityChangeA = impulseSum * invMassA;
                    velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
                    velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;

                    const Real angularVelocityChangeA = (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;
                    angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
                }

                {
                    const Vec2 linearVelocityChangeB = impulseSum * invMassB;
                    velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
                    velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;

                    const Real angularVelocityChangeB = (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                    angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
                }
            }

            // Calculate friction impulses.
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];
                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 rAPerp = rAPerpArray[i];
                    const Vec2 rBPerp = rBPerpArray[i];

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

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
                {
                    const Vec2 linearVelocityChangeA = impulseSum * invMassA;
                    velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
                    velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;

                    const Real angularVelocityChangeA = (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;
                    angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
                }

                {
                    const Vec2 linearVelocityChangeB = impulseSum * invMassB;
                    velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
                    velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;

                    const Real angularVelocityChangeB = (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                    angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
                }
            }

            // Position and velocity correction.
            const Real invTotalInvMass_x_Depth = depth / totalInvMass;
            const Real correctionStrengthA = invMassA * invTotalInvMass_x_Depth;
            const Real correctionStrengthB = invMassB * invTotalInvMass_x_Depth;
            {
                const Real correctionA = correctionStrengthA * simulationSettings.positionCorrectionPercent;
                const Real correctionB = correctionStrengthB * simulationSettings.positionCorrectionPercent;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                positionXPtr[bodyIndexA] -= correctionAVec.x;
                positionYPtr[bodyIndexA] -= correctionAVec.y;
                positionXPtr[bodyIndexB] += correctionBVec.x;
                positionYPtr[bodyIndexB] += correctionBVec.y;
            }
            if constexpr (SimulationSettings::ENABLE_VELOCITY_CORRECTION)
            {
                const Real correctionA = correctionStrengthA * simulationSettings.velocityCorrectionStrength;
                const Real correctionB = correctionStrengthB * simulationSettings.velocityCorrectionStrength;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                velocityXPtr[bodyIndexA] -= correctionAVec.x;
                velocityYPtr[bodyIndexA] -= correctionAVec.y;
                velocityXPtr[bodyIndexB] += correctionBVec.x;
                velocityYPtr[bodyIndexB] += correctionBVec.y;
            }
        }
    }
}