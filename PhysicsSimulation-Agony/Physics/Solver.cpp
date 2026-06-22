#include "Solver.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <numeric>

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
        auto& threadPool = Threading::getGlobalThreadPool();

        const size_t availableWorkerCount = threadPool.getThreadCount();// -1;
        const size_t neededWorkerCount = narrowPhaseCollisions.size() * 6 / 5000;

        const size_t workerCount = std::min(availableWorkerCount, neededWorkerCount);

        if (narrowPhaseCollisions.size() < 5000 || workerCount <= 1)
        {
            resolveCollisions(narrowPhaseCollisions);
            return;
        }

        TRACY_SCOPE_NC("Resolve collisions (Threaded)", Ecstasy::Color::Purple);

        constexpr auto MAX_VALID_INDICES_PER_PASS = ResolveCollisionsThreadedResources::MAX_VALID_INDICES_PER_PASS;

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
            w.isDestroyed.store(false, std::memory_order_relaxed);
        }

        //
        solverResources.stagingPasses.resize(workerCount + 1);
        for (auto& pass : solverResources.stagingPasses)
        {
            pass.clear();
        }
        solverResources.usedBodies.resize(bodies->getCount());

        std::atomic<uint32_t> workNotDone{ 0 };
        solverResources.workWave.store(0, std::memory_order_release);

        // Lambdas.
        auto workerFunc = [&](size_t workerIndex)
            {
                auto& wData = solverResources.workerData[workerIndex];
                try
                {
                    uint32_t previousWave = 0;
                    while (true)
                    {
                        // Wait for data next wave.
                        solverResources.workWave.wait(previousWave, std::memory_order_acquire);
                        previousWave = solverResources.workWave.load(std::memory_order_acquire);

                        // Check for stop.
                        if (previousWave == ResolveCollisionsThreadedResources::STOP_WAVE) break;
                        if (wData.indices.empty()) continue;

                        // Execute.
                        resolveCollisionsIndirect(narrowPhaseCollisions, wData.indices);

                        wData.indices.clear();

                        auto wND = workNotDone.fetch_sub(1, std::memory_order_release) - 1;
                        if (wND == 0)
                        {
                            workNotDone.notify_one();
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    throw;
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
                ResolveCollisionsThreadedResources::UsedSlot(-1)
            );

            // Coloring.
            size_t workerEnableCount = 0;
            {
                TRACY_SCOPE_NC("Coloring", Ecstasy::Color::Blue);
                size_t startReadPos = 0;
                for (size_t stageIndex = 0; stageIndex <= workerCount; stageIndex++)
                {
                    auto& stagingPass = solverResources.stagingPasses[stageIndex];

                    // Coloring.
                    if (stageIndex == 0)
                    {
                        // Quick pass: no body conflicts at stage 0.
                        // Note: The order is different from else branch.

                        size_t readSize = std::min(MAX_VALID_INDICES_PER_PASS, solverResources.remainingIndices.size());

                        const auto beg = solverResources.remainingIndices.end() - readSize;

                        // Copy first indices.
                        stagingPass.insert(stagingPass.end(),
                            beg,
                            beg + readSize
                        );

                        // Remove taken indices from remainingIndices.
                        solverResources.remainingIndices.erase(
                            beg,
                            beg + readSize
                        );

                        // Mark indices as used.
                        for (const size_t idx : stagingPass)
                        {
                            const auto& coll = narrowPhaseCollisions[idx];
                            solverResources.usedBodies[coll.bodyA] = 0;
                            solverResources.usedBodies[coll.bodyB] = 0;
                        }
                    }
                    else
                    {
                        size_t untakenStart = 0;
                        bool foundSomething = false;
                        size_t readSize = solverResources.remainingIndices.size();
                        for (size_t readPos = startReadPos; readPos < readSize && stagingPass.size() < MAX_VALID_INDICES_PER_PASS;)
                        {
                            const size_t idx = solverResources.remainingIndices[readPos];
                            const auto& coll = narrowPhaseCollisions[idx];

                            if (
                                solverResources.usedBodies[coll.bodyA] < stageIndex ||
                                solverResources.usedBodies[coll.bodyB] < stageIndex
                                )
                            {
                                readPos++;
                                if (!foundSomething)
                                {
                                    untakenStart = readPos;
                                }
                                continue;
                            }
                            stagingPass.push_back(idx);
                            solverResources.usedBodies[coll.bodyA] = stageIndex;
                            solverResources.usedBodies[coll.bodyB] = stageIndex;

                            solverResources.remainingIndices[readPos] = solverResources.remainingIndices.back();
                            solverResources.remainingIndices.pop_back();
                            readSize--;

                            foundSomething = true;
                        }
                        if (untakenStart > 0)
                        {
                            startReadPos = untakenStart;
                        }

                        // Check.
                        if (stagingPass.empty())
                        {
                            break;
                        }
                    }

                    workerEnableCount++;
                }
            }

            // Early exit.
            if (workerEnableCount == 0) [[unlikely]]
            {
                break;
            }

            // Resolve collisions on main thread, while wave is being executed.
            if (!mainThreadPass.empty())
            {
                resolveCollisionsIndirect(narrowPhaseCollisions, mainThreadPass);
                mainThreadPass.clear();
            }

            // Wait for previous wave to finish.
            {
                TRACY_SCOPE_NC("Wait for wave end", Ecstasy::Color::Brown);

                while (true)
                {
                    uint32_t val = workNotDone.load(std::memory_order_acquire);
                    if (val == 0) break;
                    workNotDone.wait(val, std::memory_order_acquire);
                }
            }

            // Push staging passes.
            {
                TRACY_SCOPE_NC("Push staging passes", Ecstasy::Color::Gold);
                workNotDone.fetch_add(workerEnableCount - 1, std::memory_order_release);
                for (size_t stageIndex = 1; stageIndex < workerEnableCount; stageIndex++)
                {
                    auto& wData = solverResources.workerData[stageIndex - 1];
                    auto& stagingPass = solverResources.stagingPasses[stageIndex];
                    wData.indices.swap(stagingPass); // Staging pass is cleared in worker thread.
                }

                // Notify workers that data is ready.
                solverResources.workWave.fetch_add(1, std::memory_order_release);
                solverResources.workWave.notify_all();
            }

            // Check.
            if (solverResources.remainingIndices.empty() || workerEnableCount <= 2) break;
        }

        // Execute remaining on main thread.
        if (!solverResources.remainingIndices.empty())
        {
            resolveCollisionsIndirect(narrowPhaseCollisions, solverResources.remainingIndices);
        }

        // Wait for last wave to finish.
        {
            TRACY_SCOPE_NC("Wait for last wave end", Ecstasy::Color::Brown);

            while (true)
            {
                uint32_t val = workNotDone.load(std::memory_order_acquire);
                if (val == 0) break;
                workNotDone.wait(val, std::memory_order_acquire);
            }

            // Signal workers. With empty indices array they will stop.
            solverResources.workWave.store(ResolveCollisionsThreadedResources::STOP_WAVE, std::memory_order_release);
            solverResources.workWave.notify_all();
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