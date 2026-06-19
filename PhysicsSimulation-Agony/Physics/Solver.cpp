#include "Solver.h"

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
            Vec2 impulseArray[2] = { Vec2(),  Vec2() };
            Vec2 rAPerpArray[2] = { Vec2(),  Vec2() };
            Vec2 rBPerpArray[2] = { Vec2(),  Vec2() };
            Real jnArray[2] = { Real(0), Real(0) };
            const uint32_t contactCount = data.contactCount;//  std::min(data.contactCount, 2u);

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
            if (SimulationSettings::ENABLE_VELOCITY_CORRECTION)
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
        if (narrowPhaseCollisions.size() < 5000)
        {
            resolveCollisions(narrowPhaseCollisions);
            return;
        }

        TRACY_SCOPE_NC("Resolve collisions (Threaded)", Ecstasy::Color::Purple);

        // TODO: (for future) We can create a lot of staging buffers, not waiting for workers to finish.
        // TODO: Push buffer immediately after worker took it.
        // TODO: Double buffering. Though then maybe some indices may interfere, that means workers need to work on same 'page'.

        constexpr auto WORKER_COUNT = ResolveCollisionsThreadedResources::WORKER_COUNT;
        constexpr auto MAX_VALID_INDICES_PER_PASS = ResolveCollisionsThreadedResources::MAX_VALID_INDICES_PER_PASS;

        using UsedSlot = ResolveCollisionsThreadedResources::UsedSlot;

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
            for (size_t i = 0; i < WORKER_COUNT; i++)
            {
                auto& wData = solverResources.workerData[i];
                wData.isDestroyed.wait(false, std::memory_order_acquire);
            }
        }
        for (auto& w : solverResources.workerData)
        {
            w.indices.clear();
            w.isProcessing.store(false, std::memory_order_relaxed);
            w.isDestroyed.store(false, std::memory_order_relaxed);
        }

        //
        for (auto& pass : solverResources.stagingPasses)
        {
            pass.clear();
        }
        solverResources.usedBodies.resize(bodies->getCount());

        std::atomic<uint32_t> workNotDone{ 0 };

        // Lambdas.
        auto workerFunc = [this, &narrowPhaseCollisions, &workNotDone](size_t workerIndex)
            {
                auto& wData = solverResources.workerData[workerIndex];
                try
                {
                    while (true)
                    {
                        // Wait for data to arrive.
                        wData.isProcessing.wait(false, std::memory_order_acquire);

                        // Check for stop.
                        if (wData.indices.empty()) break;

                        // Execute.
                        resolveCollisionsIndirect(narrowPhaseCollisions, wData.indices);

                        workNotDone.fetch_sub(1, std::memory_order_release);
                        workNotDone.notify_one();

                        wData.indices.clear();

                        // Notify main thread that worker is finished.
                        wData.isProcessing.store(false, std::memory_order_release);
                        wData.isProcessing.notify_one();
                    }
                    wData.isProcessing.store(false, std::memory_order_release);
                    wData.isProcessing.notify_one();
                }
                catch (const std::exception& e)
                {
                    std::cout << "Resolve collisions worker caught an exception: " << e.what() << "\n";
                    wData.isProcessing.store(false, std::memory_order_release);
                    wData.isProcessing.notify_one();
                }
                wData.isDestroyed.store(true, std::memory_order_release);
                wData.isDestroyed.notify_one();
            };

        // Launch workers.
        auto& threadPool = Threading::getGlobalThreadPool();

        for (size_t i = 0; i < WORKER_COUNT; i++)
        {
            threadPool.enqueue(workerFunc, i);
        }

        // Main loop.
        size_t maxAllowedStages = WORKER_COUNT;
        while (true)
        {
            // Clear body-using history.
            {
                TRACY_SCOPE_NC("Clear body-using history", Ecstasy::Color::Pink);
                std::fill(
                    solverResources.usedBodies.begin(),
                    solverResources.usedBodies.end(),
                    UsedSlot(-1)
                );
            }

            // Coloring.
            {
                TRACY_SCOPE_NC("Coloring", Ecstasy::Color::Blue);
                size_t startReadPos = 0;
                for (size_t stageIndex = 0; stageIndex < maxAllowedStages; stageIndex++)
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
                    }

                    // Check.
                    if (stagingPass.empty())
                    {
                        maxAllowedStages = stageIndex;
                        break;
                    }
                }
            }

            // Wait for previous wave to finish.
            {
                TRACY_SCOPE_NC("Wait for wave end", Ecstasy::Color::Brown);

                while (uint32_t val = workNotDone.load(std::memory_order_acquire) != 0)
                {
                    workNotDone.wait(val, std::memory_order_acquire);
                }
            }

            for (size_t stageIndex = 0; stageIndex < maxAllowedStages; stageIndex++)
            {
                auto& stagingPass = solverResources.stagingPasses[stageIndex];

                // Push staging pass.
                {
                    TRACY_SCOPE_NC("Push staging pass", Ecstasy::Color::Gold);

                    auto& wData = solverResources.workerData[stageIndex];

                    // Wait for worker to finish. Doesn't take too long.
                    wData.isProcessing.wait(true, std::memory_order_acquire);

                    // Push staging pass.
                    wData.indices.swap(stagingPass); // Staging pass is cleared in worker thread.

                    // Notify worker that data is ready.
                    workNotDone.fetch_add(1, std::memory_order_release);
                    wData.isProcessing.store(true, std::memory_order_release);
                    wData.isProcessing.notify_one();
                };
            }

            // Check.
            if (solverResources.remainingIndices.empty() || maxAllowedStages < 2) break;
        }

        // Wait for workers to finish and stop them.
        {
            TRACY_SCOPE_NC("Wait for workers to finish", Ecstasy::Color::Brown);
            for (size_t i = 0; i < WORKER_COUNT; i++)
            {
                auto& wData = solverResources.workerData[i];
                wData.isProcessing.wait(true, std::memory_order_acquire);

                // Stop. Call with empty indices array will stop worker.
                wData.isProcessing.store(true, std::memory_order_release);
                wData.isProcessing.notify_one();
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
            Vec2 impulseArray[2] = { Vec2(),  Vec2() };
            Vec2 rAPerpArray[2] = { Vec2(),  Vec2() };
            Vec2 rBPerpArray[2] = { Vec2(),  Vec2() };
            Real jnArray[2] = { Real(0), Real(0) };
            const uint32_t contactCount = data.contactCount;//  std::min(data.contactCount, 2u);

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
            if (SimulationSettings::ENABLE_VELOCITY_CORRECTION)
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