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
        const std::vector<Material>& materials,
        const SpringSoAViewer& springs
    )
    {
        this->bodies = &bodies;
        this->materials = &materials;
        this->springs = springs;
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

    void Solver::solveCollisions(
        const std::vector<BodyCollisionData>& narrowPhaseCollisions,
        uint32_t velocityIterations,
        uint32_t positionIterations
    )
    {
        const size_t workerCount = planCollisionSolvingWorkerCount(narrowPhaseCollisions.size());

        // Single-threaded path.
        if (workerCount <= 1)
        {
            TRACY_SCOPE_NC("Solve collisions (Single-threaded)", Ecstasy::Color::OliveDrab);

            computeConstantData(narrowPhaseCollisions);

            for (uint32_t i = 0; i < velocityIterations; i++)
            {
                solveCollisionVelocityConstraints(i == 0, narrowPhaseCollisions, velocityConstraintContainer, frictionDataContainer);
            }
            for (uint32_t i = 0; i < positionIterations; i++)
            {
                solveCollisionPositionConstraints(narrowPhaseCollisions, positionConstraintContainer);
            }
            return;
        }

        // Multi-threading path.
        TRACY_SCOPE_NC("Solve collisions (Multi-threaded)", Ecstasy::Color::OliveDrab);
        {
            TRACY_SCOPE_NC("Collect body pairs from collision data", Ecstasy::Color::Red);

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

        solveCollisionConstraintsThreaded(orderedCollisionData, velocityIterations, positionIterations);

        // Scatter persistent contact data back to the detector's original container.
        if (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
        {
            TRACY_SCOPE_N("Scatter persistent contact data back");
            const auto& flatIndices = solvingPlanner.getFlatIndices();
            const size_t mappedCount = flatIndices.size();
            for (size_t i = 0; i < mappedCount; i++)
            {
                const size_t originalIndex = flatIndices[i];
                // narrowPhaseCollisions is const&, but persistentContactData is mutable -> legal write.
                narrowPhaseCollisions[originalIndex].persistentContactData = orderedCollisionData[i].persistentContactData;
            }
        }
    }

    void Solver::solveSprings(Real deltaTime, uint32_t springIterations)
    {
        auto rotate = [](const Vec2& v, Real cos, Real sin) -> Vec2 {
            return { v.x * cos - v.y * sin, v.x * sin + v.y * cos };
            };

        TRACY_SCOPE_N("Solve springs");

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
        Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();

        const Real* ECSTASY_RESTRICT springStiffnessPtr = springs.stiffness;
        const Real* ECSTASY_RESTRICT springDampingPtr = springs.damping;
        const Real* ECSTASY_RESTRICT springRestLengthPtr = springs.restLength;

        // Main loop.
        const size_t springCount = springs.getCount();
        for (uint32_t iter = 0; iter < springIterations; iter++)
        {
            for (size_t i = 0; i < springCount; i++)
            {
                const BodyIndex bodyIndexA = springs.bodyIndexA[i];
                const BodyIndex bodyIndexB = springs.bodyIndexB[i];

                const Real invMassA = invMassPtr[bodyIndexA];
                const Real invMassB = invMassPtr[bodyIndexB];

                // Check if both attachments are static anchors.
                if (invMassA == Real(0) && invMassB == Real(0)) [[unlikely]] continue;

                const Real invInertiaA = invInertiaPtr[bodyIndexA];
                const Real invInertiaB = invInertiaPtr[bodyIndexB];

                // Compute center of masses.
                const Vec2 worldCOMA{ positionXPtr[bodyIndexA] + localCenterOfMassXPtr[bodyIndexA], positionYPtr[bodyIndexA] + localCenterOfMassYPtr[bodyIndexA] };
                const Vec2 worldCOMB{ positionXPtr[bodyIndexB] + localCenterOfMassXPtr[bodyIndexB], positionYPtr[bodyIndexB] + localCenterOfMassYPtr[bodyIndexB] };

                // Compute world-space anchors.
                const Vec2 rotatedAnchorA = rotate(springs.localAnchorA[i], rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
                const Vec2 rotatedAnchorB = rotate(springs.localAnchorB[i], rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

                const Vec2 worldAnchorA = worldCOMA + rotatedAnchorA;
                const Vec2 worldAnchorB = worldCOMB + rotatedAnchorB;

                // Compute current length and direction.
                const Vec2 delta = worldAnchorB - worldAnchorA;
                const Real currentLengthSq = glm::dot(delta, delta);
                if (currentLengthSq < Real(1e-8)) continue;

                const Real currentLength = glm::sqrt(currentLengthSq);
                const Vec2 dir = delta / currentLength;

                // Get velocities.
                Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
                Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
                Real angularVelocityA = angularVelocityPtr[bodyIndexA];
                Real angularVelocityB = angularVelocityPtr[bodyIndexB];

                // Compute linear velocities at points.
                const Vec2 angularLinearVelA = Vec2(-rotatedAnchorA.y, rotatedAnchorA.x) * angularVelocityA;
                const Vec2 angularLinearVelB = Vec2(-rotatedAnchorB.y, rotatedAnchorB.x) * angularVelocityB;

                const Vec2 relativeVelocity =
                    (linearVelocityB + angularLinearVelB) -
                    (linearVelocityA + angularLinearVelA);

                // Constraint error.
                const Real C = currentLength - springRestLengthPtr[i];
                const Real Cdot = glm::dot(relativeVelocity, dir);

                const Real raCn = (rotatedAnchorA.x * dir.y) - (rotatedAnchorA.y * dir.x);
                const Real rbCn = (rotatedAnchorB.x * dir.y) - (rotatedAnchorB.y * dir.x);
                
                // Compute effective mass.
                const Real effectiveMass = invMassA + invMassB + invInertiaA * raCn * raCn + invInertiaB * rbCn * rbCn;
                if (effectiveMass <= Real(0)) [[unlikely]] continue;

                // Soft constraint parameters from physical stiffness/damping.
                const Real k = springStiffnessPtr[i];
                const Real c = springDampingPtr[i];

                Real gamma = 0;
                Real beta = 0;
                {
                    const Real denom = deltaTime * (c + deltaTime * k);
                    if (denom > Real(1e-12)) [[likely]]
                    {
                        gamma = Real(1) / denom;
                        beta = deltaTime * k * gamma;
                    }
                }

                // Compute impulse.
                const Real impulseMag = (Cdot + beta * C) / (effectiveMass + gamma);
                const Vec2 impulse = dir * impulseMag;

                // Apply impulse.
                if (invMassA > Real(0))
                {
                    linearVelocityA += impulse * invMassA;

                    const Real torqueA = rotatedAnchorA.x * impulse.y - rotatedAnchorA.y * impulse.x;
                    angularVelocityA += torqueA * invInertiaA;

                    velocityXPtr[bodyIndexA] = linearVelocityA.x;
                    velocityYPtr[bodyIndexA] = linearVelocityA.y;

                    angularVelocityPtr[bodyIndexA] = angularVelocityA;
                }
                if (invMassB > Real(0))
                {
                    linearVelocityB -= impulse * invMassB;

                    const Real torqueB = rotatedAnchorB.x * impulse.y - rotatedAnchorB.y * impulse.x;
                    angularVelocityB -= torqueB * invInertiaB;

                    velocityXPtr[bodyIndexB] = linearVelocityB.x;
                    velocityYPtr[bodyIndexB] = linearVelocityB.y;

                    angularVelocityPtr[bodyIndexB] = angularVelocityB;
                }
            }
        }
    }

    size_t Solver::planCollisionSolvingWorkerCount(size_t collisionCount)
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

                velocityConstraint.points[0].rAPerp = Vec2();
                velocityConstraint.points[0].rBPerp = Vec2();
                velocityConstraint.points[1].rAPerp = Vec2();
                velocityConstraint.points[1].rBPerp = Vec2();

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

    void Solver::applyWarmStartingForCollisions(
        std::span<const BodyCollisionData> collisionDataContainer,
        std::span<const VelocityConstraintData> constraintDataContainer
    )
    {
        TRACY_SCOPE_NC("Apply warm starting for collisions", Ecstasy::Color::Violet);

        // Get pointers.
        Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

        // Main loop.
        const size_t collisionCount = collisionDataContainer.size();
        for (size_t c = 0; c < collisionCount; c++)
        {
            const BodyCollisionData& collisionData = collisionDataContainer[c];
            const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

            // Get body indices.
            const BodyIndex bodyIndexA = collisionData.bodyA;
            const BodyIndex bodyIndexB = collisionData.bodyB;

            // Get body data.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            const Real invInertiaA = invInertiaPtr[bodyIndexA];
            const Real invInertiaB = invInertiaPtr[bodyIndexB];

            Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
            Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
            Real angularVelocityA = angularVelocityPtr[bodyIndexA];
            Real angularVelocityB = angularVelocityPtr[bodyIndexB];

            //
            const Vec2 normal = collisionData.normal;
            const uint32_t contactCount = collisionData.contactCount;

            const Vec2 tangent = { -normal.y, normal.x };

            //
            std::array<Vec2, 2> impulseArray{};
            for (uint32_t i = 0; i < contactCount; i++)
            {
                Real oldJn = collisionData.persistentContactData[i].normalImpulseAccumulator;
                Real oldJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;

                const Vec2 warmStartImpulse = (oldJn * normal) + (oldJt * tangent);
                const bool isValid = oldJn > Real(0) || std::fabs(oldJt) > Real(0); // TODO: Check if it's right!

                impulseArray[i] = warmStartImpulse * Real(isValid);
            }
            { // Can apply sum of impulses, because they don't change outcome.
                const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

                linearVelocityA -= impulseSum * invMassA;
                linearVelocityB += impulseSum * invMassB;

                const Real dotSumA =
                    glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

                const Real dotSumB =
                    glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

                angularVelocityA -= dotSumA * invInertiaA;
                angularVelocityB += dotSumB * invInertiaB;
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

    void Solver::solveCollisionVelocityConstraints(
        bool firstIteration,
        std::span<const BodyCollisionData> collisionDataContainer,
        std::span<const VelocityConstraintData> constraintDataContainer,
        std::span<const FrictionData> frictionDataContainer
    )
    {
        TRACY_SCOPE_NC("Solve collision velocity constraints", Ecstasy::Color::Violet);

        // Optional warm-starting.
        if constexpr (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
        {
            if (firstIteration)
            {
                applyWarmStartingForCollisions(collisionDataContainer, constraintDataContainer);
            }
        }

        // Get pointers.
        Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
        const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

        // Main loop.
        const size_t collisionCount = collisionDataContainer.size();
        for (size_t c = 0; c < collisionCount; c++)
        {
            const BodyCollisionData& collisionData = collisionDataContainer[c];
            const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

            // Get body indices.
            const BodyIndex bodyIndexA = collisionData.bodyA;
            const BodyIndex bodyIndexB = collisionData.bodyB;

            // Get body data.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            const Real invInertiaA = invInertiaPtr[bodyIndexA];
            const Real invInertiaB = invInertiaPtr[bodyIndexB];

            Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
            Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
            Real angularVelocityA = angularVelocityPtr[bodyIndexA];
            Real angularVelocityB = angularVelocityPtr[bodyIndexB];

            //
            const Vec2 normal = collisionData.normal;
            const uint32_t contactCount = collisionData.contactCount;

            const Vec2 tangent = { -normal.y, normal.x };

            [[maybe_unused]] std::array<Vec2, 2> impulseArray{};
            std::array<Real, 2> jnArray{};

            // Collision impulses.
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

                Real& accumulatedJn = collisionData.persistentContactData[i].normalImpulseAccumulator;

                if (velocityAlongNormal > Real(0) && accumulatedJn <= Real(0)) continue; // TODO: Check if that is okay!

                const Real jn = -velocityAlongNormal * contactData.normalMassXElasticityFactor;

                const Real oldJn = accumulatedJn;
                accumulatedJn = std::fmax(Real(0), oldJn + jn);
                const Real deltaJn = accumulatedJn - oldJn;
;
                jnArray[i] = accumulatedJn;
                noContacts = false;

                if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplyImpulsesSequentially)
                {
                    const Vec2 impulse = deltaJn * normal;
                    linearVelocityA -= impulse * invMassA;
                    angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

                    linearVelocityB += impulse * invMassB;
                    angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
                }
                else if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
                {
                    impulseArray[i] = deltaJn * normal;
                }
            }

            // Check if there is at least one valid contact.
            if (noContacts)
            {
                velocityXPtr[bodyIndexA] = linearVelocityA.x;
                velocityYPtr[bodyIndexA] = linearVelocityA.y;
                velocityXPtr[bodyIndexB] = linearVelocityB.x;
                velocityYPtr[bodyIndexB] = linearVelocityB.y;
                continue;
            }

            if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
            {
                const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

                linearVelocityA -= impulseSum * invMassA;
                linearVelocityB += impulseSum * invMassB;

                const Real dotSumA =
                    glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

                const Real dotSumB =
                    glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

                angularVelocityA -= dotSumA * invInertiaA;
                angularVelocityB += dotSumB * invInertiaB;

                // Reset impulse array for friction accumulation step.
                impulseArray[0] = Vec2();
                impulseArray[1] = Vec2();
            }

            // Friction impulses.
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

                const Real currentSlipVel = glm::dot(relativeVelocity, tangent);

                const Real jt = -currentSlipVel * contactData.tangentMass;

                Real& accumulatedJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;
                const Real oldJt = accumulatedJt;
                Real targetJt = oldJt + jt;

                const Real jn = jnArray[i];

                if (std::fabs(targetJt) <= jn * staticFriction)
                {
                    accumulatedJt = targetJt; // Static friction.
                }
                else
                {
                    const Real maxDynamic = jn * dynamicFriction;
                    accumulatedJt = std::clamp(targetJt, -maxDynamic, maxDynamic); // Dynamic friction.
                }

                const Real deltaJt = accumulatedJt - oldJt;

                const Vec2 impulse = deltaJt * tangent;
                if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplyImpulsesSequentially)
                {
                    linearVelocityA -= impulse * invMassA;
                    angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

                    linearVelocityB += impulse * invMassB;
                    angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
                }
                else if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
                {
                    impulseArray[i] = impulse;
                }
            }

            if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
            {
                const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

                linearVelocityA -= impulseSum * invMassA;
                linearVelocityB += impulseSum * invMassB;

                const Real dotSumA =
                    glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

                const Real dotSumB =
                    glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
                    glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

                angularVelocityA -= dotSumA * invInertiaA;
                angularVelocityB += dotSumB * invInertiaB;
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

    void Solver::solveCollisionPositionConstraints(
        std::span<const BodyCollisionData> collisionDataContainer,
        std::span<const PositionConstraintData> constraintDataContainer
    )
    {
        TRACY_SCOPE_NC("Solve collision position constraints", Ecstasy::Color::Indigo);

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
            const Real currentPenetration = data.penetration - drift;

            const Real correctionDepth = currentPenetration - simulationSettings.positionCorrectionSlop;
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

    void Solver::solveCollisionConstraintsThreaded(
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
                            solveCollisionPositionConstraints(collisionSlice, constraintSlice);
                        }
                        else
                        {
                            std::span<const VelocityConstraintData> constraintSlice(velocityConstraintContainer.data() + passOffset.start, passOffset.size);
                            std::span<const FrictionData> frictionDataSlice(frictionDataContainer.data() + passOffset.start, passOffset.size);

                            solveCollisionVelocityConstraints(localTicket == 0, collisionSlice, constraintSlice, frictionDataSlice);
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