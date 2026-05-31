#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

namespace PS_AGONY
{
    static Real calculateCircleInertia(Real radius, Real mass)
    {
        return Real(0.5) * radius * radius * mass;
    }
    

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");

        updateTimeAccumulator += deltaTime;

        const Real fixedDeltaTime = simulationSettings.updateInterval;
        while (updateTimeAccumulator >= fixedDeltaTime)
        {
            updateTimeAccumulator -= fixedDeltaTime;
            physicsStep(fixedDeltaTime);
        }
    }

    BodyIndex Simulation::createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex)
    {
        mass = std::max(Real(0.0), mass);
        radius = std::max(Real(0.0), radius);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newCircleIndex = circles.getCount();

        bodies.positionX.push_back(position.x);
        bodies.positionY.push_back(position.y);

        bodies.velocityX.push_back(velocity.x);
        bodies.velocityY.push_back(velocity.y);

        bodies.rotation.push_back(rotation);

        bodies.angularVelocity.push_back(angularVelocity);

        bodies.mass.push_back(mass);
        bodies.invMass.push_back(mass == 0.0 ? 0.0 : 1.0 / mass);

        const Real inertia = calculateCircleInertia(radius, mass);
        bodies.inertia.push_back(inertia);
        bodies.invInertia.push_back(inertia == 0.0 ? 0.0 : 1.0 / inertia);

        bodies.materialIndex.push_back(materialIndex);

        bodies.aabb.minX.push_back(0.0);
        bodies.aabb.minY.push_back(0.0);
        bodies.aabb.maxX.push_back(0.0);
        bodies.aabb.maxY.push_back(0.0);

        bodies.bodyType.push_back(BodyType::Circle);
        bodies.shapeIndex.push_back(newCircleIndex);

        bodies.collisionDebug.push_back(0);

        circles.radius.push_back(radius);
        circles.bodyIndices.push_back(newBodyIndex);

        return newBodyIndex;
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_N("Physics step");

        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        applyExternalForces(bodyCount, deltaTime);
        integrate(bodyCount, deltaTime);
        iterativeCollisionSolving();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Apply external forces");

        Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* CORE_RESTRICT invMassPtr = bodies.invMass.data();

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };

        const RealSimd zeros = RealSimd(Real(0));

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            RealSimd velX = RealSimd::load(velocityXPtr + i);
            RealSimd velY = RealSimd::load(velocityYPtr + i);

            const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            const auto movableMask = invMassV != zeros;

            RealSimd newVelX = velX + gravityDeltaXV;
            RealSimd newVelY = velY + gravityDeltaYV;

            velX = RealSimd::blendv(velX, newVelX, movableMask);
            velY = RealSimd::blendv(velY, newVelY, movableMask);

            velX.store(velocityXPtr + i);
            velY.store(velocityYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real invMass = invMassPtr[i];
            const Real movableMask = invMass != Real(0.0);
        
            velocityXPtr[i] += gravityDelta.x * movableMask;
            velocityYPtr[i] += gravityDelta.y * movableMask;
        }
    }

    void Simulation::integrate(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Intergrate");

        Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
        Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
        const Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        const Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();

        const RealSimd deltaTimeV{ deltaTime };

        // Note: having single loop (x and y interleaved) is a very-little faster than doing two separate passes.
        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            const RealSimd velX = RealSimd::load(velocityXPtr + i);
            const RealSimd velY = RealSimd::load(velocityYPtr + i);

            RealSimd posX = RealSimd::load(positionXPtr + i);
            RealSimd posY = RealSimd::load(positionYPtr + i);

            posX = RealSimd::mul_add(velX, deltaTimeV, posX);
            posY = RealSimd::mul_add(velY, deltaTimeV, posY);

            posX.store(positionXPtr + i);
            posY.store(positionYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            positionXPtr[i] += velocityXPtr[i] * deltaTime;
            positionYPtr[i] += velocityYPtr[i] * deltaTime;
        }
    }

    void Simulation::iterativeCollisionSolving()
    {
        const size_t bodyCount = bodies.getCount();

        // Solves until runs out of iterations or no collision is found.
        for (uint32_t i = 0; i < simulationSettings.collisionSolvingIterations; i++)
        {
            // Clear debug data.
            std::fill(bodies.collisionDebug.begin(), bodies.collisionDebug.begin() + bodyCount, 0);

            // Early return.
            if (bodyCount < 2) return;

            // Build AABBs.
            {
                TRACY_SCOPE_N("Build AABBs");
                buildCircleAABBs();
            }

            // Broad phase.
            const std::vector<BodyPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions(AABBSoAViewer(bodies.aabb));
            if (broadCollisionData.empty()) return;

            // Narrow phase.
            narrowPhaseCollisionDetector.setDataViewers(
                BodySoAViewer(bodies),
                CircleSoAViewer(circles)
            );
            const std::vector<BodyCollisionData>& narrowCollisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisionData);
            if (narrowCollisionData.empty()) return;

            // Collision resolution.
            resolveCollisions(narrowCollisionData);
        }
    }

    void Simulation::buildCircleAABBs()
    {
        TRACY_SCOPE_N("Build circle AABBs");

        const Real* CORE_RESTRICT positionX = bodies.positionX.data();
        const Real* CORE_RESTRICT positionY = bodies.positionY.data();
        Real* CORE_RESTRICT aabbMinX = bodies.aabb.minX.data();
        Real* CORE_RESTRICT aabbMinY = bodies.aabb.minY.data();
        Real* CORE_RESTRICT aabbMaxX = bodies.aabb.maxX.data();
        Real* CORE_RESTRICT aabbMaxY = bodies.aabb.maxY.data();

        const Real* CORE_RESTRICT radiusPtr = circles.radius.data();
        const BodyIndex* CORE_RESTRICT bodyIndexPtr = circles.bodyIndices.data();

        const size_t circleCount = circles.getCount();

        for (size_t i = 0; i < circleCount; i++)
        {
            const Real radius = radiusPtr[i];
            const BodyIndex bodyIndex = bodyIndexPtr[i];

            const Real x = positionX[bodyIndex];
            const Real y = positionY[bodyIndex];

            aabbMinX[bodyIndex] = x - radius;
            aabbMinY[bodyIndex] = y - radius;
            aabbMaxX[bodyIndex] = x + radius;
            aabbMaxY[bodyIndex] = y + radius;
        }
    }

    void Simulation::resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        TRACY_SCOPE_N("Resolve collisions");

        Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
        Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
        Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* CORE_RESTRICT invMassPtr = bodies.invMass.data();

        const MaterialIndex* CORE_RESTRICT materialIndexPtr = bodies.materialIndex.data();
        const Material* CORE_RESTRICT materialPtr = materials.data();
        const MaterialIndex materialCount = materials.size();
        if (materialCount == 0) return;

        for (const auto& data : narrowPhaseCollisions)
        {
            // Fetch data.
            const BodyIndex bodyIndexA = data.bodyA;
            const BodyIndex bodyIndexB = data.bodyB;

            const Vec2 normal = data.normal;
            const Real depth = data.depth;
        
            const Vec2 velocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
            const Vec2 velocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
            
            // Compute velocity along normal.
            const Vec2 relativeVelocity = velocityB - velocityA;
            const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);
            if (velocityAlongNormal > Real(0))
            {
                continue;
            }

            // Fetch inv masses and compute their sum.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            const Real totalInvMass = invMassA + invMassB;
            if (totalInvMass <= Real(0)) [[unlikely]]
            {
                continue;
            }

            // Fetch material.
            MaterialIndex materialIndexA = materialIndexPtr[bodyIndexA];
            MaterialIndex materialIndexB = materialIndexPtr[bodyIndexB];

            materialIndexA = materialIndexA < materialCount ? materialIndexA : 0;
            materialIndexB = materialIndexB < materialCount ? materialIndexB : 0;

            const Material* materialA = materialPtr + materialIndexA;
            const Material* materialB = materialPtr + materialIndexB;

            // Combine materials.
            const Real elasticity = (materialA->elasticity + materialB->elasticity) * Real(0.5);

            // Compute impulse and apply it.
            const Real invTotalInvMass = Real(1.0) / totalInvMass;

            const Real impulse = (elasticity + Real(1.0)) * velocityAlongNormal * invTotalInvMass;
            const Vec2 impulseVec = normal * impulse;

            velocityXPtr[bodyIndexA] += impulseVec.x * invMassA;
            velocityYPtr[bodyIndexA] += impulseVec.y * invMassA;

            velocityXPtr[bodyIndexB] -= impulseVec.x * invMassB;
            velocityYPtr[bodyIndexB] -= impulseVec.y * invMassB;

            // Position resolution.
            const Real correction = (depth - simulationSettings.slop) * simulationSettings.positionCorrectionPercent;
            if (correction <= Real(0)) [[unlikely]]
            {
                continue;
            }

            const Vec2 correctionVec2 = normal * correction;

            const Real ratioA = invMassA * invTotalInvMass;
            const Real ratioB = invMassB * invTotalInvMass;

            positionXPtr[bodyIndexA] -= correctionVec2.x * ratioA;
            positionYPtr[bodyIndexA] -= correctionVec2.y * ratioA;

            positionXPtr[bodyIndexB] += correctionVec2.x * ratioB;
            positionYPtr[bodyIndexB] += correctionVec2.y * ratioB;
        }
    }
}