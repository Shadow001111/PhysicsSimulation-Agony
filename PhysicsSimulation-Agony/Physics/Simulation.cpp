#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
    static Real calculateCircleInertia(Real radius, Real mass)
    {
        return Real(0.5) * radius * radius * mass;
    }
    

    Simulation::Simulation()
    {
        materials.reserve(16);
        materials.emplace_back(); // Default material.
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");

        updateTimeAccumulator += deltaTime;

        const Real fixedDeltaTime = simulationSettings.updateInterval;
        while (updateTimeAccumulator >= fixedDeltaTime)
        {
            updateTimeAccumulator -= fixedDeltaTime;
            physicsStep(fixedDeltaTime * simulationSettings.timeScale);
        }
    }

    BodyIndex Simulation::createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex)
    {
        mass = std::max(Real(0.0), mass);
        radius = std::max(Real(0.0), radius);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = circles.getCount();

        const Real inertia = calculateCircleInertia(radius, mass);

        bodies.append(
            position,
            velocity,
            rotation,
            angularVelocity,
            mass, mass == 0.0 ? 0.0 : 1.0 / mass,
            inertia, inertia == 0.0 ? 0.0 : 1.0 / inertia,
			Vec2(0.0, 0.0),
            materialIndex < materials.size() ? materialIndex : 0,
            { position.x - radius, position.y - radius, position.x + radius, position.y + radius },
            BodyType::Circle,
            newShapeIndex
		);

        circles.append(
            radius,
            newBodyIndex
		);

        return newBodyIndex;
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::fetchBroadPhaseAABBs(std::vector<AABB>& outAABBs) const
    {
		broadPhaseCollisionDetector.fetchAABBs(outAABBs);
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_N("Physics step");

        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        if (materials.empty()) [[unlikely]]
        {
            std::cerr << "[AGONY][Simulation]: Material count is zero, which must be impossible.\n";
            materials.emplace_back(); // Default material.
        }

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

        const RealSimd deltaTimeV{ deltaTime };

        // Position.
        {
            Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
            Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
            const Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
            const Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();

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

        // Rotation.
        {
            Real* CORE_RESTRICT rotationPtr = bodies.rotation.data();
            const Real* CORE_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd angularVel = RealSimd::load(angularVelocityPtr + i);
                
                RealSimd rot = RealSimd::load(rotationPtr + i);
                
                rot = RealSimd::mul_add(angularVel, deltaTimeV, rot);
                
                rot.store(rotationPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                rotationPtr[i] += angularVelocityPtr[i] * deltaTime;
            }
        }
    }

    void Simulation::iterativeCollisionSolving()
    {
        const size_t bodyCount = bodies.getCount();

        // Early return.
        if (bodyCount < 2)
        {
            // Build AABBs.
            {
                TRACY_SCOPE_N("Build AABBs");
                buildCircleAABBs();
            }
            return;
        }

        // Solves until runs out of iterations or no collision is found.
        for (uint32_t i = 0; i < simulationSettings.collisionSolvingIterations; i++)
        {
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
            const MaterialIndex materialIndexA = materialIndexPtr[bodyIndexA];
            const MaterialIndex materialIndexB = materialIndexPtr[bodyIndexB];

            const Material* materialA = materialPtr + materialIndexA;
            const Material* materialB = materialPtr + materialIndexB;

            // Combine materials.
            const Real elasticity = (materialA->elasticity + materialB->elasticity) * Real(0.5) + Real(1.0); // Hoping for fused multiply-add. Adding here instead of adding in impulse calculation.
			const Real staticFriction = (materialA->staticFriction + materialB->staticFriction) * Real(0.5);
			const Real dynamicFriction = (materialA->dynamicFriction + materialB->dynamicFriction) * Real(0.5);

            // Compute impulse and apply it.
            const Real invTotalInvMass = Real(1.0) / totalInvMass;

            const Real impulse = elasticity * velocityAlongNormal * invTotalInvMass;
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