#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"
#include "Core/Simd.h"

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

        // Self-explanatory.
        applyExternalForces(bodyCount, deltaTime);
        integrate(bodyCount, deltaTime);
        boundaryCollisionResolution(bodyCount);

        // Build AABBs.
        {
            TRACY_SCOPE_N("Build AABBs");
            buildCircleAABBs();
        }

        std::fill(bodies.collisionDebug.begin(), bodies.collisionDebug.end(), 0); // Equal to 'bodyCount'.

        // Early return.
        if (bodyCount < 2) return;

        // Broad phase.
        const std::vector<BodyPair>& broadPhaseCollisions = broadPhaseCollisionDetector.findCollisions(AABBSoAViewer(bodies.aabb));
        if (broadPhaseCollisions.empty()) return;
        markBodiesOfBroadPhase(broadPhaseCollisions);

        // Narrow phase.
        narrowPhaseCollisionDetection();

        // Collision resolution.
        resolveCollisions();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Apply external forces");

        Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ simulationSettings.gravity.x * deltaTime };
        const RealSimd gravityDeltaYV{ simulationSettings.gravity.y * deltaTime };

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            RealSimd velX = RealSimd::loadu(velocityXPtr + i);
            RealSimd velY = RealSimd::loadu(velocityYPtr + i);

            velX += gravityDeltaXV;
            velY += gravityDeltaYV;

            velX.storeu(velocityXPtr + i);
            velY.storeu(velocityYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            velocityXPtr[i] += gravityDelta.x;
            velocityYPtr[i] += gravityDelta.y;
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
            const RealSimd velX = RealSimd::loadu(velocityXPtr + i);
            const RealSimd velY = RealSimd::loadu(velocityYPtr + i);

            RealSimd posX = RealSimd::loadu(positionXPtr + i);
            RealSimd posY = RealSimd::loadu(positionYPtr + i);

            posX = RealSimd::mul_add(velX, deltaTimeV, posX);
            posY = RealSimd::mul_add(velY, deltaTimeV, posY);

            posX.storeu(positionXPtr + i);
            posY.storeu(positionYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            positionXPtr[i] += velocityXPtr[i] * deltaTime;
            positionYPtr[i] += velocityYPtr[i] * deltaTime;
        }
    }

    void Simulation::boundaryCollisionResolution(size_t bodyCount)
    {
        // Note: I won't used SIMD here, because this method will get deleted.

        TRACY_SCOPE_N("Boundary collision");

        Real* CORE_RESTRICT positionX = bodies.positionX.data();
        Real* CORE_RESTRICT positionY = bodies.positionY.data();
        Real* CORE_RESTRICT velocityX = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityY = bodies.velocityY.data();

        const Real boundary = 10.0f;
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real x = positionX[i];
            const Real y = positionY[i];

            const Real absX = std::abs(x);
            const Real absY = std::abs(y);

            if (absX > boundary)
            {
                const Real sign = positionX[i] > 0.0 ? 1.0 : -1.0;
                positionX[i] = boundary * sign;
                velocityX[i] = -velocityX[i];
            }

            if (absY > boundary)
            {
                const Real sign = positionY[i] > 0.0 ? 1.0 : -1.0;
                positionY[i] = boundary * sign;
                velocityY[i] = -velocityY[i];
            }
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

    void Simulation::markBodiesOfBroadPhase(const std::vector<BodyPair>& broadPhaseCollisions)
    {
        TRACY_SCOPE_N("Mark bodies of broad phase");

        auto* CORE_RESTRICT collisionDebug = bodies.collisionDebug.data();

        for (const BodyPair& pair : broadPhaseCollisions)
        {
            collisionDebug[pair.a] = 1;
            collisionDebug[pair.b] = 1;
        }
    }

    void Simulation::narrowPhaseCollisionDetection()
    {
        TRACY_SCOPE_N("Narrow phase");

        //const size_t bodyPairCount = broadPhaseCollisions.size();
        //if (bodyPairCount == 0) return;

        //for (size_t i = 0; i < bodyPairCount; i++)
        //{
        //    BodyPair bodyPair = broadPhaseCollisions[i];
        //}
    }

    void Simulation::resolveCollisions()
    {
        TRACY_SCOPE_N("Resolve collisions");
    }
}