#include "Simulation.h"

#include "Core/TracyProfiler.h"

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
        const std::vector<BodyPair>& broadPhaseCollisions = broadPhaseCollisionDetector.findCollisions(BodySoAViewer(bodies));
        if (broadPhaseCollisions.empty()) return;

        {
            TRACY_SCOPE_N("Mark bodies of broad phase");

            for (const BodyPair& pair : broadPhaseCollisions)
            {
                bodies.collisionDebug[pair.a] = 1;
                bodies.collisionDebug[pair.b] = 1;
            }
        }

        // Narrow phase.
        narrowPhaseCollisionDetection();

        // Collision resolution.
        resolveCollisions();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_N("Apply external forces");

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityX[i] += gravityDelta.x;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityY[i] += gravityDelta.y;
        }
    }

    void Simulation::integrate(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_N("Intergrate");

        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionX[i] += bodies.velocityX[i] * deltaTime;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionY[i] += bodies.velocityY[i] * deltaTime;
        }
    }

    void Simulation::boundaryCollisionResolution(size_t bodyCount)
    {
        TRACY_SCOPE_N("Boundary collision");

        const Real boundary = 10.0f;
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real x = bodies.positionX[i];
            const Real absX = std::abs(x);

            if (absX > boundary)
            {
                const Real sign = bodies.positionX[i] > 0.0 ? 1.0 : -1.0;
                bodies.positionX[i] = boundary * sign;
                bodies.velocityX[i] = -bodies.velocityX[i];
            }
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real y = bodies.positionY[i];
            const Real absY = std::abs(y);

            if (absY > boundary)
            {
                const Real sign = bodies.positionY[i] > 0.0 ? 1.0 : -1.0;
                bodies.positionY[i] = boundary * sign;
                bodies.velocityY[i] = -bodies.velocityY[i];
            }
        }
    }

    void Simulation::buildCircleAABBs()
    {
        TRACY_SCOPE_N("Build circle AABBs");

        const size_t circleCount = circles.getCount();

        for (size_t i = 0; i < circleCount; i++)
        {
            const Real radius = circles.radius[i];
            const BodyIndex bodyIndex = circles.bodyIndices[i];

            const Real x = bodies.positionX[bodyIndex];
            const Real y = bodies.positionY[bodyIndex];

            bodies.aabb.minX[bodyIndex] = x - radius;
            bodies.aabb.minY[bodyIndex] = y - radius;
            bodies.aabb.maxX[bodyIndex] = x + radius;
            bodies.aabb.maxY[bodyIndex] = y + radius;
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