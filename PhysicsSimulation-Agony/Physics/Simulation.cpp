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

        bodies.bodyType.push_back(BodyType::Circle);
        bodies.shapeIndex.push_back(newCircleIndex);

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
        const size_t bodyCount = bodies.getCount();

        // Gravity.
        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityX[i] += gravityDelta.x;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityY[i] += gravityDelta.y;
        }

        // Integration.
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionX[i] += bodies.velocityX[i] * deltaTime;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionY[i] += bodies.velocityY[i] * deltaTime;
        }

        // Boundaries.
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
}