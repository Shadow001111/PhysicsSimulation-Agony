#include "Simulation.h"

#include "Core/TracyProfiler.h"

namespace PS_AGONY
{
    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");
    }

    BodyIndex Simulation::createCircle(Vec2 position, Vec2 velocity, Real radius)
    {
        const BodyIndex newBodyIndex = bodies.positionX.size();
        const BodyIndex newCircleIndex = circles.radius.size();

        bodies.positionX.push_back(position.x);
        bodies.positionY.push_back(position.y);
        bodies.velocityX.push_back(velocity.x);
        bodies.velocityY.push_back(velocity.y);
        bodies.bodyType.push_back(BodyType::Circle);
        bodies.shapeIndex.push_back(newCircleIndex);

        circles.radius.push_back(radius);
        circles.bodyIndices.push_back(newBodyIndex);

        return newBodyIndex;
    }
}