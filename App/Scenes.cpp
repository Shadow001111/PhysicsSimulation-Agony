#include "Scenes.h"

#include "Ecstasy/Core/Random.h"

#include "PS_AGONY2D/Simulation.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>


struct SceneOrientation
{
    float offsetX = 0;
    float offsetY = 0;
};


// Twice the signed area of the triangle (origin, pointA, pointB).
static PS_AGONY2D::Real cross(
    const PS_AGONY2D::Vec2& origin,
    const PS_AGONY2D::Vec2& pointA,
    const PS_AGONY2D::Vec2& pointB)
{
    return (pointA.x - origin.x) * (pointB.y - origin.y) - (pointA.y - origin.y) * (pointB.x - origin.x);
}

std::vector<PS_AGONY2D::Vec2> makeConvexPolygon(Ecstasy::Core::Random::Generator& randomGenerator, size_t targetVertices, float radius)
{
    const size_t sampleCount = std::max<size_t>(targetVertices * 4, 12);

    std::vector<PS_AGONY2D::Vec2> points;
    points.reserve(sampleCount);

    for (size_t i = 0; i < sampleCount; i++) {
        const float angle = randomGenerator.real<float>(0.0f, 6.28318530718f);
        const float distance = randomGenerator.real<float>(0.25f * radius, radius);
        points.push_back({ distance * std::cos(angle), distance * std::sin(angle) });
    }

    std::sort(points.begin(), points.end(), [](const auto& p1, const auto& p2)
        {
            return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
        });

    // Monotone chain: lower hull first, then upper hull.
    std::vector<PS_AGONY2D::Vec2> hull;
    hull.reserve(points.size() * 2);

    for (const auto& point : points)
    {
        while (hull.size() >= 2 &&
            cross(hull[hull.size() - 2], hull[hull.size() - 1], point) <= 0.0f)
        {
            hull.pop_back();
        }
        hull.push_back(point);
    }

    const size_t lowerSize = hull.size();

    for (auto it = points.rbegin() + 1; it != points.rend(); it++)
    {
        while (hull.size() > lowerSize &&
            cross(hull[hull.size() - 2], hull[hull.size() - 1], *it) <= 0.0f)
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }

    if (!hull.empty())
        hull.pop_back();

    return hull;
}


// Chassis center height above flat ground at rest for scale 1. Multiply by scale.
static constexpr float kCarRestHeight = 0.9f + 2.2f + 0.02f;

void load_ObjectCar(PS_AGONY2D::Simulation& simulation,
    float positionX,
    float positionY,
    float rotation,
    float scale = 1.0f)
{
    auto& objects = simulation.objects();
    auto& constraints = simulation.constraints();

    // Lengths scale linearly, mass by scale squared, stiffness linearly and damping by scale^1.5.
    const float massScale = scale * scale;
    const float stiffnessScale = scale;
    const float dampingScale = scale * std::sqrt(scale);

    const float frameWidth = 6.0f * scale;
    const float frameHeight = 1.2f * scale;
    const float chassisMass = 150.0f * massScale;

    const float wheelRadius = 0.9f * scale;
    const float wheelMass = 3.0f * massScale;
    const float wheelOffset = 2.2f * scale;      // Horizontal distance from the chassis center.
    const float suspensionDrop = 2.2f * scale;   // Vertical distance down to the wheel center.

    const float suspensionStiffness = 6000.0f * stiffnessScale;
    const float suspensionDamping = 45.0f * dampingScale;
    const float suspensionTravel = 0.5f * scale; // Max travel in both directions from the rest length.

    // The slider keeps the wheel on its axis, so it is stiff.
    const float sliderStiffness = 1.0e6f * stiffnessScale;
    const float sliderDamping = 1.0e3f * dampingScale;

    const PS_AGONY2D::MaterialIndex carMaterial = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 0.8f, .dynamicFriction = 0.6f });

    const PS_AGONY2D::MaterialIndex wheelMaterial = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 1.2f, .dynamicFriction = 0.9f });

    const float cosRotation = std::cos(rotation);
    const float sinRotation = std::sin(rotation);
    auto chassisToWorld = [&](float localX, float localY) -> PS_AGONY2D::Vec2 {
        return { positionX + localX * cosRotation - localY * sinRotation,
                 positionY + localX * sinRotation + localY * cosRotation };
        };

    const auto chassisIndex = objects.createBox({
        .base.position = { positionX, positionY },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = rotation,
        .base.angularVelocity = 0.0f,
        .base.mass = chassisMass,
        .base.materialIndex = carMaterial,
        .size = { frameWidth, frameHeight }
        });
    if (!chassisIndex) return;
    const PS_AGONY2D::Handle chassis = objects.getBodyHandle(*chassisIndex);

    // Side is -1 for the left wheel and +1 for the right wheel.
    auto createWheel = [&](float side) -> std::optional<PS_AGONY2D::Handle> {
        const auto wheelIndex = objects.createCircle({
            .base.position = chassisToWorld(side * wheelOffset, -suspensionDrop),
            .base.velocity = { 0.0f, 0.0f },
            .base.rotation = rotation,
            .base.angularVelocity = 0.0f,
            .base.mass = wheelMass,
            .base.materialIndex = wheelMaterial,
            .radius = wheelRadius
            });
        if (!wheelIndex) return std::nullopt;
        return objects.getBodyHandle(*wheelIndex);
        };

    const auto leftWheel = createWheel(-1.0f);
    const auto rightWheel = createWheel(+1.0f);
    if (!leftWheel || !rightWheel) return;

    // One spring along the suspension axis plus a slider joint that restricts the wheel to that axis.
    auto createSuspension = [&](PS_AGONY2D::Handle wheel, float side)
        {
            const float anchorX = side * wheelOffset;
            const float anchorY = -frameHeight * 0.5f;
            const float restLength = suspensionDrop - frameHeight * 0.5f;

            constraints.createSpring({
                .bodyHandleA = chassis,
                .bodyHandleB = wheel,
                .localAnchorA = { anchorX, anchorY },
                .localAnchorB = { 0.0f, 0.0f },
                .restLength = restLength,
                .stiffness = suspensionStiffness,
                .damping = suspensionDamping
                });

            constraints.createSliderJoint({
                .bodyHandleA = chassis,
                .bodyHandleB = wheel,
                .localAnchorA = { anchorX, anchorY },
                .localAnchorB = { 0.0f, 0.0f },
                .localAxisA = { 0.0f, -1.0f },   // Chassis local down.
                .stiffness = sliderStiffness,
                .damping = sliderDamping,
                .lowerLimit = restLength - suspensionTravel,
                .upperLimit = restLength + suspensionTravel,
                .enableLimit = true
                });
        };

    createSuspension(*leftWheel, -1.0f);
    createSuspension(*rightWheel, +1.0f);
}

void load_LargeWorld(
    PS_AGONY2D::Simulation& simulation,
    Ecstasy::Core::Random::Generator& randomGenerator,
    SceneOrientation sceneOrientation,
    const int bodyCountPerType
)
{
    auto& objects = simulation.objects();

    constexpr float worldWidth = 125.0f;
    constexpr float terrainSegmentWidth = 16.0f;
    constexpr float wallThickness = 5.0f;
    constexpr float wallHeight = 80.0f;
    constexpr float terrainBaseY = -20.0f;

    PS_AGONY2D::Material worldMaterial = {
        .elasticity = 0.9f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY2D::MaterialIndex materialIndex = simulation.createMaterial(worldMaterial);

    // Left wall.
    objects.createBox({
        .base.position = { -worldWidth * 0.5f - wallThickness * 0.5f + sceneOrientation.offsetX, wallHeight * 0.5f + terrainBaseY + sceneOrientation.offsetY },
        .base.mass = 0, // Static body.
        .base.materialIndex = materialIndex,
        .size = { wallThickness, wallHeight }
        });

    // Right wall.
    objects.createBox({
        .base.position = { worldWidth * 0.5f + wallThickness * 0.5f + sceneOrientation.offsetX, wallHeight * 0.5f + terrainBaseY + sceneOrientation.offsetY },
        .base.mass = 0, // Static body.
        .base.materialIndex = materialIndex,
        .size = { wallThickness, wallHeight }
        });

    // Terrain heights come from a clamped random walk.
    const int terrainSegments = static_cast<int>(std::ceil(worldWidth / terrainSegmentWidth));
    std::vector<float> heights(terrainSegments + 1);
    float currentHeight = 0.0f;

    for (int i = 0; i <= terrainSegments; i++) {
        heights[i] = currentHeight;
        currentHeight += randomGenerator.real<float>(-2.0f, 2.0f);
        currentHeight = std::fmax(-6.0f, std::fmin(currentHeight, 18.0f));
    }

    // Each segment is a convex quad with its anchor at the base midpoint.
    constexpr float halfSegmentWidth = terrainSegmentWidth * 0.5f;
    for (int i = 0; i < terrainSegments; i++) {
        const float segmentStartX = -worldWidth * 0.5f + i * terrainSegmentWidth;
        const float leftHeight = heights[i];
        const float rightHeight = heights[i + 1];

        const float positionX = segmentStartX + halfSegmentWidth;
        const float positionY = terrainBaseY;

        // Counter-clockwise, relative to the body position.
        PS_AGONY2D::Vec2 localVertices[4] = {
            { -halfSegmentWidth, 0.0f },                     // Bottom left.
            {  halfSegmentWidth, 0.0f },                     // Bottom right.
            {  halfSegmentWidth, rightHeight - terrainBaseY }, // Top right.
            { -halfSegmentWidth, leftHeight - terrainBaseY }   // Top left.
        };

        objects.createPolygon({
            .base.position = { positionX + sceneOrientation.offsetX, positionY + sceneOrientation.offsetY },
            .base.mass = 0,
            .base.materialIndex = materialIndex,
            .localVertices = { localVertices, 4 }
            });
    }

    // Keep spawns slightly away from the side walls.
    const float spawnBufferX = worldWidth * 0.45f;
    constexpr float spawnMaxY = wallHeight + terrainBaseY;
    constexpr float spawnMinY = terrainBaseY + 25.0f;

    // Circles.
    for (int i = 0; i < bodyCountPerType; i++) {
        const float x = randomGenerator.real<float>(-spawnBufferX, spawnBufferX);
        const float y = randomGenerator.real<float>(spawnMinY, spawnMaxY);
        const float velocityX = randomGenerator.real<float>(-1.5f, 1.5f);
        const float velocityY = randomGenerator.real<float>(-1.0f, 0.0f);
        const float radius = randomGenerator.real<float>(0.15f, 0.5f);
        const float mass = 3.14159f * radius * radius;

        objects.createCircle({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { velocityX, velocityY },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = mass,
            .base.materialIndex = materialIndex,
            .radius = radius
            });
    }

    // Boxes.
    for (int i = 0; i < bodyCountPerType; i++) {
        const float x = randomGenerator.real<float>(-spawnBufferX, spawnBufferX);
        const float y = randomGenerator.real<float>(spawnMinY, spawnMaxY);
        const float velocityX = randomGenerator.real<float>(-1.5f, 1.5f);
        const float velocityY = randomGenerator.real<float>(-1.0f, 0.0f);
        const float rotation = randomGenerator.real<float>(0.0f, 6.28f);
        const float width = randomGenerator.real<float>(0.3f, 0.8f);
        const float height = randomGenerator.real<float>(0.3f, 0.8f);
        const float mass = width * height;

        objects.createBox({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { velocityX, velocityY },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIndex,
            .size = { width, height }
            });
    }

    // Random convex polygons.
    for (int i = 0; i < bodyCountPerType; i++) {
        const float x = randomGenerator.real<float>(-spawnBufferX, spawnBufferX);
        const float y = randomGenerator.real<float>(spawnMinY, spawnMaxY);
        const float velocityX = randomGenerator.real<float>(-1.5f, 1.5f);
        const float velocityY = randomGenerator.real<float>(-1.0f, 0.0f);
        const float rotation = randomGenerator.real<float>(0.0f, 6.28f);
        const float radius = randomGenerator.real<float>(0.25f, 0.6f);
        const float mass = 1.2f;

        constexpr size_t maxVerticesCount = 8;
        const size_t verticesCount = randomGenerator.integer<size_t>(3, maxVerticesCount);
        auto localVertices = makeConvexPolygon(randomGenerator, verticesCount, radius);

        objects.createPolygon({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { velocityX, velocityY },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIndex,
            .localVertices = { localVertices.data(), verticesCount },
            });
    }

    // Capsules.
    for (int i = 0; i < bodyCountPerType; i++) {
        const float x = randomGenerator.real<float>(-spawnBufferX, spawnBufferX);
        const float y = randomGenerator.real<float>(spawnMinY, spawnMaxY);
        const float velocityX = randomGenerator.real<float>(-1.5f, 1.5f);
        const float velocityY = randomGenerator.real<float>(-1.0f, 0.0f);
        const float rotation = randomGenerator.real<float>(0.0f, 6.28f);
        const float length = randomGenerator.real<float>(0.3f, 0.8f);
        const float radius = randomGenerator.real<float>(0.3f, 0.8f);
        const float mass = (length * 2.0f + float(PS_AGONY2D::Constants::PI) * radius) * radius;

        objects.createCapsule({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { velocityX, velocityY },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIndex,
            .length = length,
            .radius = radius
            });
    }
}

void load_StackedPyramid(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& randomGenerator)
{
    auto& objects = simulation.objects();

    constexpr int pyramidRows = 60;
    constexpr float boxWidth = 0.5f;
    constexpr float boxHeight = 0.5f;
    constexpr float boxMass = 1.0f;

    constexpr float floorThickness = 2.0f;
    constexpr float floorWidth = (pyramidRows + 8) * boxWidth * 3;

    // High friction and low elasticity keep the pyramid stable on spawn.
    PS_AGONY2D::Material pyramidMaterial = {
        .elasticity = 0.1f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY2D::MaterialIndex materialIndex = simulation.createMaterial(pyramidMaterial);

    objects.createBox({
        .base.position = { 0.0f, -floorThickness * 0.5f },
        .base.mass = 0, // Static body.
        .base.materialIndex = materialIndex,
        .size = { floorWidth, floorThickness }
        });

    // Rows go from the bottom up to the apex.
    for (int row = 0; row < pyramidRows; row++)
    {
        const int boxesInRow = pyramidRows - row;

        // Center every row around x = 0.
        const float startX = -0.5f * (boxesInRow - 1) * boxWidth;
        const float y = (row + 0.5f) * boxHeight;

        for (int column = 0; column < boxesInRow; column++)
        {
            const float x = startX + column * boxWidth;

            objects.createBox({
                .base.position = { x, y },
                .base.velocity = { 0.0f, 0.0f },
                .base.rotation = 0.0f,
                .base.angularVelocity = 0.0f,
                .base.mass = boxMass,
                .base.materialIndex = materialIndex,
                .size = { boxWidth, boxHeight }
                });
        }
    }

    constexpr float ballRadius = 1.0f;
    objects.createCircle({
        .base.position = { -floorWidth * 0.5f + 2.0f, ballRadius },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = 0.0f,
        .base.angularVelocity = 0.0f,
        .base.mass = 50.0f,
        .base.materialIndex = materialIndex,
        .radius = ballRadius
        });
}

void load_ConstraintBridges(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& randomGenerator)
{
    auto& objects = simulation.objects();
    auto& constraints = simulation.constraints();

    constexpr float cliffWidth = 25.0f;
    constexpr float cliffHeight = 60.0f;
    constexpr float cliffY = 0.0f;
    constexpr float gapWidth = 150.0f;

    constexpr int plankCount = 100;
    constexpr float plankHeight = 0.5f;
    constexpr float plankMass = 1.5f;

    // Bridges are stacked above one another, one per constraint type.
    constexpr float bridgeSpacing = 25.0f;
    constexpr float springBridgeY = 0.0f;
    constexpr float jointBridgeY = springBridgeY + bridgeSpacing;
    constexpr float ballSpawnY = jointBridgeY + bridgeSpacing;

    // Planks leave small gaps so constraints can rotate freely.
    constexpr float plankStep = gapWidth / static_cast<float>(plankCount);
    constexpr float plankWidth = plankStep * 0.9f;

    constexpr float springStiffness = 2200.0f;
    constexpr float springDamping = 18.0f;
    constexpr float verticalOffset = plankHeight * 0.35f; // Dual anchor separation against torsional flipping.

    constexpr float jointStiffness = 2200.0f;
    constexpr float jointDamping = 18.0f;

    PS_AGONY2D::Material physicsMaterial = {
        .elasticity = 0.2f,
        .staticFriction = 0.7f,
        .dynamicFriction = 0.5f
    };
    PS_AGONY2D::MaterialIndex materialIndex = simulation.createMaterial(physicsMaterial);

    const float leftCliffX = -(gapWidth * 0.5f + cliffWidth * 0.5f);
    const auto leftCliffIndex = objects.createBox({
        .base.position = { leftCliffX, cliffY },
        .base.mass = 0, // Static anchor.
        .base.materialIndex = materialIndex,
        .size = { cliffWidth, cliffHeight }
        });

    const float rightCliffX = (gapWidth * 0.5f + cliffWidth * 0.5f);
    const auto rightCliffIndex = objects.createBox({
        .base.position = { rightCliffX, cliffY },
        .base.mass = 0, // Static anchor.
        .base.materialIndex = materialIndex,
        .size = { cliffWidth, cliffHeight }
        });

    if (!leftCliffIndex || !rightCliffIndex) return;

    const PS_AGONY2D::Handle leftCliff = objects.getBodyHandle(*leftCliffIndex);
    const PS_AGONY2D::Handle rightCliff = objects.getBodyHandle(*rightCliffIndex);

    const float spanStartX = -gapWidth * 0.5f;

    // Distance between adjacent anchors. Springs use it as rest length, joints ignore it.
    const float cliffToPlankRestLength = std::abs((spanStartX + plankStep * 0.5f - plankWidth * 0.5f) - (-gapWidth * 0.5f));
    const float interPlankRestLength = plankStep - plankWidth;

    // All join lambdas share one signature, so 'buildBridge' stays generic.
    auto joinWithSpring = [&](PS_AGONY2D::Handle bodyA, PS_AGONY2D::Handle bodyB,
        PS_AGONY2D::Vec2 localA, PS_AGONY2D::Vec2 localB, float restLength)
        {
            // Two vertically offset anchors resist torsional flipping.
            constraints.createSpring({
                .bodyHandleA = bodyA,
                .bodyHandleB = bodyB,
                .localAnchorA = { localA.x, localA.y + verticalOffset },
                .localAnchorB = { localB.x, localB.y + verticalOffset },
                .restLength = restLength,
                .stiffness = springStiffness,
                .damping = springDamping
                });
            constraints.createSpring({
                .bodyHandleA = bodyA,
                .bodyHandleB = bodyB,
                .localAnchorA = { localA.x, localA.y - verticalOffset },
                .localAnchorB = { localB.x, localB.y - verticalOffset },
                .restLength = restLength,
                .stiffness = springStiffness,
                .damping = springDamping
                });
        };

    auto joinWithJoint = [&](PS_AGONY2D::Handle bodyA, PS_AGONY2D::Handle bodyB,
        PS_AGONY2D::Vec2 localA, PS_AGONY2D::Vec2 localB, float /*restLength*/)
        {
            // The distance is implicit in the anchor placement.
            constraints.createRevoluteJoint({
                .bodyHandleA = bodyA,
                .bodyHandleB = bodyB,
                .localAnchorA = localA,
                .localAnchorB = localB,
                .stiffness = jointStiffness,
                .damping = jointDamping
                });
        };

    // 'joinPlanks' is a template parameter, so each call gets its own inlined instantiation.
    auto buildBridge = [&](float bridgeY, auto&& joinPlanks)
        {
            std::vector<PS_AGONY2D::Handle> planks;
            planks.reserve(plankCount);

            for (int i = 0; i < plankCount; i++)
            {
                const float plankX = spanStartX + plankStep * (static_cast<float>(i) + 0.5f);

                const auto plankIndex = objects.createBox({
                    .base.position = { plankX, bridgeY - plankHeight * 0.5f },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.angularVelocity = 0.0f,
                    .base.mass = plankMass,
                    .base.materialIndex = materialIndex,
                    .size = { plankWidth, plankHeight }
                    });

                if (plankIndex)
                {
                    planks.push_back(objects.getBodyHandle(*plankIndex));
                }
            }

            if (planks.empty()) return;

            const float cliffLocalY = bridgeY - plankHeight * 0.5f - cliffY;

            joinPlanks(leftCliff, planks.front(), PS_AGONY2D::Vec2{ cliffWidth * 0.5f, cliffLocalY }, PS_AGONY2D::Vec2{ -plankWidth * 0.5f, 0.0f }, cliffToPlankRestLength);

            for (size_t i = 0; i + 1 < planks.size(); i++)
            {
                joinPlanks(planks[i], planks[i + 1], PS_AGONY2D::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY2D::Vec2{ -plankWidth * 0.5f, 0.0f }, interPlankRestLength);
            }

            joinPlanks(planks.back(), rightCliff, PS_AGONY2D::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY2D::Vec2{ -cliffWidth * 0.5f, cliffLocalY }, cliffToPlankRestLength);
        };

    buildBridge(springBridgeY, joinWithSpring);
    buildBridge(jointBridgeY, joinWithJoint);

    // Balls dropped above the bridges.
    for (int i = 0; i < 5; i++)
    {
        const float spawnX = randomGenerator.real<float>(-gapWidth * 0.35f, gapWidth * 0.35f);
        const float spawnY = randomGenerator.real<float>(ballSpawnY + 4.0f, ballSpawnY + 12.0f);
        const float radius = randomGenerator.real<float>(0.6f, 1.2f);
        const float mass = 3.14159f * radius * radius * 2.0f;

        objects.createCircle({
            .base.position = { spawnX, spawnY },
            .base.velocity = { 0.0f, -2.0f },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = mass,
            .base.materialIndex = materialIndex,
            .radius = radius
            });
    }
}

void load_CarRamp(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& randomGenerator)
{
    auto& objects = simulation.objects();

    PS_AGONY2D::Material environmentMaterial = {
        .elasticity = 0.05f,
        .staticFriction = 0.9f,
        .dynamicFriction = 0.7f
    };
    PS_AGONY2D::MaterialIndex environmentMaterialIndex = simulation.createMaterial(environmentMaterial);

    constexpr int rampPoints = 120;
    constexpr float rampRadius = 65.0f;
    constexpr float yStart = 90.0f;
    constexpr float terrainBaseY = -60.0f;
    constexpr float flatWidth = 350.0f;
    constexpr float flatGroundY = yStart - 2.0f * rampRadius;

    constexpr float pi = 3.14159265359f;
    const float rampWidth = rampRadius * pi;

    const float leftRampStartX = -flatWidth * 0.5f - rampWidth;
    const float leftRampEndX = -flatWidth * 0.5f;

    // Left cycloid ramp with points evenly spaced on x.
    std::vector<PS_AGONY2D::Vec2> leftVertices;
    leftVertices.reserve(rampPoints + 3);

    leftVertices.push_back({ leftRampStartX, terrainBaseY });
    leftVertices.push_back({ leftRampEndX, terrainBaseY });

    // Solves theta - sin(theta) = targetArc with Newton iterations.
    auto solveTheta = [pi](float targetArc) {
        if (targetArc <= 0.0f) return 0.0f;
        if (targetArc >= pi) return pi;

        // Cubic approximation near zero as the initial guess.
        float theta = std::fmin(std::cbrt(6.0f * targetArc), pi);

        for (int iteration = 0; iteration < 5; iteration++)
        {
            const float residual = theta - std::sin(theta) - targetArc;
            const float derivative = 1.0f - std::cos(theta);
            if (std::abs(derivative) < 1e-6f) break;
            theta -= residual / derivative;
        }
        return std::fmax(0.0f, std::fmin(theta, pi));
        };

    for (int i = rampPoints; i >= 0; i--)
    {
        const float t = static_cast<float>(i) / rampPoints; // Goes from 1 to 0.
        const float x = leftRampStartX + t * rampWidth;

        const float theta = solveTheta(t * pi);
        const float y = yStart - rampRadius * (1.0f - std::cos(theta));

        leftVertices.push_back({ x, y });
    }

    // Mirror across the y axis. Reverse iteration keeps the counter-clockwise winding.
    std::vector<PS_AGONY2D::Vec2> rightVertices;
    rightVertices.reserve(leftVertices.size());
    for (auto it = leftVertices.rbegin(); it != leftVertices.rend(); it++)
    {
        rightVertices.push_back({ -it->x, it->y });
    }

    objects.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = environmentMaterialIndex,
        .localVertices = { leftVertices.data(), leftVertices.size() },
        .concaveDecompositionMethod = PS_AGONY2D::PhysicsGeometry::ConcaveDecompositionMethod::ReflexAngleBisector
        });

    objects.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = environmentMaterialIndex,
        .localVertices = { rightVertices.data(), rightVertices.size() },
        .concaveDecompositionMethod = PS_AGONY2D::PhysicsGeometry::ConcaveDecompositionMethod::ReflexAngleBisector
        });

    // Flat runout between the ramps.
    objects.createBox({
        .base.position = { 0.0f, flatGroundY - 2.0f },
        .base.mass = 0,
        .base.materialIndex = environmentMaterialIndex,
        .size = { flatWidth, 4.0f }
        });

    // Car spawns on the left ramp, aligned with the surface.
    const float spawnTheta = 0.35f;
    float carX = leftRampStartX + 0.7f + rampRadius * (spawnTheta - std::sin(spawnTheta));
    float carY = yStart + 20.4f - rampRadius * (1.0f - std::cos(spawnTheta));

    const float carRotation = std::atan2(-std::sin(spawnTheta), 1.0f - std::cos(spawnTheta));
    const float suspensionLift = 1.2f + 0.9f + 0.02f;
    carX += -std::sin(carRotation) * suspensionLift;
    carY += std::cos(carRotation) * suspensionLift;

    load_ObjectCar(simulation, carX, carY, carRotation);

    // Destructible target wall on the flat ground.
    constexpr int wallRows = 12;
    constexpr int wallColumns = 4;
    constexpr float boxSize = 2.0f;
    const float targetStartX = 50.0f;
    const float targetStartY = flatGroundY + (boxSize * 0.5f);

    PS_AGONY2D::Vec2 boxPolygonVertices[4] = {
        { -boxSize * 0.5f, -boxSize * 0.5f },
        {  boxSize * 0.5f, -boxSize * 0.5f },
        {  boxSize * 0.5f,  boxSize * 0.5f },
        { -boxSize * 0.5f,  boxSize * 0.5f }
    };

    for (int row = 0; row < wallRows; row++)
    {
        for (int column = 0; column < wallColumns; column++)
        {
            if (column == 0)
            {
                objects.createBox({
                    .base.position = { targetStartX + (column * boxSize), targetStartY + (row * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = environmentMaterialIndex,
                    .size = { boxSize, boxSize }
                    });
            }
            else if (column == 1)
            {
                objects.createCircle({
                    .base.position = { targetStartX + (column * boxSize), targetStartY + (row * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = environmentMaterialIndex,
                    .radius = boxSize * 0.5f
                    });
            }
            else if (column == 2)
            {
                objects.createPolygon({
                    .base.position = { targetStartX + (column * boxSize), targetStartY + (row * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = environmentMaterialIndex,
                    .localVertices = { boxPolygonVertices, 4 },
                    });
            }
            else if (column == 3 && (row & 1) == 0)
            {
                objects.createCapsule({
                    .base.position = { targetStartX + (column * boxSize), targetStartY + (row * boxSize) + (boxSize * 0.5f) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 90.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = environmentMaterialIndex,
                    .radius = boxSize * 0.5f,
                    .length = boxSize
                    });
            }
        }
    }
}

// Hull center height above flat ground at rest for scale 1. Multiply by scale.
// Sum of the road wheel axle height, half the hull height and the suspension drop.
static constexpr float kTankRestHeight = 0.725f + 0.6f + 1.6f;

// Side view tracked vehicle. Forward is +x, the powered sprocket is at the rear and the idler (tensioner) at the front.
// The bottom row holds sprung road wheels, the top row holds the end wheels and small return rollers.
// All top row wheels share one top edge, so the upper track run is flat and rests on the rollers.
// The end wheels sit outside the road wheels and higher, so the track nose slopes up to climb obstacles.
void load_ObjectTank(PS_AGONY2D::Simulation& simulation,
    float positionX,
    float positionY,
    float rotation,
    float scale = 1.0f)
{
    auto& objects = simulation.objects();
    auto& constraints = simulation.constraints();

    // Lengths scale linearly, mass by scale squared, stiffness linearly and damping by scale^1.5.
    const float massScale = scale * scale;
    const float stiffnessScale = scale;
    const float dampingScale = scale * std::sqrt(scale);

    constexpr float PI = 3.14159265358979f;
    constexpr float TWO_PI = 2.0f * PI;

    const float hullWidth = 8.0f * scale;
    const float hullHeight = 1.2f * scale;
    const float hullMass = 300.0f * massScale;

    // Road wheels (bottom row).
    constexpr int roadWheelCount = 5;
    const float roadWheelRadius = 0.5f * scale;
    const float roadWheelSpacing = 1.1f * scale;          // Center to center, leaves a gap of 0.1 * scale.
    const float roadWheelMass = 10.0f * massScale;        // Other wheels scale with radius squared.

    // End wheels (top row), smaller than the road wheels.
    const float endWheelRadius = 0.4f * scale;
    const float endWheelX = 3.6f * scale;                 // Distance from the hull center, outside the outermost road wheel.
    const float endWheelRise = 0.9f * scale;              // Height of the end wheel centers above the road wheel axle line.

    // Return rollers (top row), the smallest wheels.
    constexpr int rollerCount = 3;
    const float rollerRadius = 0.2f * scale;
    const float rollerSpacing = 1.6f * scale;             // Stays inside the road wheels' x range.

    // Smaller top row wheels are offset up to share the same top edge.
    const float topEdgeRise = endWheelRise + endWheelRadius;
    const float rollerRise = topEdgeRise - rollerRadius;

    // Track.
    const float linkHeight = 0.2f * scale;
    const float linkGap = 0.05f * scale;                  // Gap between neighbouring links. Must exceed the inner corner overlap on the tightest arc.
    const float linkPitch = 0.24f * scale;                // Target link length.
    const float linkMass = 3.0f * massScale;
    const float beltClearance = 0.02f * scale;            // Gap between the wheel rims and the inner track face.
    const float trackJointStiffness = 3.0e6f * stiffnessScale;
    const float trackJointDamping = 1.0e3f * dampingScale;

    // Link centerline radius around a wheel of the given radius.
    auto beltRadius = [&](float wheelRadius) { return wheelRadius + linkHeight * 0.5f + beltClearance; };

    // Hull bottom edge sits just above the outer face of the upper run.
    const float hullClearance = 0.08f * scale;
    const float suspensionDrop = topEdgeRise + beltClearance + linkHeight + hullClearance;   // Hull bottom edge to road wheel axle.

    const float suspensionStiffness = 25000.0f * stiffnessScale;
    const float suspensionDamping = 900.0f * dampingScale;
    const float suspensionTravel = 0.3f * scale;          // Max travel in both directions. Road wheel tops stay below the rollers when fully compressed.
    const float sliderStiffness = 1.0e6f * stiffnessScale;
    const float sliderDamping = 1.0e3f * dampingScale;

    // Spring rest length exceeds the spawn length, so the road wheel springs carry the hull weight.
    const float suspensionPreload = 0.03f * scale;

    // Sprocket and roller axles are fixed to the hull.
    const float axleStiffness = 3.0e6f * stiffnessScale;
    const float axleDamping = 1.0e3f * dampingScale;

    // Forward is +x, which needs a negative (clockwise) spin.
    const float driveSpeed = 12.0f * std::sqrt(scale);    // Track speed in m/s.
    const float driveTorque = 30000.0f * scale * scale * scale; // Force scales with scale squared, radius with scale.

    // Idler tensioner slides along hull x and keeps the upper run level. The spring is pre-compressed, so it always pushes the idler forward.
    const float tensionerLength = 0.4f * scale;           // Anchor to idler distance at spawn.
    const float tensionerTravel = 0.15f * scale;
    const float tensionerPreload = 0.08f * scale;
    const float tensionerStiffness = 30000.0f * stiffnessScale;
    const float tensionerDamping = 600.0f * dampingScale;

    // Hull local frame with the origin at the hull center.
    const float axleLocalY = -(hullHeight * 0.5f + suspensionDrop);   // Road wheel centers.
    const float endLocalY = axleLocalY + endWheelRise;                // Sprocket and idler centers.
    const float rollerLocalY = axleLocalY + rollerRise;               // Return roller centers.

    // Elasticity is zero everywhere, nothing should bounce on a tank.
    const PS_AGONY2D::MaterialIndex linkMaterial = simulation.createMaterial({       // Rubber cleats, grip ground and wheels.
        .elasticity = 0.0f, .staticFriction = 1.2f, .dynamicFriction = 1.0f });
    const PS_AGONY2D::MaterialIndex wheelMaterial = simulation.createMaterial({      // Grips the track so it can drive it.
        .elasticity = 0.0f, .staticFriction = 0.8f, .dynamicFriction = 0.7f });
    const PS_AGONY2D::MaterialIndex hullMaterial = simulation.createMaterial({       // Slippery, the track slides under it on contact.
        .elasticity = 0.0f, .staticFriction = 0.05f, .dynamicFriction = 0.03f });

    const float cosRotation = std::cos(rotation);
    const float sinRotation = std::sin(rotation);
    auto hullToWorld = [&](float localX, float localY) -> PS_AGONY2D::Vec2 {
        return { positionX + localX * cosRotation - localY * sinRotation,
                 positionY + localX * sinRotation + localY * cosRotation };
        };

    const auto hullIndex = objects.createBox({
        .base.position = { positionX, positionY },
        .base.rotation = rotation,
        .base.mass = hullMass,
        .base.materialIndex = hullMaterial,
        .size = { hullWidth, hullHeight }
        });
    if (!hullIndex) return;
    const PS_AGONY2D::Handle hull = objects.getBodyHandle(*hullIndex);

    // Mass scales with the squared radius ratio to the road wheel.
    auto createWheel = [&](float localX, float localY, float radius) -> std::optional<PS_AGONY2D::Handle>
        {
            const float ratio = radius / roadWheelRadius;
            const auto wheelIndex = objects.createCircle({
                .base.position = hullToWorld(localX, localY),
                .base.rotation = rotation,
                .base.mass = roadWheelMass * ratio * ratio,
                .base.materialIndex = wheelMaterial,
                .radius = radius
                });
            if (!wheelIndex) return std::nullopt;
            return objects.getBodyHandle(*wheelIndex);
        };

    // Sprocket at the rear: fixed axle and driven by a motor.
    const float sprocketX = -endWheelX;
    {
        const auto wheel = createWheel(sprocketX, endLocalY, endWheelRadius);
        if (!wheel) return;

        constraints.createRevoluteJoint({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { sprocketX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .stiffness = axleStiffness,
            .damping = axleDamping
            });
        constraints.createMotor({
            .bodyHandle = *wheel,
            .targetSpeed = -driveSpeed / beltRadius(endWheelRadius),
            .maxTorque = driveTorque
            });
    }

    // Idler at the front: horizontal slider plus a pre-compressed spring as the track tensioner.
    const float idlerX = endWheelX;
    {
        const auto wheel = createWheel(idlerX, endLocalY, endWheelRadius);
        if (!wheel) return;

        const float tensionerX = idlerX - tensionerLength;
        constraints.createSpring({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { tensionerX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .restLength = tensionerLength + tensionerPreload,
            .stiffness = tensionerStiffness,
            .damping = tensionerDamping
            });
        constraints.createSliderJoint({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { tensionerX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .localAxisA = { 1.0f, 0.0f },
            .stiffness = sliderStiffness,
            .damping = sliderDamping,
            .lowerLimit = tensionerLength - tensionerTravel,
            .upperLimit = tensionerLength + tensionerTravel,
            .enableLimit = true
            });
    }

    // Road wheels in the bottom row with independent vertical suspension.
    const float firstRoadWheelX = -0.5f * roadWheelSpacing * static_cast<float>(roadWheelCount - 1);
    for (int i = 0; i < roadWheelCount; i++)
    {
        const float localX = firstRoadWheelX + roadWheelSpacing * static_cast<float>(i);
        const float anchorY = -hullHeight * 0.5f;

        const auto wheel = createWheel(localX, axleLocalY, roadWheelRadius);
        if (!wheel) return;

        constraints.createSpring({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { localX, anchorY },
            .localAnchorB = { 0.0f, 0.0f },
            .restLength = suspensionDrop + suspensionPreload,
            .stiffness = suspensionStiffness,
            .damping = suspensionDamping
            });

        constraints.createSliderJoint({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { localX, anchorY },
            .localAnchorB = { 0.0f, 0.0f },
            .localAxisA = { 0.0f, -1.0f },
            .stiffness = sliderStiffness,
            .damping = sliderDamping,
            .lowerLimit = suspensionDrop - suspensionTravel,
            .upperLimit = suspensionDrop + suspensionTravel,
            .enableLimit = true
            });
    }

    // Return rollers in the top row spin freely on fixed axles.
    for (int i = 0; i < rollerCount; i++)
    {
        const float localX = rollerSpacing * (static_cast<float>(i) - 0.5f * static_cast<float>(rollerCount - 1));

        const auto wheel = createWheel(localX, rollerLocalY, rollerRadius);
        if (!wheel) return;

        constraints.createRevoluteJoint({
            .bodyHandleA = hull,
            .bodyHandleB = *wheel,
            .localAnchorA = { localX, rollerLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .stiffness = axleStiffness,
            .damping = axleDamping
            });
    }

    // The track wraps only the outer road wheels and the two end wheels.
    // The middle road wheels and the rollers touch the straight runs.
    // The path is the convex loop around those four circles, counter-clockwise from the bottom run.
    struct BeltCircle { float x, y, radius; };

    // Rear road wheel, front road wheel, idler, sprocket.
    const BeltCircle beltCircles[4] = {
        { firstRoadWheelX,  axleLocalY, beltRadius(roadWheelRadius) },
        { -firstRoadWheelX, axleLocalY, beltRadius(roadWheelRadius) },
        { idlerX,           endLocalY,  beltRadius(endWheelRadius) },
        { sprocketX,        endLocalY,  beltRadius(endWheelRadius) }
    };

    // External tangent between circle i and circle i + 1 with the loop interior on the left of travel.
    // The tangent angle is the outward normal angle of that line, also the angle of its tangent points on both circles.
    // Derived from (centerTo - centerFrom) . normal = radiusFrom - radiusTo.
    float tangentAngle[4];
    float tangentLength[4];
    for (int i = 0; i < 4; i++)
    {
        const BeltCircle& from = beltCircles[i];
        const BeltCircle& to = beltCircles[(i + 1) % 4];
        const float dx = to.x - from.x;
        const float dy = to.y - from.y;
        const float distance = std::sqrt(dx * dx + dy * dy);
        const float radiusDifference = from.radius - to.radius;
        tangentAngle[i] = std::atan2(dy, dx) - std::acos(radiusDifference / distance);
        tangentLength[i] = std::sqrt(distance * distance - radiusDifference * radiusDifference);
    }

    // Path of alternating segments: line 0, arc 1, line 1, arc 2, line 2, arc 3, line 3, arc 0.
    struct PathSegment
    {
        bool isArc;
        float x, y;         // Line start point or arc center.
        float dirX, dirY;   // Line unit direction.
        float radius;       // Arc radius.
        float startAngle;   // Arc start angle, increasing is counter-clockwise.
        float length;
    };
    std::vector<PathSegment> path;
    path.reserve(8);

    auto addLine = [&](int i)
        {
            const BeltCircle& from = beltCircles[i];
            PathSegment segment{};
            segment.isArc = false;
            segment.x = from.x + from.radius * std::cos(tangentAngle[i]);
            segment.y = from.y + from.radius * std::sin(tangentAngle[i]);
            segment.dirX = -std::sin(tangentAngle[i]);
            segment.dirY = std::cos(tangentAngle[i]);
            segment.length = tangentLength[i];
            path.push_back(segment);
        };
    auto addArc = [&](int i)
        {
            const BeltCircle& circle = beltCircles[i];
            const float startAngle = tangentAngle[(i + 3) % 4];
            float sweep = tangentAngle[i] - startAngle;
            while (sweep < 0.0f) sweep += TWO_PI;
            while (sweep >= TWO_PI) sweep -= TWO_PI;

            PathSegment segment{};
            segment.isArc = true;
            segment.x = circle.x;
            segment.y = circle.y;
            segment.radius = circle.radius;
            segment.startAngle = startAngle;
            segment.length = circle.radius * sweep;
            path.push_back(segment);
        };

    addLine(0); addArc(1);
    addLine(1); addArc(2);
    addLine(2); addArc(3);
    addLine(3); addArc(0);

    float perimeter = 0.0f;
    for (const PathSegment& segment : path)
        perimeter += segment.length;

    // Point on the path at the given distance from the start.
    auto pathPoint = [&](float distance) -> PS_AGONY2D::Vec2
        {
            for (size_t segmentIndex = 0; segmentIndex < path.size(); segmentIndex++)
            {
                const PathSegment& segment = path[segmentIndex];
                if (distance > segment.length && segmentIndex + 1 < path.size())
                {
                    distance -= segment.length;
                    continue;
                }
                if (segment.isArc)
                {
                    const float angle = segment.startAngle + distance / segment.radius;
                    return { segment.x + segment.radius * std::cos(angle), segment.y + segment.radius * std::sin(angle) };
                }
                return { segment.x + segment.dirX * distance, segment.y + segment.dirY * distance };
            }
            return { 0.0f, 0.0f };
        };

    const int linkCount = static_cast<int>(perimeter / linkPitch + 0.5f);

    // Pivot i joins link i - 1 and link i. Link i spans pivot i to pivot i + 1.
    std::vector<PS_AGONY2D::Vec2> pivots(linkCount);
    for (int i = 0; i < linkCount; i++)
        pivots[i] = pathPoint(perimeter * static_cast<float>(i) / static_cast<float>(linkCount));

    std::vector<PS_AGONY2D::Handle> links(linkCount);
    std::vector<float> halfChord(linkCount);
    for (int i = 0; i < linkCount; i++)
    {
        const PS_AGONY2D::Vec2 start = pivots[i];
        const PS_AGONY2D::Vec2 end = pivots[(i + 1) % linkCount];
        const float dx = end.x - start.x;
        const float dy = end.y - start.y;
        const float chord = std::sqrt(dx * dx + dy * dy);
        halfChord[i] = 0.5f * chord;

        const auto linkIndex = objects.createBox({
            .base.position = hullToWorld(0.5f * (start.x + end.x), 0.5f * (start.y + end.y)),
            .base.rotation = rotation + std::atan2(dy, dx),
            .base.mass = linkMass,
            .base.materialIndex = linkMaterial,
            .size = { chord - linkGap, linkHeight }
            });
        if (!linkIndex) return;
        links[i] = objects.getBodyHandle(*linkIndex);
    }

    // Anchors sit at the chord ends, slightly outside the box edges (by half of 'linkGap').
    for (int i = 0; i < linkCount; i++)
    {
        const int next = (i + 1) % linkCount;
        constraints.createRevoluteJoint({
            .bodyHandleA = links[i],
            .bodyHandleB = links[next],
            .localAnchorA = { +halfChord[i], 0.0f },
            .localAnchorB = { -halfChord[next], 0.0f },
            .stiffness = trackJointStiffness,
            .damping = trackJointDamping
            });
    }
}

// Flat ground with configurable obstacles, a tank (scale 2) and a car (scale 1) in front of it.
void load_Tank(PS_AGONY2D::Simulation& simulation)
{
    auto& objects = simulation.objects();

    constexpr float groundWidth = 400.0f;
    constexpr float groundThickness = 4.0f;

    constexpr float tankScale = 2.0f;
    constexpr float carScale = 1.0f;
    constexpr float tankX = 0.0f;
    constexpr float carX = 16.0f;       // Tank front reaches about 8.3, car rear wheel edge about 12.9.

    // Obstacle x is the center. Trenches must be sorted by x and must not overlap.
    // The car chassis is 6 long, so it can't bridge any trench wider than 6.
    // The tank flat track contact spans about 8.8 (five road wheels, scale 2) plus the sloped nose.
    struct Trench { float x; float width; float depth; };
    struct Bump { float x; float width; float height; };

    const std::vector<Trench> trenches = {
        { 35.5f, 7.0f, 4.0f },
        { 58.5f, 9.0f, 4.0f },
    };
    const std::vector<Bump> bumps = {
        { 22.0f, 1.4f, 0.5f },
        { 26.0f, 1.4f, 0.5f },
        { 45.0f, 1.4f, 0.5f },
    };

    const PS_AGONY2D::MaterialIndex groundMaterial = simulation.createMaterial({
        .elasticity = 0.0f, .staticFriction = 1.0f, .dynamicFriction = 0.9f });

    auto addBlock = [&](float startX, float endX, float topY)
        {
            objects.createBox({
                .base.position = { 0.5f * (startX + endX), topY - groundThickness * 0.5f },
                .base.mass = 0.0f,
                .base.materialIndex = groundMaterial,
                .size = { endX - startX, groundThickness }
                });
        };

    float cursorX = -groundWidth * 0.5f;
    for (const Trench& trench : trenches)
    {
        const float trenchStartX = trench.x - trench.width * 0.5f;
        const float trenchEndX = trench.x + trench.width * 0.5f;
        addBlock(cursorX, trenchStartX, 0.0f);          // Ground up to the trench.
        addBlock(trenchStartX, trenchEndX, -trench.depth); // Trench floor.
        cursorX = trenchEndX;
    }
    addBlock(cursorX, groundWidth * 0.5f, 0.0f);

    for (const Bump& bump : bumps)
        addBlock(bump.x - bump.width * 0.5f, bump.x + bump.width * 0.5f, bump.height);

    load_ObjectTank(simulation, tankX, kTankRestHeight * tankScale, 0.0f, tankScale);
    load_ObjectCar(simulation, carX, kCarRestHeight * carScale, 0.0f, carScale);
}

struct SplitContext
{
    PS_AGONY2D::MaterialIndex material;
    PS_AGONY2D::CollisionCallbackIndex callback;
    float minRadius; // Circles with half their radius below this no longer split.
};

// Circle with mass equal to radius. Splittable circles get the callback with a threshold of 16 * radius.
static void spawnSplitCircle(SplitContext& context, PS_AGONY2D::Simulation& simulation, PS_AGONY2D::Vec2 position, PS_AGONY2D::Vec2 velocity, float radius)
{
    auto& objects = simulation.objects();
    const auto body = objects.createCircle({
        .base.position = position,
        .base.velocity = velocity,
        .base.mass = radius,
        .base.materialIndex = context.material,
        .radius = radius
        });
    if (!body) return;

    if (radius * 0.5f < context.minRadius) return;
    for (const PS_AGONY2D::ObjectDenseIndex collider : objects.getBodyColliders(*body))
        simulation.setColliderCollisionCallback(collider, context.callback, 16.0f * radius);
}

static void onCircleSplit(const PS_AGONY2D::CollisionEvent& event, PS_AGONY2D::Simulation& simulation, void* userData)
{
    SplitContext& context = *static_cast<SplitContext*>(userData);

    auto& objects = simulation.objects();

    const auto data = objects.getBodyData(event.selfBody);
    if (!data) return;

    const float radius = float(data->mass); // Mass equals radius by construction.
    const PS_AGONY2D::Vec2 position = data->offset;
    const PS_AGONY2D::Vec2 velocity = data->velocity;

    objects.destroyBody(objects.getBodyHandle(event.selfBody));

    // Children sit side by side along the contact tangent inside the parent's footprint, so they don't overlap.
    const float half = radius * 0.5f;
    const PS_AGONY2D::Vec2 tangent = { -event.normal.y, event.normal.x };
    spawnSplitCircle(context, simulation, { position.x + tangent.x * half, position.y + tangent.y * half }, { velocity.x + tangent.x, velocity.y + tangent.y }, half);
    spawnSplitCircle(context, simulation, { position.x - tangent.x * half, position.y - tangent.y * half }, { velocity.x - tangent.x, velocity.y - tangent.y }, half);
}

static void load_CollisionCallbackTest(PS_AGONY2D::Simulation& simulation)
{
    auto& objects = simulation.objects();
    using namespace PS_AGONY2D;

    const MaterialIndex material = simulation.createMaterial({
        .elasticity = 0.3f,
        .staticFriction = 0.6f,
        .dynamicFriction = 0.4f
        });

    // Closed static room with the interior x in [-20, 20] and y in [-14, 14].
    auto createWall = [&](float x, float y, float width, float height)
        {
            objects.createBox({
                .base.position = { x, y },
                .base.mass = 0,
                .base.materialIndex = material,
                .size = { width, height }
                });
        };
    createWall(0.0f, -15.0f, 44.0f, 2.0f);
    createWall(0.0f, 15.0f, 44.0f, 2.0f);
    createWall(-21.0f, 0.0f, 2.0f, 32.0f);
    createWall(21.0f, 0.0f, 2.0f, 32.0f);

    // Five splits: 3.2, 1.6, 0.8, 0.4, 0.2, 0.1. Powers of two are exact in float.
    constexpr float startRadius = 3.2f;
    static SplitContext context; // Static, the callback references it for the whole scene lifetime.
    context.material = material;
    context.minRadius = 0.1f;

    const auto callback = simulation.createCollisionCallback(onCircleSplit, &context);
    if (!callback) return;
    context.callback = *callback;

    spawnSplitCircle(context, simulation, { 0.0f, 8.0f }, { 0.0f, 0.0f }, startRadius);
}

void loadScene(PS_AGONY2D::Simulation& simulation, int scene)
{
    Ecstasy::Core::Random::Generator randomGenerator{};
    randomGenerator.setSeed(0);

    if (scene == 0)
    {
        load_LargeWorld(simulation, randomGenerator, {}, 1500);
    }
    else if (scene == 1)
    {
        load_StackedPyramid(simulation, randomGenerator);
    }
    else if (scene == 2)
    {
        load_ConstraintBridges(simulation, randomGenerator);
    }
    else if (scene == 3)
    {
        load_CarRamp(simulation, randomGenerator);
    }
    else if (scene == 4)
    {
        load_Tank(simulation);
    }
    else if (scene == 5)
    {
        load_CollisionCallbackTest(simulation);
    }
}