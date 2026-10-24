#include "Scenes.h"

#include "Ecstasy/Core/Random.h"

#include "PS_AGONY2D/Simulation.h"

#include <cmath>
#include <iostream>


struct SceneOrientation
{
    float offsetX = 0;
    float offsetY = 0;
};


static PS_AGONY2D::Real cross(
    const PS_AGONY2D::Vec2& a,
    const PS_AGONY2D::Vec2& b,
    const PS_AGONY2D::Vec2& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<PS_AGONY2D::Vec2> makeConvexPolygon(Ecstasy::Core::Random::Generator& rvg, size_t targetVertices, float radius)
{
    const size_t sampleCount = std::max<size_t>(targetVertices * 4, 12);

    std::vector<PS_AGONY2D::Vec2> pts;
    pts.reserve(sampleCount);

    for (size_t i = 0; i < sampleCount; ++i) {
        const float a = rvg.real<float>(0.0f, 6.28318530718f);
        const float r = rvg.real<float>(0.25f * radius, radius);
        pts.push_back({ r * std::cos(a), r * std::sin(a) });
    }

    std::sort(pts.begin(), pts.end(), [](const auto& p1, const auto& p2)
        {
            return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
        });

    std::vector<PS_AGONY2D::Vec2> hull;
    hull.reserve(pts.size() * 2);

    for (const auto& p : pts)
    {
        while (hull.size() >= 2 &&
            cross(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0.0f)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    const size_t lowerSize = hull.size();

    for (auto it = pts.rbegin() + 1; it != pts.rend(); ++it)
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


// Chassis-center height above flat ground at rest, for scale = 1 (multiply by scale).
static constexpr float kCarRestHeight = 0.9f + 2.2f + 0.02f;

void load_ObjectCar(PS_AGONY2D::Simulation& simulation,
    float positionX,
    float positionY,
    float rotation,
    float scale = 1.0f)
{
    // Scaling: lengths x s, mass x s^2 (constant density), stiffness x s, damping x s^1.5
    // (keeps sag and damping ratio proportional to size).
    const float s = scale;
    const float massScale = s * s;
    const float stiffScale = s;
    const float dampScale = s * std::sqrt(s);

    // === CONFIGURATION ===
    const float frameWidth = 6.0f * s;
    const float frameHeight = 1.2f * s;
    const float chassisMass = 150.0f * massScale;

    const float wheelRadius = 0.9f * s;
    const float wheelMass = 3.0f * massScale;
    const float wheelOffset = 2.2f * s;   // Horizontal distance from chassis center
    const float suspensionDrop = 2.2f * s;   // Vertical distance down to wheel center

    const float suspensionK = 6000.0f * stiffScale; // was 999999999 (debug value?)
    const float suspensionD = 45.0f * dampScale;
    const float suspensionTravel = 0.5f * s;   // Max +/- travel from rest length

    // Slider joint (keeps the wheel on the axis, so it is a stiff constraint)
    const float sliderStiffness = 1.0e6f * stiffScale;
    const float sliderDamping = 1.0e3f * dampScale;

    // === MATERIALS ===
    const PS_AGONY2D::MaterialIndex carMatIdx = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 0.8f, .dynamicFriction = 0.6f });

    const PS_AGONY2D::MaterialIndex wheelMatIdx = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 1.2f, .dynamicFriction = 0.9f });

    // === CHASSIS-LOCAL -> WORLD ===
    const float cosR = std::cos(rotation);
    const float sinR = std::sin(rotation);
    auto toWorld = [&](float lx, float ly) -> PS_AGONY2D::Vec2 {
        return { positionX + lx * cosR - ly * sinR,
                 positionY + lx * sinR + ly * cosR };
        };

    // === CHASSIS ===
    auto chassisOpt = simulation.createBox({
        .base.position = { positionX, positionY },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = rotation,
        .base.angularVelocity = 0.0f,
        .base.mass = chassisMass,
        .base.materialIndex = carMatIdx,
        .size = { frameWidth, frameHeight }
        });
    if (!chassisOpt) return;
    const PS_AGONY2D::ObjectDenseIndex chassis = *chassisOpt;

    // === WHEEL FACTORY ===
    auto createWheel = [&](float side) -> std::optional<PS_AGONY2D::ObjectDenseIndex> {
        return simulation.createCircle({
            .base.position = toWorld(side * wheelOffset, -suspensionDrop),
            .base.velocity = { 0.0f, 0.0f },
            .base.rotation = rotation,
            .base.angularVelocity = 0.0f,
            .base.mass = wheelMass,
            .base.materialIndex = wheelMatIdx,
            .radius = wheelRadius
            });
        };

    auto leftWheelOpt = createWheel(-1.0f);
    auto rightWheelOpt = createWheel(+1.0f);
    if (!leftWheelOpt || !rightWheelOpt) return;

    // === SUSPENSION FACTORY ===
    // side = -1 (left) or +1 (right)
    auto createSuspension = [&](PS_AGONY2D::ObjectDenseIndex wheel, float side)
        {
            const float anchorX = side * wheelOffset;
            const float anchorY = -frameHeight * 0.5f;
            const float restLength = suspensionDrop - frameHeight * 0.5f;

            /* ---------------- OLD: triangulated springs ----------------
            constexpr float anchorSpread = 0.6f;
            const float diagonalRestLength =
                std::sqrt(anchorSpread * anchorSpread + restLength * restLength);

            for (float spread : { -anchorSpread, +anchorSpread })
            {
                simulation.createSpring({
                    .bodyIndexA   = chassis,
                    .bodyIndexB   = wheel,
                    .localAnchorA = { anchorX + spread, anchorY },
                    .localAnchorB = { 0.0f, 0.0f },
                    .restLength   = diagonalRestLength,
                    .stiffness    = suspensionK,
                    .damping      = suspensionD
                });
            }
            ------------------------------------------------------------ */

            // NEW: one spring along the suspension axis...
            simulation.createSpring({
                .bodyIndexA = chassis,
                .bodyIndexB = wheel,
                .localAnchorA = { anchorX, anchorY },
                .localAnchorB = { 0.0f, 0.0f },
                .restLength = restLength,
                .stiffness = suspensionK,
                .damping = suspensionD
                });

            // ...plus a prismatic joint that restricts the wheel to that axis.
            simulation.createSliderJoint({
                .bodyIndexA = chassis,
                .bodyIndexB = wheel,
                .localAnchorA = { anchorX, anchorY },
                .localAnchorB = { 0.0f, 0.0f },
                .localAxisA = { 0.0f, -1.0f },   // chassis-local "down"
                .stiffness = sliderStiffness,
                .damping = sliderDamping,
                .lowerLimit = restLength - suspensionTravel,
                .upperLimit = restLength + suspensionTravel,
                .enableLimit = true
                });
        };

    createSuspension(*leftWheelOpt, -1.0f);
    createSuspension(*rightWheelOpt, +1.0f);
}


void load_LargeWorld(
    PS_AGONY2D::Simulation& simulation,
    Ecstasy::Core::Random::Generator& rvg,
    SceneOrientation sceneOrientation,
    const int bodyCountPerType
)
{
    // === CONFIGURATION VARIABLES ===
    constexpr float worldWidth = 125.0f;
    constexpr float terrainSegmentWidth = 16.0f;
    constexpr float wallThickness = 5.0f;
    constexpr float wallHeight = 80.0f;
    constexpr float terrainBaseY = -20.0f;

    // === 1. MATERIAL SETUP ===
    PS_AGONY2D::Material worldMaterial = {
        .elasticity = 0.9f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY2D::MaterialIndex materialIdx = simulation.createMaterial(worldMaterial);

    // === 2. HIGH BOUNDS (SIDE WALLS) ===
    // Left Bounding Wall
    simulation.createBox({
        .base.position = { -worldWidth * 0.5f - wallThickness * 0.5f + sceneOrientation.offsetX, wallHeight * 0.5f + terrainBaseY + sceneOrientation.offsetY },
        .base.mass = 0, // Static
        .base.materialIndex = materialIdx,
        .size = { wallThickness, wallHeight }
        });

    // Right Bounding Wall
    simulation.createBox({
        .base.position = { worldWidth * 0.5f + wallThickness * 0.5f + sceneOrientation.offsetX, wallHeight * 0.5f + terrainBaseY + sceneOrientation.offsetY },
        .base.mass = 0, // Static
        .base.materialIndex = materialIdx,
        .size = { wallThickness, wallHeight }
        });

    // === 3. JOINED POLYGON LANDSCAPE (LOCAL COORDINATES) ===
    const int terrainSegments = static_cast<int>(std::ceil(worldWidth / terrainSegmentWidth));
    std::vector<float> heights(terrainSegments + 1);
    float currentHeight = 0.0f;

    // Pre-calculate heights using a random walk to create continuous terrain hills
    for (int i = 0; i <= terrainSegments; ++i) {
        heights[i] = currentHeight;
        currentHeight += rvg.real<float>(-2.0f, 2.0f);
        currentHeight = std::max(-6.0f, std::min(currentHeight, 18.0f)); // Clamp terrain elevations safely
    }

    // Build contiguous 4-vertex convex blocks across the world width
    constexpr float halfSegmentWidth = terrainSegmentWidth * 0.5f;
    for (int i = 0; i < terrainSegments; ++i) {
        float x1 = -worldWidth * 0.5f + i * terrainSegmentWidth;
        float y1 = heights[i];
        float y2 = heights[i + 1];

        // Define the explicit anchor position for this segment (midpoint base)
        float posX = x1 + halfSegmentWidth;
        float posY = terrainBaseY;

        // Define vertices locally relative to `.base.position` in Counter-Clockwise order
        PS_AGONY2D::Vec2 localVertices[4] = {
            { -halfSegmentWidth, 0.0f },             // Bottom-Left
            {  halfSegmentWidth, 0.0f },             // Bottom-Right
            {  halfSegmentWidth, y2 - terrainBaseY }, // Top-Right
            { -halfSegmentWidth, y1 - terrainBaseY }  // Top-Left
        };

        simulation.createPolygon({
            .base.position = { posX + sceneOrientation.offsetX, posY + sceneOrientation.offsetY },
            .base.mass = 0,
            .base.materialIndex = materialIdx,
            .localVertices = { localVertices, 4 }
            });
    }

    // === 4. EVENLY SPREAD DYNAMIC OBJECTS ===
    const float spawnBufferX = worldWidth * 0.45f; // Keep spawns slightly away from outer edge walls
    constexpr float spawnMaxY = wallHeight + terrainBaseY;
    constexpr float spawnMinY = terrainBaseY + 25.0f;

    // Spawn Circles
    for (int i = 0; i < bodyCountPerType; ++i) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float r = rvg.real<float>(0.15f, 0.5f);
        float mass = 3.14159f * r * r;

        simulation.createCircle({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { vx, vy },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .radius = r
            });
    }

    // Spawn Boxes
    for (int i = 0; i < bodyCountPerType; ++i) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float rotation = rvg.real<float>(0.0f, 6.28f);
        float width = rvg.real<float>(0.3f, 0.8f);
        float height = rvg.real<float>(0.3f, 0.8f);
        float mass = width * height;

        simulation.createBox({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { vx, vy },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .size = { width, height }
            });
    }

    // Spawn Random Convex Polygons
    for (int i = 0; i < bodyCountPerType; i++) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float rotation = rvg.real<float>(0.0f, 6.28f);
        float radius = rvg.real<float>(0.25f, 0.6f);
        float mass = 1.2f;

        constexpr size_t maxVerticesCount = 8;
        size_t verticesCount = rvg.integer<size_t>(3, maxVerticesCount);
        auto localVertices = makeConvexPolygon(rvg, verticesCount, radius);

        simulation.createPolygon({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { vx, vy },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .localVertices = { localVertices.data(), verticesCount },
            });
    }

    // Spawn Capsules
    for (int i = 0; i < bodyCountPerType; ++i) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float rotation = rvg.real<float>(0.0f, 6.28f);
        float length = rvg.real<float>(0.3f, 0.8f);
        float radius = rvg.real<float>(0.3f, 0.8f);
        float mass = (length * 2.0f + float(PS_AGONY2D::Constants::PI) * radius) * radius;

        simulation.createCapsule({
            .base.position = { x + sceneOrientation.offsetX, y + sceneOrientation.offsetY },
            .base.velocity = { vx, vy },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .length = length,
            .radius = radius
            });
    }
}

void load_StackedPyramid(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === CONFIGURATION VARIABLES ===
    constexpr int pyramidRows = 60;
    constexpr float boxWidth = 0.5f;
    constexpr float boxHeight = 0.5f;
    constexpr float boxMass = 1.0f;

    constexpr float floorThickness = 2.0f;
    constexpr float floorWidth = (pyramidRows + 8) * boxWidth * 3;

    // === 1. MATERIAL SETUP ===
    // High friction and lower elasticity help the pyramid remain stable upon initialization
    PS_AGONY2D::Material pyramidMaterial = {
        .elasticity = 0.1f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY2D::MaterialIndex materialIdx = simulation.createMaterial(pyramidMaterial);

    // === 2. STATIC FLOOR ===
    simulation.createBox({
        .base.position = { 0.0f, -floorThickness * 0.5f },
        .base.mass = 0, // Static
        .base.materialIndex = materialIdx,
        .size = { floorWidth, floorThickness }
        });

    // === 3. PYRAMID GENERATION ===
    // Outer loop moves from the bottom row up to the top apex
    for (int row = 0; row < pyramidRows; ++row)
    {
        int boxesInRow = pyramidRows - row;

        // Calculate starting X coordinate to center each row perfectly around X = 0
        float startX = -0.5f * (boxesInRow - 1) * boxWidth;

        // Calculate Y coordinate for the center of the boxes in the current row
        float y = (row + 0.5f) * boxHeight;

        for (int col = 0; col < boxesInRow; ++col)
        {
            float x = startX + col * boxWidth;

            simulation.createBox({
                .base.position = { x, y },
                .base.velocity = { 0.0f, 0.0f },
                .base.rotation = 0.0f,
                .base.angularVelocity = 0.0f,
                .base.mass = boxMass,
                .base.materialIndex = materialIdx,
                .size = { boxWidth, boxHeight }
                });
        }
    }

    {
        constexpr float ballRadius = 1.0f;
        simulation.createCircle({
            .base.position = { -floorWidth * 0.5f + 2.0f, ballRadius },
            .base.velocity = { 0.0f, 0.0f },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = 50.0f,
            .base.materialIndex = materialIdx,
            .radius = ballRadius
            });
    }
}

void load_ConstraintBridges(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === 1. CONFIGURATION AND MATERIALS ===
    constexpr float cliffWidth = 25.0f;
    constexpr float cliffHeight = 60.0f;
    constexpr float cliffY = 0.0f;
    constexpr float gapWidth = 150.0f;

    constexpr int plankCount = 100;
    constexpr float plankHeight = 0.5f;
    constexpr float plankMass = 1.5f;

    // Three identical bridges stacked above one another, one per constraint type.
    constexpr float bridgeSpacing = 25.0f;
    constexpr float springBridgeY = 0.0f;
    constexpr float jointBridgeY = springBridgeY + bridgeSpacing;
    constexpr float rodBridgeY = jointBridgeY + bridgeSpacing;

    // Calculate width per plank with breathing spacing between planks to let constraints rotate freely.
    constexpr float totalSpanWidth = gapWidth;
    constexpr float plankStep = totalSpanWidth / static_cast<float>(plankCount);
    constexpr float plankWidth = plankStep * 0.9f;

    // Spring mechanics configuration.
    constexpr float springK = 2200.0f;  // Structural stiffness.
    constexpr float springD = 18.0f;    // Structural damping.
    constexpr float verticalOffset = plankHeight * 0.35f; // Dual-anchor separation to prevent torsional flipping.

    // Joint mechanics configuration.
    constexpr float jointK = 2200.0f;   // Joint stiffness.
    constexpr float jointD = 18.0f;     // Joint damping.

    PS_AGONY2D::Material physicsMaterial = {
        .elasticity = 0.2f,
        .staticFriction = 0.7f,
        .dynamicFriction = 0.5f
    };
    PS_AGONY2D::MaterialIndex materialIdx = simulation.createMaterial(physicsMaterial);

    // === 2. CREATE TERRAIN CLIFFS ===
    const float leftCliffX = -(gapWidth * 0.5f + cliffWidth * 0.5f);
    auto leftCliffOpt = simulation.createBox({
        .base.position = { leftCliffX, cliffY },
        .base.mass = 0, // Static Anchor
        .base.materialIndex = materialIdx,
        .size = { cliffWidth, cliffHeight }
        });

    const float rightCliffX = (gapWidth * 0.5f + cliffWidth * 0.5f);
    auto rightCliffOpt = simulation.createBox({
        .base.position = { rightCliffX, cliffY },
        .base.mass = 0, // Static Anchor
        .base.materialIndex = materialIdx,
        .size = { cliffWidth, cliffHeight }
        });

    if (!leftCliffOpt || !rightCliffOpt) return;

    const PS_AGONY2D::ObjectDenseIndex leftCliff = *leftCliffOpt;
    const PS_AGONY2D::ObjectDenseIndex rightCliff = *rightCliffOpt;

    const float spanStartX = -gapWidth * 0.5f;

    // Distance between adjacent anchor points, shared by every constraint type -
    // springs use it as rest length, joints ignore it, rods use it as their fixed length.
    const float cliffToPlankRestLen = std::abs((spanStartX + plankStep * 0.5f - plankWidth * 0.5f) - (-gapWidth * 0.5f));
    const float interPlankRestLen = plankStep - plankWidth;

    // === 3. PER-CONSTRAINT-TYPE JOIN LAMBDAS ===
    // Each has the same signature (bodyA, bodyB, localA, localB, restLen), so buildBridge
    // below can stay a single generic lambda regardless of which constraint it's fed.

    auto joinWithSpring = [&](PS_AGONY2D::ObjectDenseIndex bodyA, PS_AGONY2D::ObjectDenseIndex bodyB,
        PS_AGONY2D::Vec2 localA, PS_AGONY2D::Vec2 localB, float restLen)
        {
            // Dual anchors (top/bottom offset) so the pair resists torsional flipping.
            simulation.createSpring({
                .bodyIndexA = bodyA,
                .bodyIndexB = bodyB,
                .localAnchorA = { localA.x, localA.y + verticalOffset },
                .localAnchorB = { localB.x, localB.y + verticalOffset },
                .restLength = restLen,
                .stiffness = springK,
                .damping = springD
                });
            simulation.createSpring({
                .bodyIndexA = bodyA,
                .bodyIndexB = bodyB,
                .localAnchorA = { localA.x, localA.y - verticalOffset },
                .localAnchorB = { localB.x, localB.y - verticalOffset },
                .restLength = restLen,
                .stiffness = springK,
                .damping = springD
                });
        };

    auto joinWithJoint = [&](PS_AGONY2D::ObjectDenseIndex bodyA, PS_AGONY2D::ObjectDenseIndex bodyB,
        PS_AGONY2D::Vec2 localA, PS_AGONY2D::Vec2 localB, float /*restLen*/)
        {
            // Point constraint - distance is implicit in the anchor placement, not a parameter.
            simulation.createRevoluteJoint({
                .bodyIndexA = bodyA,
                .bodyIndexB = bodyB,
                .localAnchorA = localA,
                .localAnchorB = localB,
                .stiffness = jointK,
                .damping = jointD
                });
        };

    auto joinWithRod = [&](PS_AGONY2D::ObjectDenseIndex bodyA, PS_AGONY2D::ObjectDenseIndex bodyB,
        PS_AGONY2D::Vec2 localA, PS_AGONY2D::Vec2 localB, float restLen)
        {
            // Rigid, inextensible - restLen becomes the rod's fixed length.
            simulation.createRod({
                .bodyIndexA = bodyA,
                .bodyIndexB = bodyB,
                .localAnchorA = localA,
                .localAnchorB = localB,
                .length = restLen
                });
        };

    // === 4. GENERIC BRIDGE BUILDER ===
    // joinPlanks is a template (auto) parameter, not std::function - each call below gets
    // its own inlined instantiation, so picking a constraint type costs nothing at runtime.
    auto buildBridge = [&](float bridgeY, auto&& joinPlanks)
        {
            std::vector<PS_AGONY2D::ObjectDenseIndex> planks;
            planks.reserve(plankCount);

            for (int i = 0; i < plankCount; ++i)
            {
                const float plankX = spanStartX + plankStep * (static_cast<float>(i) + 0.5f);

                auto plankOpt = simulation.createBox({
                    .base.position = { plankX, bridgeY - plankHeight * 0.5f },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.angularVelocity = 0.0f,
                    .base.mass = plankMass,
                    .base.materialIndex = materialIdx,
                    .size = { plankWidth, plankHeight }
                    });

                if (plankOpt)
                {
                    planks.push_back(*plankOpt);
                }
            }

            if (planks.empty()) return;

            const float cliffLocalY = bridgeY - plankHeight * 0.5f - cliffY;

            joinPlanks(leftCliff, planks.front(), PS_AGONY2D::Vec2{ cliffWidth * 0.5f, cliffLocalY }, PS_AGONY2D::Vec2{ -plankWidth * 0.5f, 0.0f }, cliffToPlankRestLen);

            for (size_t i = 0; i + 1 < planks.size(); ++i)
            {
                joinPlanks(planks[i], planks[i + 1], PS_AGONY2D::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY2D::Vec2{ -plankWidth * 0.5f, 0.0f }, interPlankRestLen);
            }

            joinPlanks(planks.back(), rightCliff, PS_AGONY2D::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY2D::Vec2{ -cliffWidth * 0.5f, cliffLocalY }, cliffToPlankRestLen);
        };

    // === 5. BUILD THE THREE BRIDGES ===
    buildBridge(springBridgeY, joinWithSpring);
    buildBridge(jointBridgeY, joinWithJoint);
    buildBridge(rodBridgeY, joinWithRod);

    // === 6. SPAWN INTERACTIONS (DECORATIVE INTERACTION BALLS) ===
    for (int i = 0; i < 5; ++i)
    {
        float rx = rvg.real<float>(-gapWidth * 0.35f, gapWidth * 0.35f);
        float ry = rvg.real<float>(rodBridgeY + 4.0f, rodBridgeY + 12.0f);
        float radius = rvg.real<float>(0.6f, 1.2f);
        float mass = 3.14159f * radius * radius * 2.0f;

        simulation.createCircle({
            .base.position = { rx, ry },
            .base.velocity = { 0.0f, -2.0f },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .radius = radius
            });
    }
}

void load_CarRamp(PS_AGONY2D::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // Set up physical properties for the environment.
    PS_AGONY2D::Material envMaterial = {
        .elasticity = 0.05f,
        .staticFriction = 0.9f,
        .dynamicFriction = 0.7f
    };
    PS_AGONY2D::MaterialIndex envMatIdx = simulation.createMaterial(envMaterial);

    // Define ramp geometry parameters and ground limits.
    constexpr int rampPoints = 120;
    constexpr float R = 65.0f;
    constexpr float yStart = 90.0f;
    constexpr float terrainBaseY = -60.0f;
    constexpr float flatWidth = 350.0f;
    constexpr float flatGroundY = yStart - 2.0f * R;

    constexpr float pi = 3.14159265359f;
    const float rampWidth = R * pi;

    const float leftRampStartX = -flatWidth * 0.5f - rampWidth;
    const float leftRampEndX = -flatWidth * 0.5f;

    // Create the left cycloid ramp polygon with points evenly spaced on X.
    std::vector<PS_AGONY2D::Vec2> leftVertices;
    leftVertices.reserve(rampPoints + 3);

    leftVertices.push_back({ leftRampStartX, terrainBaseY });
    leftVertices.push_back({ leftRampEndX, terrainBaseY });

    // Solver for theta in: theta - sin(theta) = S
    auto solveTheta = [pi](float S) {
        if (S <= 0.0f) return 0.0f;
        if (S >= pi) return pi;

        // Initial guess based on cubic approximation near S = 0
        float theta = std::cbrt(6.0f * S);
        if (theta > pi) theta = pi;

        for (int iter = 0; iter < 5; ++iter)
        {
            float f = theta - std::sin(theta) - S;
            float df = 1.0f - std::cos(theta);
            if (std::abs(df) < 1e-6f) break;
            theta -= f / df;
        }
        return std::clamp(theta, 0.0f, pi);
        };

    for (int i = rampPoints; i >= 0; --i)
    {
        float t = static_cast<float>(i) / rampPoints; // Normalized 1.0 -> 0.0
        float x = leftRampStartX + t * rampWidth;     // Even spacing along X

        float S = t * pi;                             // S in [0, pi]
        float theta = solveTheta(S);
        float y = yStart - R * (1.0f - std::cos(theta));

        leftVertices.push_back({ x, y });
    }

    // Create right ramp by mirroring leftVertices across X axis (-x)
    // Iterating in reverse preserves Counter-Clockwise (CCW) winding order
    std::vector<PS_AGONY2D::Vec2> rightVertices;
    rightVertices.reserve(leftVertices.size());
    for (auto it = leftVertices.rbegin(); it != leftVertices.rend(); ++it)
    {
        rightVertices.push_back({ -it->x, it->y });
    }

    simulation.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = envMatIdx,
        .localVertices = { leftVertices.data(), leftVertices.size() },
        .concaveDecompositionMethod = PS_AGONY2D::PhysicsGeometry::ConcaveDecompositionMethod::ReflexAngleBisector
        });

    simulation.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = envMatIdx,
        .localVertices = {rightVertices.data(), rightVertices.size()},
        .concaveDecompositionMethod = PS_AGONY2D::PhysicsGeometry::ConcaveDecompositionMethod::ReflexAngleBisector
        });

    // Create the central flat ground runout area centered at zero.
    simulation.createBox({
        .base.position = { 0.0f, flatGroundY - 2.0f },
        .base.mass = 0,
        .base.materialIndex = envMatIdx,
        .size = { flatWidth, 4.0f }
        });

    // Position and rotate the car on the left ramp.
    float spawnTheta = 0.35f;
    float carX = leftRampStartX + 0.7f + R * (spawnTheta - std::sin(spawnTheta));
    float carY = yStart + 20.4f - R * (1.0f - std::cos(spawnTheta));

    float carRotation = std::atan2(-std::sin(spawnTheta), 1.0f - std::cos(spawnTheta));
    float suspensionLift = 1.2f + 0.9f + 0.02f;
    carX += -std::sin(carRotation) * suspensionLift;
    carY += std::cos(carRotation) * suspensionLift;

    load_ObjectCar(simulation, carX, carY, carRotation);

    // Build the destructible target wall on the flat ground.
    constexpr int wallRows = 12;
    constexpr int wallCols = 4;
    constexpr float boxSize = 2.0f;
    const float targetStartX = 50.0f;
    const float targetStartY = flatGroundY + (boxSize * 0.5f);

    PS_AGONY2D::Vec2 boxPolyVertices[4] = {
        { -boxSize * 0.5f, -boxSize * 0.5f },
        {  boxSize * 0.5f, -boxSize * 0.5f },
        {  boxSize * 0.5f,  boxSize * 0.5f },
        { -boxSize * 0.5f,  boxSize * 0.5f }
    };

    for (int r = 0; r < wallRows; ++r)
    {
        for (int c = 0; c < wallCols; ++c)
        {
            if (c == 0)
            {
                simulation.createBox({
                    .base.position = { targetStartX + (c * boxSize), targetStartY + (r * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = envMatIdx,
                    .size = { boxSize, boxSize }
                    });
            }
            else if (c == 1)
            {
                simulation.createCircle({
                    .base.position = { targetStartX + (c * boxSize), targetStartY + (r * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = envMatIdx,
                    .radius = boxSize * 0.5f
                    });
            }
            else if (c == 2)
            {
                simulation.createPolygon({
                    .base.position = { targetStartX + (c * boxSize), targetStartY + (r * boxSize) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 0.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = envMatIdx,
                    .localVertices = {boxPolyVertices, 4},
                    });
            }
            else if (c == 3 && (r & 1) == 0)
            {
                simulation.createCapsule({
                    .base.position = { targetStartX + (c * boxSize), targetStartY + (r * boxSize) + (boxSize * 0.5f) },
                    .base.velocity = { 0.0f, 0.0f },
                    .base.rotation = 90.0f,
                    .base.mass = 2.5f,
                    .base.materialIndex = envMatIdx,
                    .radius = boxSize * 0.5f,
                    .length = boxSize
                    });
            }
        }
    }
}

// Hull-center height above flat ground at rest, for scale = 1 (multiply by scale).
// = road wheel axle height (wheelRadius + linkHeight + clearance + 5 mm = 0.725)
//   + hullHeight / 2 (0.6) + suspensionDrop (1.6, hull bottom edge to road wheel axle).
static constexpr float kTankRestHeight = 0.725f + 0.6f + 1.6f;

// Side-view tank with an upside-down trapezoid track, modelled on a real tracked vehicle:
//
//          .--o--o--o--.            top row:    sprocket (rear) + idler (front), both smaller than the
//         /  small      \                       road wheels, plus 3 small return rollers between them.
//        o  return        o                     The tops of all top-row wheels are at the same height, so
//         \ rollers      /                      the upper run is flat and rests on the rollers.
//          '-o-o-o-o-o-'            bottom row: 5 big road wheels on independent sprung suspension.
//
// The end wheels sit outside the road wheels on x and higher up, so the front and rear of the
// track slope upward and climb obstacles. The rollers stay inside the road wheels' x range.
// Forward is +x; the sprocket (powered) is at the rear, the idler (tensioner) at the front.
void load_ObjectTank(PS_AGONY2D::Simulation& simulation,
    float positionX,
    float positionY,
    float rotation,
    float scale = 1.0f)
{
    // Scaling: lengths x s, mass x s^2 (constant density), stiffness x s, damping x s^1.5.
    const float s = scale;
    const float massScale = s * s;
    const float stiffScale = s;
    const float dampScale = s * std::sqrt(s);

    // === CONFIGURATION ===
    constexpr float PI = 3.14159265358979f;
    constexpr float TWO_PI = 2.0f * PI;

    // Hull
    const float hullWidth = 8.0f * s;
    const float hullHeight = 1.2f * s;
    const float hullMass = 300.0f * massScale;

    // Road wheels (bottom row)
    constexpr int roadWheelCount = 5;
    const float roadWheelRadius = 0.5f * s;
    const float roadWheelSpacing = 1.1f * s;              // Center-to-center, leaves a 0.1 * s gap.
    const float roadWheelMass = 10.0f * massScale;        // Other wheels scale with radius^2.

    // End wheels (top row): sprocket at the rear, idler at the front. Smaller than the road wheels.
    const float endWheelRadius = 0.4f * s;
    const float endWheelX = 3.6f * s;                     // |x| from hull center; outside the outermost road wheel (2.2 * s).
    const float endWheelRise = 0.9f * s;                  // Height of the end wheel centers above the road wheel axle line.

    // Return rollers (top row, between the end wheels): the smallest wheels.
    constexpr int rollerCount = 3;
    const float rollerRadius = 0.2f * s;
    const float rollerSpacing = 1.6f * s;                 // Stays inside the road wheels' x range (+/- 2.2 * s).

    // All top-row wheels share the same top edge, so the upper run is flat.
    // Smaller wheels are offset up to match.
    const float topEdgeRise = endWheelRise + endWheelRadius;
    const float rollerRise = topEdgeRise - rollerRadius;

    // Track
    const float linkHeight = 0.2f * s;
    const float linkGap = 0.05f * s;                      // Total gap between neighbouring link boxes; must exceed the inner-corner overlap on the tightest arc.
    const float linkPitch = 0.24f * s;                    // Target link length (~0.25 x road wheel diameter, like a real track).
    const float linkMass = 3.0f * massScale;
    const float beltClearance = 0.02f * s;                // Gap between the wheel rims and the inner face of the track.
    const float jointStiffness = 3.0e6f * stiffScale;
    const float jointDamping = 1.0e3f * dampScale;

    // Link centerline radius around a wheel of the given radius.
    auto beltRadius = [&](float wheelRadius) { return wheelRadius + linkHeight * 0.5f + beltClearance; };

    // Hull bottom edge sits just above the outer face of the upper run.
    const float hullClearance = 0.08f * s;
    const float suspensionDrop = topEdgeRise + beltClearance + linkHeight + hullClearance;   // Hull bottom edge to road wheel axle (1.6 * s).

    // Suspension
    const float suspensionK = 25000.0f * stiffScale;
    const float suspensionD = 900.0f * dampScale;
    const float suspensionTravel = 0.3f * s;              // Max +/- from rest length. Road wheel tops stay below the rollers when fully compressed.
    const float sliderStiffness = 1.0e6f * stiffScale;
    const float sliderDamping = 1.0e3f * dampScale;

    // Spring rest length exceeds the spawn length so the road wheel springs carry the hull weight.
    const float suspensionPreload = 0.03f * s;

    // Sprocket: fixed axle on the hull.
    const float axleStiffness = 3.0e6f * stiffScale;
    const float axleDamping = 1.0e3f * dampScale;

    // Drive: angular motor on the sprocket. Forward is +x, which needs negative (clockwise) spin.
    const float driveSpeed = 12.0f * std::sqrt(s);          // Track speed, m/s.
    const float driveTorque = 30000.0f * s * s * s;         // Force ~ s^2, radius ~ s.

    // Idler: spring-loaded tensioner sliding along hull X (keeps the upper run level while taking up
    // slack). The spring is pre-compressed, so it always pushes the idler forward.
    const float tensionerLen = 0.4f * s;                  // Anchor-to-idler distance at spawn.
    const float tensionerTravel = 0.15f * s;
    const float tensionerPreload = 0.08f * s;
    const float tensionerK = 30000.0f * stiffScale;
    const float tensionerD = 600.0f * dampScale;

    // Local frame: origin at hull center.
    const float axleLocalY = -(hullHeight * 0.5f + suspensionDrop);   // Road wheel centers.
    const float endLocalY = axleLocalY + endWheelRise;                // Sprocket and idler centers.
    const float rollerLocalY = axleLocalY + rollerRise;               // Return roller centers.

    // === MATERIALS ===
    // Elasticity 0 everywhere: nothing should bounce on a tank.
    const PS_AGONY2D::MaterialIndex linkMat = simulation.createMaterial({       // Rubber/cleats: grips ground and wheels.
        .elasticity = 0.0f, .staticFriction = 1.2f, .dynamicFriction = 1.0f });
    const PS_AGONY2D::MaterialIndex wheelMat = simulation.createMaterial({      // Grips the track so it can drive it.
        .elasticity = 0.0f, .staticFriction = 0.8f, .dynamicFriction = 0.7f });
    const PS_AGONY2D::MaterialIndex hullMat = simulation.createMaterial({       // Slippery: the track slides under it if they touch.
        .elasticity = 0.0f, .staticFriction = 0.05f, .dynamicFriction = 0.03f });

    // === HULL-LOCAL -> WORLD ===
    const float cosR = std::cos(rotation);
    const float sinR = std::sin(rotation);
    auto toWorld = [&](float lx, float ly) -> PS_AGONY2D::Vec2 {
        return { positionX + lx * cosR - ly * sinR,
                 positionY + lx * sinR + ly * cosR };
        };

    // === HULL ===
    auto hullOpt = simulation.createBox({
        .base.position = { positionX, positionY },
        .base.rotation = rotation,
        .base.mass = hullMass,
        .base.materialIndex = hullMat,
        .size = { hullWidth, hullHeight }
        });
    if (!hullOpt) return;
    const PS_AGONY2D::ObjectDenseIndex hull = *hullOpt;

    // === WHEEL FACTORY ===
    // Mass scales with area (radius^2), anchored to the road wheel.
    auto createWheel = [&](float localX, float localY, float radius) -> std::optional<PS_AGONY2D::ObjectDenseIndex>
        {
            const float ratio = radius / roadWheelRadius;
            return simulation.createCircle({
                .base.position = toWorld(localX, localY),
                .base.rotation = rotation,
                .base.mass = roadWheelMass * ratio * ratio,
                .base.materialIndex = wheelMat,
                .radius = radius
                });
        };

    // === SPROCKET (rear, top row): fixed axle, driven ===
    const float sprocketX = -endWheelX;
    {
        auto wheelOpt = createWheel(sprocketX, endLocalY, endWheelRadius);
        if (!wheelOpt) return;

        simulation.createRevoluteJoint({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
            .localAnchorA = { sprocketX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .stiffness = axleStiffness,
            .damping = axleDamping
            });
        simulation.createMotor({
            .bodyIndex = *wheelOpt,
            .targetSpeed = -driveSpeed / beltRadius(endWheelRadius),
            .maxTorque = driveTorque
            });
    }

    // === IDLER (front, top row): horizontal slider + pre-compressed spring (track tensioner) ===
    const float idlerX = endWheelX;
    {
        auto wheelOpt = createWheel(idlerX, endLocalY, endWheelRadius);
        if (!wheelOpt) return;

        const float tensionerX = idlerX - tensionerLen;
        simulation.createSpring({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
            .localAnchorA = { tensionerX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .restLength = tensionerLen + tensionerPreload,
            .stiffness = tensionerK,
            .damping = tensionerD
            });
        simulation.createSliderJoint({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
            .localAnchorA = { tensionerX, endLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .localAxisA = { 1.0f, 0.0f },
            .stiffness = sliderStiffness,
            .damping = sliderDamping,
            .lowerLimit = tensionerLen - tensionerTravel,
            .upperLimit = tensionerLen + tensionerTravel,
            .enableLimit = true
            });
    }

    // === ROAD WHEELS (bottom row): independent vertical suspension ===
    const float firstRoadWheelX = -0.5f * roadWheelSpacing * static_cast<float>(roadWheelCount - 1);
    for (int i = 0; i < roadWheelCount; ++i)
    {
        const float localX = firstRoadWheelX + roadWheelSpacing * static_cast<float>(i);
        const float anchorY = -hullHeight * 0.5f;

        auto wheelOpt = createWheel(localX, axleLocalY, roadWheelRadius);
        if (!wheelOpt) return;

        simulation.createSpring({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
            .localAnchorA = { localX, anchorY },
            .localAnchorB = { 0.0f, 0.0f },
            .restLength = suspensionDrop + suspensionPreload,
            .stiffness = suspensionK,
            .damping = suspensionD
            });

        simulation.createSliderJoint({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
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

    // === RETURN ROLLERS (top row, between the end wheels): free-spinning on fixed axles ===
    for (int i = 0; i < rollerCount; ++i)
    {
        const float localX = rollerSpacing * (static_cast<float>(i) - 0.5f * static_cast<float>(rollerCount - 1));

        auto wheelOpt = createWheel(localX, rollerLocalY, rollerRadius);
        if (!wheelOpt) return;

        simulation.createRevoluteJoint({
            .bodyIndexA = hull,
            .bodyIndexB = *wheelOpt,
            .localAnchorA = { localX, rollerLocalY },
            .localAnchorB = { 0.0f, 0.0f },
            .stiffness = axleStiffness,
            .damping = axleDamping
            });
    }

    // === TRACK PATH (hull-local) ===
    // The track only wraps around four wheels: the outer road wheels and the two end wheels. The
    // middle road wheels and the rollers touch the straight runs. The path is the convex loop around
    // those four circles: counter-clockwise from the bottom run -> front road wheel arc -> front
    // slope -> idler arc -> top run -> sprocket arc -> rear slope -> rear road wheel arc.
    struct BeltCircle { float x, y, r; };

    // Order: rear road wheel, front road wheel, idler, sprocket (counter-clockwise).
    const BeltCircle circles[4] = {
        { firstRoadWheelX,  axleLocalY, beltRadius(roadWheelRadius) },
        { -firstRoadWheelX, axleLocalY, beltRadius(roadWheelRadius) },
        { idlerX,           endLocalY,  beltRadius(endWheelRadius) },
        { sprocketX,        endLocalY,  beltRadius(endWheelRadius) }
    };

    // External tangent between circle i and circle i+1, with the loop interior on the left of travel.
    // phi[i] is the angle of the outward normal of that tangent line (also the angle, on both circles,
    // of the tangent points). Derived from (c_j - c_i) . n = r_i - r_j.
    float phi[4];
    float lineLength[4];
    for (int i = 0; i < 4; ++i)
    {
        const BeltCircle& from = circles[i];
        const BeltCircle& to = circles[(i + 1) % 4];
        const float dx = to.x - from.x;
        const float dy = to.y - from.y;
        const float dist = std::sqrt(dx * dx + dy * dy);
        const float dr = from.r - to.r;
        phi[i] = std::atan2(dy, dx) - std::acos(dr / dist);
        lineLength[i] = std::sqrt(dist * dist - dr * dr);
    }

    // Path as alternating segments: line 0, arc 1, line 1, arc 2, line 2, arc 3, line 3, arc 0.
    struct PathSegment
    {
        bool isArc;
        float x, y;         // Line: start point. Arc: center.
        float dirX, dirY;   // Line: unit direction.
        float radius;       // Arc.
        float angle0;       // Arc: start angle (increasing = counter-clockwise).
        float length;
    };
    std::vector<PathSegment> path;
    path.reserve(8);

    auto addLine = [&](int i)
        {
            const BeltCircle& from = circles[i];
            PathSegment seg{};
            seg.isArc = false;
            seg.x = from.x + from.r * std::cos(phi[i]);
            seg.y = from.y + from.r * std::sin(phi[i]);
            seg.dirX = -std::sin(phi[i]);
            seg.dirY = std::cos(phi[i]);
            seg.length = lineLength[i];
            path.push_back(seg);
        };
    auto addArc = [&](int i)
        {
            const BeltCircle& c = circles[i];
            const float a0 = phi[(i + 3) % 4];
            float sweep = phi[i] - a0;
            while (sweep < 0.0f) sweep += TWO_PI;
            while (sweep >= TWO_PI) sweep -= TWO_PI;

            PathSegment seg{};
            seg.isArc = true;
            seg.x = c.x;
            seg.y = c.y;
            seg.radius = c.r;
            seg.angle0 = a0;
            seg.length = c.r * sweep;
            path.push_back(seg);
        };

    addLine(0); addArc(1);
    addLine(1); addArc(2);
    addLine(2); addArc(3);
    addLine(3); addArc(0);

    float perimeter = 0.0f;
    for (const PathSegment& seg : path)
        perimeter += seg.length;

    auto pathPoint = [&](float d) -> PS_AGONY2D::Vec2
        {
            for (size_t k = 0; k < path.size(); ++k)
            {
                const PathSegment& seg = path[k];
                if (d > seg.length && k + 1 < path.size())
                {
                    d -= seg.length;
                    continue;
                }
                if (seg.isArc)
                {
                    const float a = seg.angle0 + d / seg.radius;
                    return { seg.x + seg.radius * std::cos(a), seg.y + seg.radius * std::sin(a) };
                }
                return { seg.x + seg.dirX * d, seg.y + seg.dirY * d };
            }
            return { 0.0f, 0.0f };
        };

    const int linkCount = static_cast<int>(perimeter / linkPitch + 0.5f);

    // Pivot i is the joint point between link i-1 and link i; link i spans pivot i -> pivot i+1.
    std::vector<PS_AGONY2D::Vec2> pivots(linkCount);
    for (int i = 0; i < linkCount; ++i)
        pivots[i] = pathPoint(perimeter * static_cast<float>(i) / static_cast<float>(linkCount));

    // === TRACK LINKS ===
    std::vector<PS_AGONY2D::ObjectDenseIndex> links(linkCount);
    std::vector<float> halfChord(linkCount);
    for (int i = 0; i < linkCount; ++i)
    {
        const PS_AGONY2D::Vec2 a = pivots[i];
        const PS_AGONY2D::Vec2 b = pivots[(i + 1) % linkCount];
        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float chord = std::sqrt(dx * dx + dy * dy);
        halfChord[i] = 0.5f * chord;

        auto linkOpt = simulation.createBox({
            .base.position = toWorld(0.5f * (a.x + b.x), 0.5f * (a.y + b.y)),
            .base.rotation = rotation + std::atan2(dy, dx),
            .base.mass = linkMass,
            .base.materialIndex = linkMat,
            .size = { chord - linkGap, linkHeight }
            });
        if (!linkOpt) return;
        links[i] = *linkOpt;
    }

    // === TRACK JOINTS ===
    // Anchors sit at the chord ends, slightly outside the box edges (by linkGap / 2).
    for (int i = 0; i < linkCount; ++i)
    {
        const int next = (i + 1) % linkCount;
        simulation.createRevoluteJoint({
            .bodyIndexA = links[i],
            .bodyIndexB = links[next],
            .localAnchorA = { +halfChord[i], 0.0f },
            .localAnchorB = { -halfChord[next], 0.0f },
            .stiffness = jointStiffness,
            .damping = jointDamping
            });
    }
}

// Scene: flat ground with configurable obstacles, a tank (scale 2) and a car (scale 1) in front of it.
void load_Tank(PS_AGONY2D::Simulation& simulation)
{
    // === CONFIGURATION ===
    constexpr float groundWidth = 400.0f;
    constexpr float groundThickness = 4.0f;

    constexpr float tankScale = 2.0f;
    constexpr float carScale = 1.0f;
    constexpr float tankX = 0.0f;
    constexpr float carX = 16.0f;       // Tank front reaches ~8.3, car rear wheel edge ~12.9.

    // Obstacles. x = center. Trenches must be sorted by x and must not overlap.
    // Car chassis is 6 long, so any trench > 6 wide can't be bridged by it.
    // Tank flat track contact spans ~8.8 (5 road wheels, scale 2) plus the sloped nose, so it bridges up to roughly that.
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

    // === MATERIAL ===
    const PS_AGONY2D::MaterialIndex groundMat = simulation.createMaterial({
        .elasticity = 0.0f, .staticFriction = 1.0f, .dynamicFriction = 0.9f });

    // === TERRAIN ===
    auto addBlock = [&](float x0, float x1, float topY)
        {
            simulation.createBox({
                .base.position = { 0.5f * (x0 + x1), topY - groundThickness * 0.5f },
                .base.mass = 0.0f,
                .base.materialIndex = groundMat,
                .size = { x1 - x0, groundThickness }
                });
        };

    float cursor = -groundWidth * 0.5f;
    for (const Trench& t : trenches)
    {
        const float x0 = t.x - t.width * 0.5f;
        const float x1 = t.x + t.width * 0.5f;
        addBlock(cursor, x0, 0.0f);     // Ground up to the trench.
        addBlock(x0, x1, -t.depth);     // Trench floor.
        cursor = x1;
    }
    addBlock(cursor, groundWidth * 0.5f, 0.0f);

    for (const Bump& b : bumps)
        addBlock(b.x - b.width * 0.5f, b.x + b.width * 0.5f, b.height);

    // === VEHICLES ===
    load_ObjectTank(simulation, tankX, kTankRestHeight * tankScale, 0.0f, tankScale);
    load_ObjectCar(simulation, carX, kCarRestHeight * carScale, 0.0f, carScale);
}

struct SplitContext
{
    PS_AGONY2D::Simulation* simulation;
    PS_AGONY2D::MaterialIndex material;
    PS_AGONY2D::CollisionCallbackIndex callback;
    float minRadius; // Circles with radius/2 below this no longer split.
};

// Circle with mass == radius. Splittable circles get the callback (threshold = 4 * radius, i.e. approach speed ~4).
static void spawnSplitCircle(SplitContext& ctx, PS_AGONY2D::Vec2 position, PS_AGONY2D::Vec2 velocity, float radius)
{
    const auto body = ctx.simulation->createCircle({
        .base.position = position,
        .base.velocity = velocity,
        .base.mass = radius,
        .base.materialIndex = ctx.material,
        .radius = radius
        });
    if (!body) return;

    if (radius * 0.5f < ctx.minRadius) return;
    for (const PS_AGONY2D::ColliderDenseIndex collider : ctx.simulation->getBodyColliders(*body))
        ctx.simulation->setColliderCollisionCallback(collider, ctx.callback, 16.0f * radius);
}

static void onCircleSplit(const PS_AGONY2D::CollisionEvent& event, void* userData)
{
    SplitContext& ctx = *static_cast<SplitContext*>(userData);
    PS_AGONY2D::Simulation& simulation = *ctx.simulation;

    const auto data = simulation.getBodyData(event.selfBody);
    if (!data) return;

    const float radius = float(data->mass); // Mass equals radius by construction.
    const PS_AGONY2D::Vec2 pos = data->offset;
    const PS_AGONY2D::Vec2 vel = data->velocity;

    simulation.destroyBody(event.selfBody);

    // Children (radius/2) sit inside the parent's footprint, side by side along the contact tangent: no overlap.
    const float half = radius * 0.5f;
    const PS_AGONY2D::Vec2 dir = { -event.normal.y, event.normal.x };
    spawnSplitCircle(ctx, { pos.x + dir.x * half, pos.y + dir.y * half }, { vel.x + dir.x, vel.y + dir.y }, half);
    spawnSplitCircle(ctx, { pos.x - dir.x * half, pos.y - dir.y * half }, { vel.x - dir.x, vel.y - dir.y }, half);
}

static void load_CollisionCallbackTest(PS_AGONY2D::Simulation& simulation)
{
    using namespace PS_AGONY2D;

    const MaterialIndex mat = simulation.createMaterial({
        .elasticity = 0.3f,
        .staticFriction = 0.6f,
        .dynamicFriction = 0.4f
        });

    // Closed static room: interior x in [-20, 20], y in [-14, 14].
    auto wall = [&](float x, float y, float w, float h)
        {
            simulation.createBox({
                .base.position = { x, y },
                .base.mass = 0,
                .base.materialIndex = mat,
                .size = { w, h }
                });
        };
    wall(0.0f, -15.0f, 44.0f, 2.0f);
    wall(0.0f, 15.0f, 44.0f, 2.0f);
    wall(-21.0f, 0.0f, 2.0f, 32.0f);
    wall(21.0f, 0.0f, 2.0f, 32.0f);

    // 5 splits: 3.2 -> 1.6 -> 0.8 -> 0.4 -> 0.2 -> 0.1 (powers of two, exact in float).
    constexpr float startRadius = 3.2f;
    static SplitContext ctx; // Static: referenced by the callback for the whole scene lifetime.
    ctx.simulation = &simulation;
    ctx.material = mat;
    ctx.minRadius = 0.1f;

    const auto callback = simulation.createCollisionCallback(onCircleSplit, &ctx);
    if (!callback) return;
    ctx.callback = *callback;

    spawnSplitCircle(ctx, { 0.0f, 8.0f }, { 0.0f, 0.0f }, startRadius);
}

void loadScene(PS_AGONY2D::Simulation& simulation, int scene)
{
    Ecstasy::Core::Random::Generator rvg{}; // Random value generator.
    rvg.setSeed(0);

    if (scene == 0)
    {
        load_LargeWorld(simulation, rvg, {}, 1500);

        //load_LargeWorld(simulation, rvg, { -70.0f, -50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, {  70.0f, -50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, { -70.0f,  50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, {  70.0f,  50.0f }, 2500 / 4);
    }
    else if (scene == 1)
    {
        load_StackedPyramid(simulation, rvg);
    }
    else if (scene == 2)
    {
        load_ConstraintBridges(simulation, rvg);
    }
    else if (scene == 3)
    {
        load_CarRamp(simulation, rvg);
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