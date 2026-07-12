#include "Scenes.h"

#include "EcstasyCore/Random.h"

#include "Physics/Simulation.h"

#include <cmath>


static PS_AGONY::Real cross(
    const PS_AGONY::Vec2& a,
    const PS_AGONY::Vec2& b,
    const PS_AGONY::Vec2& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<PS_AGONY::Vec2> makeConvexPolygon(Ecstasy::Random::Generator& rvg, size_t targetVertices, float radius)
{
    const size_t sampleCount = std::max<size_t>(targetVertices * 4, 12);

    std::vector<PS_AGONY::Vec2> pts;
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

    std::vector<PS_AGONY::Vec2> hull;
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


void load_GaltonBoard(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    constexpr int ballCount = 1500;
    constexpr float ballRadius = 0.05f;

    constexpr int pegRows = 20;
    constexpr int pegColumns = 40;
    constexpr float pegRadius = 0.04f;
    constexpr float pegTopY = 0.0f;

    constexpr float binHeight = 5.0f;
    constexpr float binWidth = 0.01f;

    constexpr float wallThickness = 0.1f;
    constexpr float wallHeight = binHeight + 8.0f;

    constexpr float funnelOpeningWidth = ballRadius * 5.0f * 1.5f;
    constexpr float funnelRotation = 0.9f;
    constexpr float funnelLength = 10.0f;
    constexpr float funnelWidth = 0.05f;
    constexpr float funnelYOffset = 2.0f;

    //
    constexpr float pegSpacing = (ballRadius + 0.03f + pegRadius) * 2.0f;
    constexpr float pegBottomY = pegTopY - pegRows * pegSpacing - pegRadius;
    constexpr float pegXBoundary = float(pegColumns - 1) * 0.5f * pegSpacing + pegRadius;


    PS_AGONY::Material material0 = {
        .elasticity = 0.8,
        .staticFriction = 0.0,
        .dynamicFriction = 0.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);

    // Pegs.
    for (int y = 0; y < pegRows; y++)
    {
        const float pegY = pegTopY - pegRadius - y * pegSpacing;

        const int columns = (y & 1) == 0 ? pegColumns : pegColumns - 1;
        const float offset = (y & 1) == 0 ? 0.0f : pegSpacing * 0.5f;
        for (int x = 0; x < columns; x++)
        {
            const float pegX = -pegXBoundary + pegRadius + pegSpacing * x + offset;

            simulation.createCircle({
                .base.position = { pegX, pegY },
                .base.velocity = { 0, 0 },
                .base.rotation = 0,
                .base.angularVelocity = 0,
                .base.mass = 0,
                .base.materialIndex = material0Index,
                .radius = pegRadius
            });
        }
    }

    // Walls.
    {
        constexpr float wallInnerBoundaryX = pegXBoundary + pegSpacing - pegRadius;
        constexpr float wallInnerBottomY = pegBottomY - binHeight;

        simulation.createBox({
            .base.position = { 0.0f, wallInnerBottomY - wallThickness * 0.5f },
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .size = { wallInnerBoundaryX * 2.0f, wallThickness }
        });

        simulation.createBox({
            .base.position = {-(wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .size = { wallThickness, wallHeight }
        });

        simulation.createBox({
            .base.position = { (wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .size = { wallThickness, wallHeight }
        });
    }

    // Bins.
    {
        for (int x = 0; x < pegColumns; x++)
        {
            const float binX = -pegXBoundary + pegRadius + pegSpacing * x;

            simulation.createBox({
                .base.position = { binX, pegBottomY - binHeight * 0.5f },
                .base.mass = 0,
                .base.materialIndex = material0Index,
                .size = { binWidth, binHeight }
            });
        }
    }

    // Funnel.
    {
        const float funnelX = funnelOpeningWidth * 0.5f + std::sin(funnelRotation) * funnelLength * 0.5f;
        const float funnelY = funnelYOffset + std::cos(funnelRotation) * funnelLength * 0.5f;

        simulation.createBox({
            .base.position = { -funnelX, funnelY },
            .base.rotation = funnelRotation,
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .size = { funnelWidth, funnelLength }
        });

        simulation.createBox({
            .base.position = { funnelX, funnelY },
            .base.rotation = -funnelRotation,
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .size = { funnelWidth, funnelLength }
        });
    }

    // Balls.
    {
        constexpr float spawnBottom = 0.1f + ballRadius + funnelYOffset;

        const float rotTan = std::tan(funnelRotation);
        const float rndStart = ballRadius * 5.0f;
        const float rndEnd = std::cos(funnelRotation) * funnelLength - ballRadius;

        for (int i = 0; i < ballCount; i++)
        {
            float rnd = rvg.real<float>(0.0f, 1.0f);
            rnd = 1.0f - rnd;
            rnd *= rnd;
            rnd = 1.0f - rnd;
            rnd = rndStart + rnd * (rndEnd - rndStart);

            const float xBorder = funnelOpeningWidth * 0.5f + rnd * rotTan;

            const float x = rvg.real<float>(-xBorder, xBorder);
            const float y = spawnBottom + rnd;

            simulation.createCircle({
                .base.position = { x, y },
                .base.velocity = { 0, 0 },
                .base.rotation = 0,
                .base.angularVelocity = 0,
                .base.mass = 1,
                .base.materialIndex = material0Index,
                .radius = ballRadius
                });
        }
    }
}

void load_ALotOfNotTouching(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    constexpr int rowCount = 40;
    constexpr int columnCount = 40;

    PS_AGONY::Material material0 = {
            .elasticity = 0.7,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);

    for (int ix = 0; ix < rowCount; ix++)
    for (int iy = 0; iy < columnCount; iy++)
    {
        const float x = ix * 2.1f;
        const float y = iy * 2.1f;

        //simulation.createCircle({
        //    .base.position = { x, y },
        //    .base.velocity = { 0, 0 },
        //    .base.rotation = 0,
        //    .base.angularVelocity = 0,
        //    .base.mass = 1,
        //    .base.materialIndex = material0Index,
        //    .radius = 1
        //});

        constexpr size_t maxVerticesCount = 10;
        const size_t verticesCount = rvg.integer<size_t>(3, maxVerticesCount);
        auto localVertices = makeConvexPolygon(rvg, verticesCount, 1);
        simulation.createPolygon({
            .base.position = { x, y },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.mass = 1,
            .base.materialIndex = material0Index,
            .localVertices = localVertices.data(),
            .verticesCount = verticesCount
        });
    }

    simulation.createCircle({
            .base.position = { -5, 0 },
            .base.velocity = {  0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = 20,
            .base.materialIndex = material0Index,
            .radius = 4
        });
}

void load_LargeWorld(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    // === CONFIGURATION VARIABLES ===
    constexpr float worldWidth = 125.0f;
    constexpr float terrainSegmentWidth = 16.0f;
    constexpr float wallThickness = 5.0f;
    constexpr float wallHeight = 50.0f;
    constexpr float terrainBaseY = -20.0f;

    // Configurable object counts
    constexpr int circleCount = 1500;
    constexpr int boxCount = 1500;
    constexpr int polygonCount = 1500;

    // === 1. MATERIAL SETUP ===
    PS_AGONY::Material worldMaterial = {
        .elasticity = 0.9f,
        .staticFriction = 1.0f,
        .dynamicFriction = 0.5f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(worldMaterial);

    // === 2. HIGH BOUNDS (SIDE WALLS) ===
    // Left Bounding Wall
    simulation.createBox({
        .base.position = { -worldWidth * 0.5f - wallThickness * 0.5f, wallHeight * 0.5f + terrainBaseY },
        .base.mass = 0, // Static
        .base.materialIndex = materialIdx,
        .size = { wallThickness, wallHeight }
        });

    // Right Bounding Wall
    simulation.createBox({
        .base.position = { worldWidth * 0.5f + wallThickness * 0.5f, wallHeight * 0.5f + terrainBaseY },
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
        PS_AGONY::Vec2 localVertices[4] = {
            { -halfSegmentWidth, 0.0f },             // Bottom-Left
            {  halfSegmentWidth, 0.0f },             // Bottom-Right
            {  halfSegmentWidth, y2 - terrainBaseY }, // Top-Right
            { -halfSegmentWidth, y1 - terrainBaseY }  // Top-Left
        };

        simulation.createPolygon({
            .base.position = { posX, posY },
            .base.mass = 0,
            .base.materialIndex = materialIdx,
            .localVertices = localVertices,
            .verticesCount = 4
            });
    }

    // === 4. EVENLY SPREAD DYNAMIC OBJECTS ===
    const float spawnBufferX = worldWidth * 0.45f; // Keep spawns slightly away from outer edge walls
    constexpr float spawnMaxY = wallHeight + terrainBaseY;
    constexpr float spawnMinY = terrainBaseY + 25.0f;

    // Spawn Circles
    for (int i = 0; i < circleCount; ++i) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float r = rvg.real<float>(0.15f, 0.5f);
        float mass = 3.14159f * r * r;

        simulation.createCircle({
            .base.position = { x, y },
            .base.velocity = { vx, vy },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .radius = r
            });
    }

    // Spawn Boxes
    for (int i = 0; i < boxCount; ++i) {
        float x = rvg.real<float>(-spawnBufferX, spawnBufferX);
        float y = rvg.real<float>(spawnMinY, spawnMaxY);
        float vx = rvg.real<float>(-1.5f, 1.5f);
        float vy = rvg.real<float>(-1.0f, 0.0f);
        float rotation = rvg.real<float>(0.0f, 6.28f);
        float width = rvg.real<float>(0.3f, 0.8f);
        float height = rvg.real<float>(0.3f, 0.8f);
        float mass = width * height;

        simulation.createBox({
            .base.position = { x, y },
            .base.velocity = { vx, vy },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .size = { width, height }
            });
    }

    // Spawn Random Convex Polygons
    for (int i = 0; i < polygonCount; i++) {
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
            .base.position = { x, y },
            .base.velocity = { vx, vy },
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = materialIdx,
            .localVertices = localVertices.data(),
            .verticesCount = verticesCount
            });
    }
}

void load_StackedPyramid(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    // === CONFIGURATION VARIABLES ===
    constexpr int pyramidRows = 40; // Number of rows in the pyramid
    constexpr float boxWidth = 0.5f;
    constexpr float boxHeight = 0.5f;
    constexpr float boxMass = 1.0f;

    constexpr float floorThickness = 2.0f;
    constexpr float floorWidth = (pyramidRows + 8) * boxWidth * 3;

    // === 1. MATERIAL SETUP ===
    // High friction and lower elasticity help the pyramid remain stable upon initialization
    PS_AGONY::Material pyramidMaterial = {
        .elasticity = 0.1f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(pyramidMaterial);

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

void load_SpringBridge(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    // === 1. CONFIGURATION AND MATERIALS ===
    constexpr float cliffWidth = 25.0f;
    constexpr float cliffHeight = 30.0f;
    constexpr float cliffY = -cliffHeight * 0.5f; // Top surface will align perfectly with Y = 0.0
    constexpr float gapWidth = 32.0f;

    constexpr int plankCount = 14;
    constexpr float plankHeight = 0.5f;
    constexpr float plankMass = 1.5f;

    // Calculate precise width per plank to span the gap with minor breathing spacing
    constexpr float totalSpanWidth = gapWidth;
    constexpr float plankStep = totalSpanWidth / static_cast<float>(plankCount);
    constexpr float plankWidth = plankStep * 0.9f;

    // Spring mechanics configuration variables
    constexpr float springK = 2200.0f;  // Structural stiffness
    constexpr float springD = 18.0f;    // Joint damping
    constexpr float verticalOffset = plankHeight * 0.35f; // Dual-anchor separation to prevent torsional flipping

    PS_AGONY::Material physicsMaterial = {
        .elasticity = 0.2f,
        .staticFriction = 0.7f,
        .dynamicFriction = 0.5f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(physicsMaterial);

    // === 2. CREATE HUGE TERRAIN CLIFFS ===
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

    // Halt if the terrain cliff allocations failed
    if (!leftCliffOpt || !rightCliffOpt) return;

    PS_AGONY::BodyIndex leftCliff = *leftCliffOpt;
    PS_AGONY::BodyIndex rightCliff = *rightCliffOpt;

    // === 3. SPAN BRIDGE PLANKS & CONNECT WITH SPRINGS ===
    std::vector<PS_AGONY::BodyIndex> planks;
    planks.reserve(plankCount);

    const float spanStartX = -gapWidth * 0.5f;

    for (int i = 0; i < plankCount; ++i)
    {
        float plankX = spanStartX + plankStep * (static_cast<float>(i) + 0.5f);

        auto plankOpt = simulation.createBox({
            .base.position = { plankX, plankHeight * -0.5f },
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

    // Early out if no bridge planks could be generated
    if (planks.empty()) return;

    // Helper lambda to construct dual-anchor spring pairs cleanly between structural components
    auto attachWithDualSprings = [&](PS_AGONY::BodyIndex bodyA, PS_AGONY::BodyIndex bodyB,
        PS_AGONY::Vec2 localA, PS_AGONY::Vec2 localB,
        float restLen) {
            // Upper Support Spring
            simulation.createSpring({
                .bodyIndexA = bodyA,
                .bodyIndexB = bodyB,
                .localAnchorA = { localA.x, localA.y + verticalOffset },
                .localAnchorB = { localB.x, localB.y + verticalOffset },
                .restLength = restLen,
                .stiffness = springK,
                .damping = springD
                });
            // Lower Support Spring
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

    // Connect Left Cliff Anchor to First Plank
    float cliffToPlankRestLen = std::abs((spanStartX + plankStep * 0.5f - plankWidth * 0.5f) - (-gapWidth * 0.5f));
    attachWithDualSprings(leftCliff, planks.front(), { cliffWidth * 0.5f, cliffHeight * 0.5f - plankHeight * 0.5f }, { -plankWidth * 0.5f, 0.0f }, cliffToPlankRestLen);

    // Connect Continuous Plank Sequence Chains
    float interPlankRestLen = plankStep - plankWidth;
    for (size_t i = 0; i < planks.size() - 1; ++i)
    {
        attachWithDualSprings(planks[i], planks[i + 1], { plankWidth * 0.5f, 0.0f }, { -plankWidth * 0.5f, 0.0f }, interPlankRestLen);
    }

    // Connect Last Plank to Right Cliff Anchor
    attachWithDualSprings(planks.back(), rightCliff, { plankWidth * 0.5f, 0.0f }, { -cliffWidth * 0.5f, cliffHeight * 0.5f - plankHeight * 0.5f }, cliffToPlankRestLen);

    // === 4. SPAWN INTERACTIONS (DECORATIVE INTERACTION BALLS) ===
    for (int i = 0; i < 5; ++i)
    {
        float rx = rvg.real<float>(-gapWidth * 0.35f, gapWidth * 0.35f);
        float ry = rvg.real<float>(4.0f, 12.0f);
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

void load_SoftBodyStressTest(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    // === 1. CONFIGURATION AND VARIABLES ===
    constexpr int gridWidth = 90;   // Number of horizontal particles
    constexpr int gridHeight = 90;  // Number of vertical particles
    constexpr float spacing = 0.3f;
    constexpr float radius = spacing * 0.5f * 0.5;
    constexpr float particleMass = 0.2f;

    // Spring constants chosen for elastic but stable structural behavior
    constexpr float springK = 4000.0f;
    constexpr float springD = 0.0f;

    PS_AGONY::Material physicsMaterial = {
        .elasticity = 0.1f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(physicsMaterial);

    // === 2. STATIC GROUND PLATFORM ===
    simulation.createBox({
        .base.position = { 0.0f, -8.0f },
        .base.mass = 0, // Static platform
        .base.materialIndex = materialIdx,
        .size = { 2000.0f, 2.0f }
        });

    // === 3. SPAWN PARTICLE GRID ===
    // Array to store created IDs; standard flat vector mapping layout: index = y * gridWidth + x
    std::vector<std::optional<PS_AGONY::BodyIndex>> gridNodeMap(gridWidth * gridHeight, std::nullopt);

    const float startX = -static_cast<float>(gridWidth - 1) * spacing * 0.5f;
    const float startY = 2.0f; // Elevate above the ground box

    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            float posX = startX + static_cast<float>(x) * spacing;
            float posY = startY + static_cast<float>(y) * spacing;

            // Slight offset or initialization tilt to stimulate dynamic cloth folding deformation
            auto ballOpt = simulation.createCircle({
                .base.position = { posX, posY },
                .base.velocity = { 1.5f, -3.0f }, // Initial throw velocity vector
                .base.rotation = 0.0f,
                .base.angularVelocity = 0.0f,
                .base.mass = particleMass,
                .base.materialIndex = materialIdx,
                .radius = radius
                });

            gridNodeMap[y * gridWidth + x] = ballOpt;
        }
    }

    // === 4. GENERATE MESH OF SPRING CONSTRAINTS ===
    // Lambda helper to safely tie nodes center-to-center if both allocations succeeded
    auto tryConnectSpring = [&](int x1, int y1, int x2, int y2, float restLength)
        {
            auto nodeA = gridNodeMap[y1 * gridWidth + x1];
            auto nodeB = gridNodeMap[y2 * gridWidth + x2];

            if (nodeA && nodeB)
            {
                simulation.createSpring({
                    .bodyIndexA = *nodeA,
                    .bodyIndexB = *nodeB,
                    .localAnchorA = { 0.0f, 0.0f }, // Center anchor
                    .localAnchorB = { 0.0f, 0.0f }, // Center anchor
                    .restLength = restLength,
                    .stiffness = springK,
                    .damping = springD
                    });
            }
        };

    const float diagSpacing = std::sqrt(2.0f) * spacing;

    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            // Structural Horizontal Constraints (Right)
            if (x < gridWidth - 1)
            {
                tryConnectSpring(x, y, x + 1, y, spacing);
            }

            // Structural Vertical Constraints (Down)
            if (y < gridHeight - 1)
            {
                tryConnectSpring(x, y, x, y + 1, spacing);
            }

            // Shear Diagonal Constraints (Down-Right)
            if (x < gridWidth - 1 && y < gridHeight - 1)
            {
                tryConnectSpring(x, y, x + 1, y + 1, diagSpacing);
            }

            // Shear Diagonal Constraints (Down-Left)
            if (x > 0 && y < gridHeight - 1)
            {
                tryConnectSpring(x, y, x - 1, y + 1, diagSpacing);
            }
        }
    }
}

void loadScene(PS_AGONY::Simulation& simulation, int scene)
{
    Ecstasy::Random::Generator rvg; // Random value generator.
    rvg.setSeed(0);

	if (scene == 0)
	{
		load_GaltonBoard(simulation, rvg);
	}
    else if (scene == 1)
    {
        load_ALotOfNotTouching(simulation, rvg);
    }
    else if (scene == 2)
    {
        load_LargeWorld(simulation, rvg);
    }
    else if (scene == 3)
    {
        load_StackedPyramid(simulation, rvg);
    }
    else if (scene == 4)
    {
        load_SpringBridge(simulation, rvg);
    }
    else if (scene == 5)
    {
        load_SoftBodyStressTest(simulation, rvg);
    }
}
