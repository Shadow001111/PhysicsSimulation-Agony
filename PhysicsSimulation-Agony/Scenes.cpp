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



void load_ALotOfCollisions(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg, float globalOffsetX, float globalOffsetY)
{
    constexpr float boundary = 20.0f;
    constexpr float thickness = 5.0f;

    constexpr int circleCount = 1000;// 2'500;
    constexpr int boxCount = 1000;// 2'500;
    constexpr int polygonCount = 1000;// 2'500;

    PS_AGONY::Material material0 = {
            .elasticity = 0.9,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
    };

    PS_AGONY::Material material1 = {
        .elasticity = 0.5,
        .staticFriction = 1.0,
        .dynamicFriction = 1.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);
    PS_AGONY::MaterialIndex material1Index = simulation.createMaterial(material1);

    constexpr float spawnBoundary = boundary - 1.0f;
    {
        constexpr float halfThickness = thickness * 0.5f;
        constexpr float length = boundary * 2.0f + 2.0f;
        const float angularVelocity = 0.0f;

        const float b = boundary + halfThickness;

        const PS_AGONY::Vec2 p1{ globalOffsetX - b, globalOffsetY };
        const PS_AGONY::Vec2 p2{ globalOffsetX + b, globalOffsetY };
        const PS_AGONY::Vec2 p3{ globalOffsetX, globalOffsetY - b };
        const PS_AGONY::Vec2 p4{ globalOffsetX, globalOffsetY + b };

        simulation.createBox({
            .base.position = p1,
            .base.angularVelocity = angularVelocity,
            .base.mass = 0,
            .base.centerOfMass = -p1,
            .base.materialIndex = material0Index,
            .size = { thickness, length }
        });

        simulation.createBox({
            .base.position = p2,
            .base.angularVelocity = angularVelocity,
            .base.mass = 0,
            .base.centerOfMass = -p2,
            .base.materialIndex = material0Index,
            .size = { thickness, length }
            });

        simulation.createBox({
            .base.position = p3,
            .base.angularVelocity = angularVelocity,
            .base.mass = 0,
            .base.centerOfMass = -p3,
            .base.materialIndex = material0Index,
            .size = { length, thickness }
            });

        simulation.createBox({
            .base.position = p4,
            .base.angularVelocity = angularVelocity,
            .base.mass = 0,
            .base.centerOfMass = -p4,
            .base.materialIndex = material0Index,
            .size = { length, thickness }
            });
    }
    for (int i = 0; i < circleCount; i++)
    {
        const float x = globalOffsetX + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float y = globalOffsetY + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float vx = rvg.real<float>(-2.0f, 2.0f);
        const float vy = rvg.real<float>(-2.0f, 2.0f);
        const float r = rvg.real<float>(0.1f, 0.15f);
        const float mass = 3.14f * r * r;

        simulation.createCircle({
            .base.position = { x, y },
            .base.velocity = { vx, vy },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = mass,
            .base.materialIndex = material0Index,
            .radius = r
            });
    }
    for (int i = 0; i < boxCount; i++)
    {
        const float x = globalOffsetX + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float y = globalOffsetY + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float vx = rvg.real<float>(-2.0f, 2.0f);
        const float vy = rvg.real<float>(-2.0f, 2.0f);
        const float rotation = rvg.real<float>(0.0f, 6.28f);
        const float width = rvg.real<float>(0.2f, 0.4f);
        const float height = rvg.real<float>(0.2f, 0.4f);
        const float mass = width * height;

        simulation.createBox({
            .base.position = { x, y},
            .base.velocity = { vx, vy},
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = material0Index,
            .size = { width, height }
        });
    }
    for (int i = 0; i < polygonCount; i++)
    {
        const float x = globalOffsetX + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float y = globalOffsetY + rvg.real<float>(-spawnBoundary, spawnBoundary);
        const float vx = rvg.real<float>(-2.0f, 2.0f);
        const float vy = rvg.real<float>(-2.0f, 2.0f);
        const float rotation = 0;
        const float r = rvg.real<float>(0.1f, 0.15f) * 2.0f;
        const float mass = 1.0f;

        constexpr size_t maxVerticesCount = 10;
        const size_t verticesCount = rvg.integer<size_t>(3, maxVerticesCount);

        auto localVertices = makeConvexPolygon(rvg, verticesCount, r);

        simulation.createPolygon({
            .base.position = { x, y },
            .base.velocity = { vx, vy},
            .base.rotation = rotation,
            .base.mass = mass,
            .base.materialIndex = material0Index,
            .localVertices = localVertices.data(),
            .verticesCount = verticesCount
            });
    }

    {
        constexpr float radius = 4.0f;

        simulation.createCircle({
            .base.position = { 0, 0 },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = 100,
            .base.materialIndex = material1Index,
            .radius = radius
        });
    }
}

void load_CleanPerfomanceOfContactsTest(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    constexpr int objectCount = 500;
    constexpr float boxSize = 0.1f;
    constexpr float boxPadding = 0.03f;

    constexpr float paddingY = 5.0f;

    constexpr float floorThickness = 1.0f;
    constexpr float floorWidth = objectCount * (boxSize + boxPadding);


    PS_AGONY::Material material0 = {
            .elasticity = 0.9,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);

    simulation.createBox({
        .base.position = { 0.0f,  -floorThickness * 0.5f},
        .base.mass = 0,
        .base.materialIndex = material0Index,
        .size = { floorWidth, floorThickness }
    });

    simulation.createBox({
        .base.position = { 0.0f,  -floorThickness * 0.5f + paddingY},
        .base.mass = 0,
        .base.materialIndex = material0Index,
        .size = { floorWidth, floorThickness }
    });

    constexpr float dx = boxSize + boxPadding;
    for (int i = 0; i < objectCount; i++)
    {
        const float x = (-floorWidth + boxSize + boxPadding) * 0.5f + i * dx;
        
        simulation.createBox({
            .base.position = { x, boxSize * 0.5f},
            .base.mass = 1,
            .base.materialIndex = material0Index,
            .size = { boxSize, boxSize }
        });

        simulation.createCircle({
            .base.position = { x, boxSize * 0.5f + paddingY },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = 1,
            .base.materialIndex = material0Index,
            .radius = boxSize * 0.5f
        });
    }
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

void load_Planet(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    constexpr float planetRadius = 40.0f;

    constexpr int circleCount = 800;

    PS_AGONY::Material material0 = {
            .elasticity = 0.9,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
    };

    PS_AGONY::Material material1 = {
        .elasticity = 0.0,
        .staticFriction = 0.0,
        .dynamicFriction = 0.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);
    PS_AGONY::MaterialIndex material1Index = simulation.createMaterial(material1);

    simulation.createCircle({
            .base.position = { 0, 0 },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = 0,
            .base.materialIndex = material0Index,
            .radius = planetRadius
        });

    for (int i = 0; i < circleCount; i++)
    {
        const float angle = rvg.real<float>(0.0f, 6.28f);
        const float r = rvg.real<float>(0.1f, 1.0f);
        const float mass = 3.14f * r * r;

        const float dist = planetRadius + r;
        const float x = std::cos(angle) * dist;
        const float y = std::sin(angle) * dist;

        simulation.createCircle({
            .base.position = { x, y },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = mass,
            .base.materialIndex = material0Index,
            .radius = r
            });
    }

    {
        constexpr float radius = 4.0f;
        simulation.createCircle({
            .base.position = { 0.0, planetRadius + radius },
            .base.velocity = { 0, 0 },
            .base.rotation = 0,
            .base.angularVelocity = 0,
            .base.mass = 20000,
            .base.materialIndex = material1Index,
            .radius = radius
            });
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
    constexpr float worldWidth = 35.0f;
    constexpr float terrainSegmentWidth = 16.0f;
    constexpr float wallThickness = 5.0f;
    constexpr float wallHeight = 50.0f;
    constexpr float terrainBaseY = -20.0f;

    // Configurable object counts
    constexpr int circleCount = 2;
    constexpr int boxCount = 2;
    constexpr int polygonCount = 2;

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
    constexpr float boxWidth = 0.6f;
    constexpr float boxHeight = 0.6f;
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

void load_StackedTower(PS_AGONY::Simulation& simulation, Ecstasy::Random::Generator& rvg)
{
    constexpr int towerRows = 100;
    constexpr float boxSize = 0.6f;
    constexpr float boxMass = 1.0f;

    constexpr float boundaryThickness = 2.0f;
    constexpr float floorWidth = 100.0f;

    PS_AGONY::Material towerMaterial = {
        .elasticity = 0.9f,
        .staticFriction = 0.0f,
        .dynamicFriction = 0.0f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(towerMaterial);


    simulation.createBox({
        .base.position = { 0.0f, -boundaryThickness * 0.5f },
        .base.mass = 0,
        .base.materialIndex = materialIdx,
        .size = { floorWidth, boundaryThickness }
        });

    constexpr float wallHeight = towerRows * boxSize;
    simulation.createBox({
        .base.position = { -(boundaryThickness + boxSize) * 0.5f, wallHeight * 0.5f },
        .base.mass = 0,
        .base.materialIndex = materialIdx,
        .size = { boundaryThickness, wallHeight  }
        });
    simulation.createBox({
        .base.position = {  (boundaryThickness + boxSize) * 0.5f, wallHeight * 0.5f },
        .base.mass = 0,
        .base.materialIndex = materialIdx,
        .size = { boundaryThickness, wallHeight  }
        });


    for (int i = 0; i < towerRows; ++i)
    {
        float y = (i + 0.5f) * boxSize;

        simulation.createBox({
            .base.position = { 0.0f, y },
            .base.velocity = { 0.0f, 0.0f },
            .base.rotation = 0.0f,
            .base.angularVelocity = 0.0f,
            .base.mass = boxMass,
            .base.materialIndex = materialIdx,
            .size = { boxSize, boxSize }
            });
    }
}

void loadScene(PS_AGONY::Simulation& simulation, int scene)
{
    Ecstasy::Random::Generator rvg; // Random value generator.
    rvg.setSeed(0);

	if (scene == 0)
	{
        load_ALotOfCollisions(simulation, rvg, 0.0f, 0.0f);
		//load_ALotOfCollisions(simulation, rvg, -30.0f, 0.0f);
        //load_ALotOfCollisions(simulation, rvg,  30.0f, 0.0f);
	}
	else if (scene == 1)
	{
		load_CleanPerfomanceOfContactsTest(simulation, rvg);
	}
	else if (scene == 2)
	{
		load_GaltonBoard(simulation, rvg);
	}
	else if (scene == 3)
	{
		load_Planet(simulation, rvg);
	}
    else if (scene == 4)
    {
        load_ALotOfNotTouching(simulation, rvg);
    }
    else if (scene == 5)
    {
        load_LargeWorld(simulation, rvg);
    }
    else if (scene == 6)
    {
        load_StackedPyramid(simulation, rvg);
    }
    else if (scene == 7)
    {
        load_StackedTower(simulation, rvg);
    }
}
