#include "Scenes.h"

#include "Ecstasy/Core/Random.h"

#include "Physics/Simulation.h"

#include <cmath>


struct SceneOrientation
{
    float offsetX = 0;
    float offsetY = 0;
};


static PS_AGONY::Real cross(
    const PS_AGONY::Vec2& a,
    const PS_AGONY::Vec2& b,
    const PS_AGONY::Vec2& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<PS_AGONY::Vec2> makeConvexPolygon(Ecstasy::Core::Random::Generator& rvg, size_t targetVertices, float radius)
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


// Chassis-center height above flat ground at rest, for scale = 1 (multiply by scale).
static constexpr float kCarRestHeight = 0.9f + 2.2f + 0.02f;

void load_ObjectCar(PS_AGONY::Simulation& simulation,
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
    const PS_AGONY::MaterialIndex carMatIdx = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 0.8f, .dynamicFriction = 0.6f });

    const PS_AGONY::MaterialIndex wheelMatIdx = simulation.createMaterial({
        .elasticity = 0.1f, .staticFriction = 1.2f, .dynamicFriction = 0.9f });

    // === CHASSIS-LOCAL -> WORLD ===
    const float cosR = std::cos(rotation);
    const float sinR = std::sin(rotation);
    auto toWorld = [&](float lx, float ly) -> PS_AGONY::Vec2 {
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
    const PS_AGONY::ObjectIndex chassis = *chassisOpt;

    // === WHEEL FACTORY ===
    auto createWheel = [&](float side) -> std::optional<PS_AGONY::ObjectIndex> {
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
    auto createSuspension = [&](PS_AGONY::ObjectIndex wheel, float side)
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


void load_GaltonBoard(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
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
        .elasticity = 0.5,
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

void load_ALotOfNotTouching(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
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

void load_LargeWorld(
    PS_AGONY::Simulation& simulation,
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
    PS_AGONY::Material worldMaterial = {
        .elasticity = 0.9f,
        .staticFriction = 0.8f,
        .dynamicFriction = 0.6f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(worldMaterial);

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
        PS_AGONY::Vec2 localVertices[4] = {
            { -halfSegmentWidth, 0.0f },             // Bottom-Left
            {  halfSegmentWidth, 0.0f },             // Bottom-Right
            {  halfSegmentWidth, y2 - terrainBaseY }, // Top-Right
            { -halfSegmentWidth, y1 - terrainBaseY }  // Top-Left
        };

        simulation.createPolygon({
            .base.position = { posX + sceneOrientation.offsetX, posY + sceneOrientation.offsetY },
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
            .localVertices = localVertices.data(),
            .verticesCount = verticesCount
            });
    }
}

void load_StackedPyramid(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
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

void load_ConstraintBridges(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
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

    PS_AGONY::Material physicsMaterial = {
        .elasticity = 0.2f,
        .staticFriction = 0.7f,
        .dynamicFriction = 0.5f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(physicsMaterial);

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

    const PS_AGONY::ObjectIndex leftCliff = *leftCliffOpt;
    const PS_AGONY::ObjectIndex rightCliff = *rightCliffOpt;

    const float spanStartX = -gapWidth * 0.5f;

    // Distance between adjacent anchor points, shared by every constraint type -
    // springs use it as rest length, joints ignore it, rods use it as their fixed length.
    const float cliffToPlankRestLen = std::abs((spanStartX + plankStep * 0.5f - plankWidth * 0.5f) - (-gapWidth * 0.5f));
    const float interPlankRestLen = plankStep - plankWidth;

    // === 3. PER-CONSTRAINT-TYPE JOIN LAMBDAS ===
    // Each has the same signature (bodyA, bodyB, localA, localB, restLen), so buildBridge
    // below can stay a single generic lambda regardless of which constraint it's fed.

    auto joinWithSpring = [&](PS_AGONY::ObjectIndex bodyA, PS_AGONY::ObjectIndex bodyB,
        PS_AGONY::Vec2 localA, PS_AGONY::Vec2 localB, float restLen)
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

    auto joinWithJoint = [&](PS_AGONY::ObjectIndex bodyA, PS_AGONY::ObjectIndex bodyB,
        PS_AGONY::Vec2 localA, PS_AGONY::Vec2 localB, float /*restLen*/)
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

    auto joinWithRod = [&](PS_AGONY::ObjectIndex bodyA, PS_AGONY::ObjectIndex bodyB,
        PS_AGONY::Vec2 localA, PS_AGONY::Vec2 localB, float restLen)
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
            std::vector<PS_AGONY::ObjectIndex> planks;
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

            joinPlanks(leftCliff, planks.front(), PS_AGONY::Vec2{ cliffWidth * 0.5f, cliffLocalY }, PS_AGONY::Vec2{ -plankWidth * 0.5f, 0.0f }, cliffToPlankRestLen);

            for (size_t i = 0; i + 1 < planks.size(); ++i)
            {
                joinPlanks(planks[i], planks[i + 1], PS_AGONY::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY::Vec2{ -plankWidth * 0.5f, 0.0f }, interPlankRestLen);
            }

            joinPlanks(planks.back(), rightCliff, PS_AGONY::Vec2{ plankWidth * 0.5f, 0.0f }, PS_AGONY::Vec2{ -cliffWidth * 0.5f, cliffLocalY }, cliffToPlankRestLen);
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

void load_SoftBodyComparisonTest(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === 1. CONFIGURATION AND VARIABLES ===
    constexpr int gridWidth = 80;   // Number of horizontal particles.
    constexpr int gridHeight = 80;  // Number of vertical particles.
    constexpr float spacing = 0.5f;
    constexpr float radius = spacing * 0.5f * 0.5;
    constexpr float particleMass = 0.2f;

    // Spring constants chosen for elastic but stable structural behavior
    constexpr float springK = 4000.0f;
    constexpr float springD = 0.0f;

    constexpr float cubeGap = 8.0f; // Horizontal separation between the two soft-body cubes

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
        .size = { 3000.0f, 2.0f } // Longer ground
        });

    // === 3. HELPER: BUILD A SQUARE SOFT-BODY CUBE, LINKED WITH EITHER SPRINGS OR RODS ===
    const float diagSpacing = std::sqrt(2.0f) * spacing;
    const float cubeWidth = static_cast<float>(gridWidth - 1) * spacing;

    auto buildSoftBodyCube = [&](float centerX, bool useRods)
        {
            // Array to store created IDs; standard flat vector mapping layout: index = y * gridWidth + x
            std::vector<std::optional<PS_AGONY::ObjectIndex>> gridNodeMap(gridWidth * gridHeight, std::nullopt);

            const float startX = centerX - cubeWidth * 0.5f;
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

            // Lambda helper to safely tie nodes center-to-center if both allocations succeeded
            auto tryConnect = [&](int x1, int y1, int x2, int y2, float restLength)
                {
                    auto nodeA = gridNodeMap[y1 * gridWidth + x1];
                    auto nodeB = gridNodeMap[y2 * gridWidth + x2];

                    if (!nodeA || !nodeB) return;

                    if (useRods)
                    {
                        simulation.createRod({
                            .bodyIndexA = *nodeA,
                            .bodyIndexB = *nodeB,
                            .localAnchorA = { 0.0f, 0.0f }, // Center anchor
                            .localAnchorB = { 0.0f, 0.0f }, // Center anchor
                            .length = restLength
                            });
                    }
                    else
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

            for (int y = 0; y < gridHeight; ++y)
            {
                for (int x = 0; x < gridWidth; ++x)
                {
                    // Structural Horizontal Constraints (Right)
                    if (x < gridWidth - 1)
                    {
                        tryConnect(x, y, x + 1, y, spacing);
                    }

                    // Structural Vertical Constraints (Down)
                    if (y < gridHeight - 1)
                    {
                        tryConnect(x, y, x, y + 1, spacing);
                    }

                    // Shear Diagonal Constraints (Down-Right)
                    if (x < gridWidth - 1 && y < gridHeight - 1)
                    {
                        tryConnect(x, y, x + 1, y + 1, diagSpacing);
                    }

                    // Shear Diagonal Constraints (Down-Left)
                    if (x > 0 && y < gridHeight - 1)
                    {
                        tryConnect(x, y, x - 1, y + 1, diagSpacing);
                    }
                }
            }
        };

    // === 4. SPAWN BOTH CUBES, SEPARATED HORIZONTALLY ===
    const float halfOffset = cubeWidth * 0.5f + cubeGap * 0.5f;
    buildSoftBodyCube(-halfOffset, false); // Left cube: springs (original behavior)
    //buildSoftBodyCube(halfOffset, true);   // Right cube: rods
}

void load_CarRamp(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // Set up physical properties for the environment.
    PS_AGONY::Material envMaterial = {
        .elasticity = 0.05f,
        .staticFriction = 0.9f,
        .dynamicFriction = 0.7f
    };
    PS_AGONY::MaterialIndex envMatIdx = simulation.createMaterial(envMaterial);

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
    std::vector<PS_AGONY::Vec2> leftVertices;
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
    std::vector<PS_AGONY::Vec2> rightVertices;
    rightVertices.reserve(leftVertices.size());
    for (auto it = leftVertices.rbegin(); it != leftVertices.rend(); ++it)
    {
        rightVertices.push_back({ -it->x, it->y });
    }

    simulation.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = envMatIdx,
        .localVertices = leftVertices.data(),
        .verticesCount = static_cast<uint32_t>(leftVertices.size()),
        .decompositionMethod = PS_AGONY::PhysicsGeometry::DecompositionMethod::ReflexAngleBisector
        });

    simulation.createPolygon({
        .base.position = { 0.0f, 0.0f },
        .base.mass = 0,
        .base.materialIndex = envMatIdx,
        .localVertices = rightVertices.data(),
        .verticesCount = static_cast<uint32_t>(rightVertices.size()),
        .decompositionMethod = PS_AGONY::PhysicsGeometry::DecompositionMethod::ReflexAngleBisector
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
    constexpr int wallCols = 3;
    constexpr float boxSize = 2.0f;
    const float targetStartX = 50.0f;
    const float targetStartY = flatGroundY + (boxSize * 0.5f);

    PS_AGONY::Vec2 boxPolyVertices[4] = {
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
                    .localVertices = boxPolyVertices,
                    .verticesCount = 4
                    });
            }
        }
    }
}

void load_Test(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === 1. MATERIAL SETUP ===
    PS_AGONY::Material material = {
        .elasticity = 0.2f,
        .staticFriction = 0.6f,
        .dynamicFriction = 0.4f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(material);

    // === 2. STATIC FLOOR ===
    constexpr float floorWidth = 60.0f;
    constexpr float floorHeight = 2.0f;
    simulation.createBox({
        .base.position = { 0.0f, -floorHeight * 0.5f },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = 0.0f,
        .base.angularVelocity = 0.0f,
        .base.mass = 0.0f, // Static
        .base.materialIndex = materialIdx,
        .size = { floorWidth, floorHeight }
        });

    // === 3. SINGLE DYNAMIC LONG PLANK ===
    constexpr float plankWidth = 14.0f;
    constexpr float plankHeight = 0.4f;
    constexpr float plankMass = 8.0f;

    simulation.createBox({
        .base.position = { 0.0f, 6.0f },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = 0.25f, // Slightly tilted to make the drop and collision interesting
        .base.angularVelocity = 0.0f,
        .base.mass = plankMass,
        .base.materialIndex = materialIdx,
        .size = { plankWidth, plankHeight }
        });
}

void load_JointedCircles(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === 1. MATERIAL SETUP ===
    PS_AGONY::Material material = {
        .elasticity = 0.5f,
        .staticFriction = 0.5f,
        .dynamicFriction = 0.3f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(material);

    // === 2. STATIC CIRCLE (MASS = 0) ===
    auto staticCircleOpt = simulation.createCircle({
        .base.position = { 0.0f, 0.0f },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = 0.0f,
        .base.angularVelocity = 0.0f,
        .base.mass = 0.0f,
        .base.materialIndex = materialIdx,
        .radius = 0.8f
        });

    // === 3. DYNAMIC CIRCLE ===
    auto dynamicCircleOpt = simulation.createCircle({
        .base.position = { 4.0f, 0.0f },
        .base.velocity = { 0.0f, 0.0f },
        .base.rotation = 0.0f,
        .base.angularVelocity = 0.0f,
        .base.mass = 3.0f,
        .base.materialIndex = materialIdx,
        .radius = 1.0f
        });

    if (!staticCircleOpt || !dynamicCircleOpt) return;

    PS_AGONY::ObjectIndex staticCircle = *staticCircleOpt;
    PS_AGONY::ObjectIndex dynamicCircle = *dynamicCircleOpt;

    // === 4. JOINT CONNECTION ===
    constexpr float jointK = 50'000.0f;
    constexpr float jointD = 1000.0f;

    simulation.createRevoluteJoint({
        .bodyIndexA = staticCircle,
        .bodyIndexB = dynamicCircle,
        .localAnchorA = { 0.8f, 0.0f },
        .localAnchorB = { -4.0f, 0.0f },
        .stiffness = jointK,
        .damping = jointD
        });
}

// A row of piston rigs (static cylinder body + sliding piston body), each using a
// prismatic joint with different stiffness/damping/limit settings, to exercise the
// full parameter space in one visual pass.
//
// NOTE: assumes Simulation::createPrismaticJoint auto-captures referenceAngle from
// the two bodies' rotations at creation time (mirroring the SoA field comment), so
// it isn't passed here. Adjust if the actual API expects it explicitly.
void load_PrismaticJointGallery(PS_AGONY::Simulation& simulation, Ecstasy::Core::Random::Generator& rvg)
{
    // === 1. MATERIAL SETUP ===
    PS_AGONY::Material material = {
        .elasticity = 0.2f,
        .staticFriction = 0.5f,
        .dynamicFriction = 0.4f
    };
    PS_AGONY::MaterialIndex materialIdx = simulation.createMaterial(material);

    // === 2. RIG BUILDER ===
    // anchorPos/rotationRad define the static cylinder; axisLocal is the slide
    // direction in the cylinder's local space (rotates with it). axisOffset/perpOffset
    // place the piston's initial position along/off that axis, so misaligned starts
    // (perpOffset != 0) show the joint pulling the body back onto the axis.
    auto makePistonRig = [&](
        PS_AGONY::Vec2 anchorPos, float rotationRad, PS_AGONY::Vec2 axisLocal,
        float axisOffset, float perpOffset,
        float stiffness, float damping,
        float lowerLimit, float upperLimit, bool enableLimit,
        float sliderMass)
        {
            auto anchorOpt = simulation.createBox({
                .base.position = anchorPos,
                .base.velocity = { 0.0f, 0.0f },
                .base.rotation = rotationRad,
                .base.angularVelocity = 0.0f,
                .base.mass = 0.0f,
                .base.materialIndex = materialIdx,
                .size = { 1.0f, 0.5f }
                });

            if (!anchorOpt) return;
            PS_AGONY::ObjectIndex anchor = *anchorOpt;

            const float cosR = std::cos(rotationRad);
            const float sinR = std::sin(rotationRad);

            // World-space axis (rotates with the anchor, same convention as the joint).
            const float worldAxisX = axisLocal.x * cosR - axisLocal.y * sinR;
            const float worldAxisY = axisLocal.x * sinR + axisLocal.y * cosR;

            // World-space perpendicular (90 degrees CCW from axis), used only to seed
            // an off-axis starting position for testing the correction visually.
            const float worldPerpX = -worldAxisY;
            const float worldPerpY = worldAxisX;

            const float sliderX = anchorPos.x + worldAxisX * axisOffset + worldPerpX * perpOffset;
            const float sliderY = anchorPos.y + worldAxisY * axisOffset + worldPerpY * perpOffset;

            auto sliderOpt = simulation.createBox({
                .base.position = { sliderX, sliderY },
                .base.velocity = { 0.0f, 0.0f },
                .base.rotation = rotationRad,
                .base.angularVelocity = 0.0f,
                .base.mass = sliderMass,
                .base.materialIndex = materialIdx,
                .size = { 0.8f, 0.8f }
                });

            if (!sliderOpt) return;
            PS_AGONY::ObjectIndex slider = *sliderOpt;

            simulation.createPrismaticJoint({
                .bodyIndexA = anchor,
                .bodyIndexB = slider,
                .localAnchorA = { 0.0f, 0.0f },
                .localAnchorB = { 0.0f, 0.0f },
                .localAxisA = axisLocal,
                .stiffness = stiffness,
                .damping = damping,
                .lowerLimit = lowerLimit,
                .upperLimit = upperLimit,
                .enableLimit = enableLimit
                });
        };

    // === 3. RIG LAYOUT ===
    // Vertical piston shafts, axis pointing down so gravity drives the slider toward
    // the limit. Spaced along X, stiffness/damping ramp up left to right.
    constexpr float ceilingY = 6.0f;
    constexpr PS_AGONY::Vec2 downAxis = { 0.0f, -1.0f };

    // Very soft: sags loosely, oscillates a lot before settling near the limit.
    makePistonRig({ -12.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        /*stiffness*/ 100.0f, /*damping*/ 10.0f,
        /*lowerLimit*/ 0.0f, /*upperLimit*/ 4.0f, /*enableLimit*/ true,
        /*sliderMass*/ 1.0f);

    // Soft.
    makePistonRig({ -8.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        2'000.0f, 150.0f,
        0.0f, 4.0f, true,
        1.0f);

    // Medium.
    makePistonRig({ -4.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        20'000.0f, 800.0f,
        0.0f, 4.0f, true,
        1.0f);

    // Stiff.
    makePistonRig({ 0.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        200'000.0f, 4'000.0f,
        0.0f, 4.0f, true,
        1.0f);

    // Rigid: should barely deviate from the initial offset regardless of load.
    makePistonRig({ 4.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        2'000'000.0f, 20'000.0f,
        0.0f, 4.0f, true,
        1.0f);

    // No limit: same mid stiffness as the "Medium" rig, but free to fall indefinitely.
    makePistonRig({ 8.0f, ceilingY }, 0.0f, downAxis, 1.0f, 0.0f,
        20'000.0f, 800.0f,
        0.0f, 0.0f, false,
        1.0f);

    // Diagonal axis, deliberately started off-axis: exercises axis rotation with body A
    // and the axis-projection/clamp correction (visible as red-vs-blue divergence
    // in the debug render) instead of straight vertical sag.
    constexpr float diagonalRotation = 0.785398f; // 45 degrees.
    makePistonRig({ 12.0f, ceilingY }, diagonalRotation, PS_AGONY::Vec2{ 1.0f, 0.0f }, 1.0f, /*perpOffset*/ 1.0f,
        20'000.0f, 800.0f,
        -3.0f, 3.0f, true,
        1.0f);
}

// Hull-center height above flat ground at rest, for scale = 1 (multiply by scale).
// = wheelY (trackRadius + linkHeight / 2 + 5 mm) + suspensionDrop + hullHeight / 2.
static constexpr float kTankRestHeight = 0.725f + 1.2f + 0.6f;

void load_ObjectTank(PS_AGONY::Simulation& simulation,
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

    // Hull
    const float hullWidth = 8.0f * s;
    const float hullHeight = 1.2f * s;
    const float hullMass = 300.0f * massScale;

    // Wheels (all identical: road wheels, idler and sprocket)
    constexpr int wheelCount = 7;
    const float wheelRadius = 0.5f * s;
    const float wheelMass = 10.0f * massScale;
    const float wheelSpacing = 1.2f * s;                  // Center-to-center, leaves a 0.2 * s gap between wheels.

    // Track
    constexpr int linkCount = 70;
    const float linkHeight = 0.2f * s;
    const float linkGap = 0.04f * s;                      // Total gap between neighbouring link boxes; prevents overlap on the arcs at spawn.
    const float linkMass = 3.0f * massScale;
    const float trackRadius = wheelRadius + linkHeight * 0.5f + 0.02f * s; // Link centerline radius around the end wheels.
    const float jointStiffness = 3.0e6f * stiffScale;
    const float jointDamping = 1.0e3f * dampScale;

    // Suspension
    const float suspensionDrop = 1.2f * s;                // Hull bottom edge to wheel center. Keeps the upper track run clear of the hull.
    const float suspensionK = 25000.0f * stiffScale;
    const float suspensionD = 900.0f * dampScale;
    const float suspensionTravel = 0.3f * s;              // Max +/- from rest length (hull still clears the upper run when fully compressed).
    const float sliderStiffness = 1.0e6f * stiffScale;
    const float sliderDamping = 1.0e3f * dampScale;

    // Spring rest length exceeds the spawn length so the road wheel springs carry the hull weight
    // (the rigidly mounted sprocket/idler then barely load the ground). Hull settles ~5 mm higher.
    const float suspensionPreload = 0.03f * s;

    // Sprocket (rear wheel): fixed axle on the hull.
    const float axleStiffness = 3.0e6f * stiffScale;
    const float axleDamping = 1.0e3f * dampScale;

    // Drive: angular motor on the sprocket. Forward is +x, which needs negative (clockwise) spin.
    const float driveSpeed = 6.0f * std::sqrt(s);          // Track speed, m/s.
    const float driveTorque = 3000.0f * s * s * s;         // Force ~ s^2, radius ~ s.

    // Idler (front wheel): spring-loaded tensioner sliding along hull X. The spring is pre-compressed,
    // so it always pushes the idler forward and takes up any track slack.
    const float tensionerLen = 0.4f * s;                  // Anchor-to-idler distance at spawn.
    const float tensionerTravel = 0.15f * s;
    const float tensionerPreload = 0.08f * s;
    const float tensionerK = 30000.0f * stiffScale;
    const float tensionerD = 600.0f * dampScale;

    // Local frame: origin at hull center. Wheels sit at localY = -(hullHeight / 2 + suspensionDrop).
    const float wheelLocalY = -(hullHeight * 0.5f + suspensionDrop);

    // === MATERIALS ===
    // Elasticity 0 everywhere: nothing should bounce on a tank.
    const PS_AGONY::MaterialIndex linkMat = simulation.createMaterial({       // Rubber/cleats: grips ground and wheels.
        .elasticity = 0.0f, .staticFriction = 1.2f, .dynamicFriction = 1.0f });
    const PS_AGONY::MaterialIndex wheelMat = simulation.createMaterial({      // Grips the track so it can drive it.
        .elasticity = 0.0f, .staticFriction = 0.8f, .dynamicFriction = 0.7f });
    const PS_AGONY::MaterialIndex hullMat = simulation.createMaterial({       // Slippery: the track slides under it if they touch.
        .elasticity = 0.0f, .staticFriction = 0.05f, .dynamicFriction = 0.03f });

    // === HULL-LOCAL -> WORLD ===
    const float cosR = std::cos(rotation);
    const float sinR = std::sin(rotation);
    auto toWorld = [&](float lx, float ly) -> PS_AGONY::Vec2 {
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
    const PS_AGONY::ObjectIndex hull = *hullOpt;

    // === WHEELS + SUSPENSION ===
    const float firstWheelX = -0.5f * wheelSpacing * static_cast<float>(wheelCount - 1);
    for (int i = 0; i < wheelCount; ++i)
    {
        const float localX = firstWheelX + wheelSpacing * static_cast<float>(i);

        auto wheelOpt = simulation.createCircle({
            .base.position = toWorld(localX, wheelLocalY),
            .base.rotation = rotation,
            .base.mass = wheelMass,
            .base.materialIndex = wheelMat,
            .radius = wheelRadius
            });
        if (!wheelOpt) return;

        // Sprocket: fixed axle, only free to spin.
        if (i == 0)
        {
            simulation.createRevoluteJoint({
                .bodyIndexA = hull,
                .bodyIndexB = *wheelOpt,
                .localAnchorA = { localX, wheelLocalY },
                .localAnchorB = { 0.0f, 0.0f },
                .stiffness = axleStiffness,
                .damping = axleDamping
                });
            simulation.createMotor({
                .bodyIndex = *wheelOpt,
                .targetSpeed = -driveSpeed / trackRadius,
                .maxTorque = driveTorque
                });
            continue;
        }

        // Idler: horizontal slider + pre-compressed spring (track tensioner).
        if (i == wheelCount - 1)
        {
            const float tensionerX = localX - tensionerLen;
            simulation.createSpring({
                .bodyIndexA = hull,
                .bodyIndexB = *wheelOpt,
                .localAnchorA = { tensionerX, wheelLocalY },
                .localAnchorB = { 0.0f, 0.0f },
                .restLength = tensionerLen + tensionerPreload,
                .stiffness = tensionerK,
                .damping = tensionerD
                });
            simulation.createSliderJoint({
                .bodyIndexA = hull,
                .bodyIndexB = *wheelOpt,
                .localAnchorA = { tensionerX, wheelLocalY },
                .localAnchorB = { 0.0f, 0.0f },
                .localAxisA = { 1.0f, 0.0f },
                .stiffness = sliderStiffness,
                .damping = sliderDamping,
                .lowerLimit = tensionerLen - tensionerTravel,
                .upperLimit = tensionerLen + tensionerTravel,
                .enableLimit = true
                });
            continue;
        }

        // Road wheels: independent vertical suspension.
        const float anchorY = -hullHeight * 0.5f;

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

    // === TRACK PATH (hull-local) ===
    // Closed stadium around the first and last wheel, counter-clockwise starting on the bottom run:
    // bottom run (+x) -> right arc -> top run (-x) -> left arc.
    const float xL = firstWheelX;
    const float xR = -firstWheelX;
    const float straight = xR - xL;
    const float arcLen = PI * trackRadius;
    const float perimeter = 2.0f * straight + 2.0f * arcLen;

    auto pathPoint = [&](float d) -> PS_AGONY::Vec2
        {
            if (d < straight)
                return { xL + d, wheelLocalY - trackRadius };
            d -= straight;
            if (d < arcLen)
            {
                const float a = -0.5f * PI + d / trackRadius;
                return { xR + trackRadius * std::cos(a), wheelLocalY + trackRadius * std::sin(a) };
            }
            d -= arcLen;
            if (d < straight)
                return { xR - d, wheelLocalY + trackRadius };
            d -= straight;
            const float a = 0.5f * PI + d / trackRadius;
            return { xL + trackRadius * std::cos(a), wheelLocalY + trackRadius * std::sin(a) };
        };

    // Pivot i is the joint point between link i-1 and link i; link i spans pivot i -> pivot i+1.
    std::vector<PS_AGONY::Vec2> pivots(linkCount);
    for (int i = 0; i < linkCount; ++i)
        pivots[i] = pathPoint(perimeter * static_cast<float>(i) / static_cast<float>(linkCount));

    // === TRACK LINKS ===
    std::vector<PS_AGONY::ObjectIndex> links(linkCount);
    std::vector<float> halfChord(linkCount);
    for (int i = 0; i < linkCount; ++i)
    {
        const PS_AGONY::Vec2 a = pivots[i];
        const PS_AGONY::Vec2 b = pivots[(i + 1) % linkCount];
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
void load_Tank(PS_AGONY::Simulation& simulation)
{
    // === CONFIGURATION ===
    constexpr float groundWidth = 400.0f;
    constexpr float groundThickness = 4.0f;

    constexpr float tankScale = 2.0f;
    constexpr float carScale = 1.0f;
    constexpr float tankX = 0.0f;
    constexpr float carX = 16.0f;       // Tank front reaches ~8.6, car rear wheel edge ~12.9.

    // Obstacles. x = center. Trenches must be sorted by x and must not overlap.
    // Car chassis is 6 long, so any trench > 6 wide can't be bridged by it.
    // Tank track contact spans ~14.4, so it bridges up to roughly that.
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
    const PS_AGONY::MaterialIndex groundMat = simulation.createMaterial({
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

void loadScene(PS_AGONY::Simulation& simulation, int scene)
{
    Ecstasy::Core::Random::Generator rvg; // Random value generator.
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
        load_LargeWorld(simulation, rvg, {}, 2000);

        //load_LargeWorld(simulation, rvg, { -70.0f, -50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, {  70.0f, -50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, { -70.0f,  50.0f }, 2500 / 4);
        //load_LargeWorld(simulation, rvg, {  70.0f,  50.0f }, 2500 / 4);
    }
    else if (scene == 3)
    {
        load_StackedPyramid(simulation, rvg);
    }
    else if (scene == 4)
    {
        load_ConstraintBridges(simulation, rvg);
    }
    else if (scene == 5)
    {
        load_SoftBodyComparisonTest(simulation, rvg);
    }
    else if (scene == 6)
    {
        load_CarRamp(simulation, rvg);
    }
    else if (scene == 7)
    {
        load_Test(simulation, rvg);
    }
    else if (scene == 8)
    {
        load_JointedCircles(simulation, rvg);
    }
    else if (scene == 9)
    {
        load_PrismaticJointGallery(simulation, rvg);
    }
    else if (scene == 10)
    {
        load_Tank(simulation);
    }
}