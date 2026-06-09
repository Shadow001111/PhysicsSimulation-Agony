#include "Scenes.h"
#include "Core/Random.h"

#include "Physics/Simulation.h"

#include <cmath>

void load_ALotOfCollisions(PS_AGONY::Simulation& simulation)
{
    constexpr float boundary = 12.0f;
    constexpr float thickness = 20.0f;

    constexpr int circleCount = 10;
    constexpr int boxCount = 10;

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

    {

        constexpr float halfThickness = thickness * 0.5f;
        constexpr float length = boundary * 2.0f + 2.0f;

        simulation.createBox({ -(boundary + halfThickness),  0.0 }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { thickness, length });
        simulation.createBox({ (boundary + halfThickness),  0.0 }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { thickness, length });
        simulation.createBox({ 0.0, -(boundary + halfThickness) }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { length, thickness });
    }
    for (int i = 0; i < circleCount; i++)
    {
        const float x = Random::real<float>(-5.0f, 5.0f);
        const float y = Random::real<float>(-5.0f, 5.0f);
        const float vx = Random::real<float>(-2.0f, 2.0f);
        const float vy = Random::real<float>(-2.0f, 2.0f);
        const float r = Random::real<float>(0.1f, 0.2f);
        const float mass = 3.14f * r * r;

        simulation.createCircle({ x, y }, { vx, vy }, 0.0f, 0.0f, mass, { 0.0f, 0.0f }, material0Index, r);
    }
    for (int i = 0; i < boxCount; i++)
    {
        const float x = Random::real<float>(-5.0f, 5.0f);
        const float y = Random::real<float>(-5.0f, 5.0f);
        const float vx = Random::real<float>(-2.0f, 2.0f);
        const float vy = Random::real<float>(-2.0f, 2.0f);
        const float rotation = Random::real<float>(0.0f, 6.28f);
        const float width = Random::real<float>(0.2f, 0.4f);
        const float height = Random::real<float>(0.2f, 0.4f);
        const float mass = width * height;

        simulation.createBox({ x, y }, { vx, vy }, rotation, 0.0f, mass, { 0.0f, 0.0f }, material0Index, { width, height });
    }

    {
        constexpr float radius = 4.0f;
        simulation.createCircle({ 0.0f, boundary + thickness + radius + 0.5f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 20000.0f, { 0.0f, 0.0f }, material1Index, 4.0f, 1);
    }
}

void load_CleanPerfomanceOfContactsTest(PS_AGONY::Simulation& simulation)
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

    simulation.createBox({ 0.0f,  -floorThickness * 0.5f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, { 0.0f, 0.0f }, material0Index, { floorWidth, floorThickness });
    simulation.createBox({ 0.0f,  -floorThickness * 0.5f + paddingY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, { 0.0f, 0.0f }, material0Index, { floorWidth, floorThickness });

    constexpr float dx = boxSize + boxPadding;
    for (int i = 0; i < objectCount; i++)
    {
        const float x = (-floorWidth + boxSize + boxPadding) * 0.5f + i * dx;

        simulation.createBox({ x, boxSize * 0.5f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, { 0.0f, 0.0f }, material0Index, { boxSize, boxSize });
        simulation.createCircle({ x, boxSize * 0.5f + paddingY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, { 0.0f, 0.0f }, material0Index, boxSize * 0.5f);
    }
}

void load_GaltonBoard(PS_AGONY::Simulation& simulation)
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

    constexpr float funnelOpeningWidth = ballRadius * 4.0f * 1.5f;
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

            simulation.createCircle({ pegX, pegY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, { 0.0f, 0.0f }, material0Index, pegRadius);
        }
    }

    // Walls.
    {
        constexpr float wallInnerBoundaryX = pegXBoundary + pegSpacing - pegRadius;
        constexpr float wallInnerBottomY = pegBottomY - binHeight;

        simulation.createBox(
            { 0.0f, wallInnerBottomY - wallThickness * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            { 0.0f, 0.0f }, material0Index,
            { wallInnerBoundaryX * 2.0f, wallThickness }
        );

        simulation.createBox(
            { -(wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            { 0.0f, 0.0f }, material0Index,
            { wallThickness, wallHeight }
        );

        simulation.createBox(
            { (wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            { 0.0f, 0.0f }, material0Index,
            { wallThickness, wallHeight }
        );
    }

    // Bins.
    {
        for (int x = 0; x < pegColumns; x++)
        {
            const float binX = -pegXBoundary + pegRadius + pegSpacing * x;

            simulation.createBox(
                { binX, pegBottomY - binHeight * 0.5f },
                { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
                { 0.0f, 0.0f }, material0Index,
                { binWidth, binHeight }
            );
        }
    }

    // Funnel.
    {
        const float funnelX = funnelOpeningWidth * 0.5f + std::sin(funnelRotation) * funnelLength * 0.5f;
        const float funnelY = funnelYOffset + std::cos(funnelRotation) * funnelLength * 0.5f;

        simulation.createBox(
            { -funnelX, funnelY },
            { 0.0f, 0.0f }, funnelRotation, 0.0f, 0.0f,
            { 0.0f, 0.0f }, material0Index,
            { funnelWidth, funnelLength }
        );

        simulation.createBox(
            { funnelX, funnelY },
            { 0.0f, 0.0f }, -funnelRotation, 0.0f, 0.0f,
            { 0.0f, 0.0f }, material0Index,
            { funnelWidth, funnelLength }
        );
    }

    // Balls.
    {
        constexpr float spawnBottom = 0.1f + ballRadius + funnelYOffset;

        const float rotTan = std::tan(funnelRotation);
        const float rndStart = ballRadius * 5.0f;
        const float rndEnd = std::cos(funnelRotation) * funnelLength - ballRadius;

        for (int i = 0; i < ballCount; i++)
        {
            float rnd = Random::real<float>(0.0f, 1.0f);
            rnd = 1.0f - rnd;
            rnd *= rnd;
            rnd = 1.0f - rnd;
            rnd = rndStart + rnd * (rndEnd - rndStart);

            const float xBorder = funnelOpeningWidth * 0.5f + rnd * rotTan;

            const float x = Random::real<float>(-xBorder, xBorder);
            const float y = spawnBottom + rnd;
            simulation.createCircle({ x, y }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, { 0.0f, 0.0f }, material0Index, ballRadius);
        }
    }
}

void load_Planet(PS_AGONY::Simulation& simulation)
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

    simulation.createCircle({ 0.0f, 0.0f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, { 0.0f, 0.0f }, material0Index, planetRadius);

    for (int i = 0; i < circleCount; i++)
    {
        const float angle = Random::real<float>(0.0f, 6.28f);
        const float r = Random::real<float>(0.1f, 1.0f);
        const float mass = 3.14f * r * r;

        const float dist = planetRadius + r;
        const float x = std::cos(angle) * dist;
        const float y = std::sin(angle) * dist;

        simulation.createCircle({ x, y }, { 0.0f, 0.0f }, 0.0f, 0.0f, mass, { 0.0f, 0.0f }, material0Index, r);
    }

    {
        constexpr float radius = 4.0f;
        simulation.createCircle({ 0.0f, planetRadius + radius }, { 0.0f, 0.0f }, 0.0f, 0.0f, 20000.0f, { 0.0f, 0.0f }, material1Index, radius, 1);
    }
}

void load_BalancerSwing(PS_AGONY::Simulation& simulation)
{
    constexpr float balancerWidth = 30.0f;
    constexpr float balancerThickness = 3.0f;
    constexpr float balancerAtitude = 10.0f;
    constexpr float balancerMass = 100000.0f;

    PS_AGONY::Material material0 = {
            .elasticity = 0.0,
            .staticFriction = 1000.0,
            .dynamicFriction = 1000.0
    };

    PS_AGONY::Material material1 = {
        .elasticity = 0.5,
        .staticFriction = 1.0,
        .dynamicFriction = 1.0
    };

    PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);
    PS_AGONY::MaterialIndex material1Index = simulation.createMaterial(material1);

    {

        constexpr float thickness = 2.0f;
        constexpr float length = 100.0f;

        simulation.createBox({ 0.0, thickness * -0.5f }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { length, thickness });
    }
    {
        const float size = (balancerAtitude * balancerAtitude * 0.2f) / std::sqrt(2);

        simulation.createBox({ 0.0, 0.0 }, { 0.0, 0.0 }, PS_AGONY::Constants::PI * 0.25f, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { size, size });
    }
    {
        simulation.createBox({ 0.0, balancerAtitude + balancerThickness * 0.5f }, { 0.0, 0.0 }, 0.0, 0.0, balancerMass, { 0.0f, 0.0f }, material0Index, { balancerWidth, balancerThickness });
        
        //constexpr float littleWidth = 0.1f;
        //constexpr float littleThickness = 0.1f;
        //simulation.createBox({ 0.0, balancerAtitude - littleThickness  * 0.5f}, { 0.0, 0.0 }, 0.0, 0.0, 0.0, { 0.0f, 0.0f }, material0Index, { littleWidth, littleThickness });
    }
}

void loadScene(PS_AGONY::Simulation& simulation, int scene)
{
	if (scene == 0)
	{
		load_ALotOfCollisions(simulation);
	}
	else if (scene == 1)
	{
		load_CleanPerfomanceOfContactsTest(simulation);
	}
	else if (scene == 2)
	{
		load_GaltonBoard(simulation);
	}
	else if (scene == 3)
	{
		load_Planet(simulation);
	}
    else if (scene == 4)
    {
        load_BalancerSwing(simulation);
    }
}
