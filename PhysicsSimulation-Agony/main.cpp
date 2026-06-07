#include "WindowManager.h"

#include "Core/FileLogger.h"
#include "Core/TracyProfiler.h"
#include "Core/Random.h"

#include "Physics/Simulation.h"
#include "Physics/SimulationRenderer.h"

#include "Graphics/TextRenderer.h"

#include <iostream>
#include <iomanip>
#include <memory>


static std::string formatSize(size_t value)
{
    static const char* suffixes[] = { "", "k", "M", "G", "T", "P", "E" };
    constexpr size_t suffixCount = sizeof(suffixes) / sizeof(suffixes[0]);
    double scaled = static_cast<double>(value);
    size_t suffixIndex = 0;

    while (scaled >= 1000.0 && suffixIndex < suffixCount - 1)
    {
        scaled /= 1000.0;
        ++suffixIndex;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision((scaled < 10.0 && suffixIndex > 0) ? 1 : 0);
    oss << scaled << suffixes[suffixIndex];
    return oss.str();
}

static std::string formatSizeBinary(size_t value)
{
    static const char* suffixes[] = { "B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB" };
    constexpr size_t suffixCount = sizeof(suffixes) / sizeof(suffixes[0]);
    double scaled = static_cast<double>(value);
    size_t suffixIndex = 0;

    while (scaled >= 1024.0 && suffixIndex < suffixCount - 1)
    {
        scaled /= 1024.0;
        ++suffixIndex;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision((scaled < 10.0 && suffixIndex > 0) ? 1 : 0);
    oss << scaled << suffixes[suffixIndex];
    return oss.str();
}


struct DebugData
{
    float smoothedDelta = 0.0f; // Seconds.
    float deltaTime = 0.0f; // Seconds.

    PS_AGONY::Simulation::DebugData simulationDebugData;
};

static void renderDebugText(float aspectRatio, const DebugData& debugData)
{
    constexpr float rowHeight = 0.06f;
    constexpr float sideOffset = 0.014f;

    // References.
    const auto& simulationData = debugData.simulationDebugData;

    // Push data on stream.

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);

    // Real-time counters?
    {
        const float smoothedDelta = debugData.smoothedDelta;

        const float FPS = smoothedDelta > 0.0f
            ? 1.0f / smoothedDelta
            : 0.0f;
        ss << "FPS: " << FPS << " (" << smoothedDelta * 1000.0f << " ms)";

        float upsPercent = (float)simulationData.updatesHappened / (float)simulationData.updatesSupposedToHappen;
        upsPercent = std::min(upsPercent, 1.0f);
        ss << "\nUPS: " << simulationData.updatesHappened << " / " << simulationData.updatesSupposedToHappen
            << " (" << upsPercent * 100.0f << "%)";
    }

    // Memory.
    {
        const size_t total =
            simulationData.bodyDataMemoryUsage +
            simulationData.circleDataMemoryUsage +
            simulationData.boxDataMemoryUsage +
            simulationData.broadPhaseDetectorMemoryUsage +
            simulationData.narrowPhaseDetectorMemoryUsage;

        ss << "\nMemory: " << formatSizeBinary(total);
        ss << "\n  Bodies data: " << formatSizeBinary(simulationData.bodyDataMemoryUsage);
        ss << "\n  Circles data: " << formatSizeBinary(simulationData.circleDataMemoryUsage);
        ss << "\n  Boxes data: " << formatSizeBinary(simulationData.boxDataMemoryUsage);
        ss << "\n  Broad collision detector: " << formatSizeBinary(simulationData.broadPhaseDetectorMemoryUsage);
        ss << "\n  Narrow collision detector: " << formatSizeBinary(simulationData.narrowPhaseDetectorMemoryUsage);
    }

    // Convert stream to string.
    const std::string text = ss.str();

    // Prepare.
    TextRenderer::setCustomCoordinateSpace(-aspectRatio, aspectRatio, -1.0f, 1.0f);
    TextRenderer::startTextRendering();

    // Render
    TextRenderer::renderText(text, -aspectRatio + sideOffset, 1.0f - sideOffset, rowHeight, glm::vec3(1.0f, 0.0f, 0.0f));
}

void setupGaltonBoard(PS_AGONY::Simulation& simulation)
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

            simulation.createCircle({ pegX, pegY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, material0Index, pegRadius);
        }
    }

    // Walls.
    {
        constexpr float wallInnerBoundaryX = pegXBoundary + pegSpacing - pegRadius;
        constexpr float wallInnerBottomY = pegBottomY - binHeight;

        simulation.createBox(
            { 0.0f, wallInnerBottomY - wallThickness * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            material0Index,
            { wallInnerBoundaryX * 2.0f, wallThickness}
        );

        simulation.createBox(
            { -(wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            material0Index,
            { wallThickness, wallHeight }
        );

        simulation.createBox(
            { (wallInnerBoundaryX + wallThickness * 0.5f), wallInnerBottomY + wallHeight * 0.5f },
            { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f,
            material0Index,
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
                material0Index,
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
            material0Index,
            { funnelWidth, funnelLength }
        );

        simulation.createBox(
            { funnelX, funnelY },
            { 0.0f, 0.0f }, -funnelRotation, 0.0f, 0.0f,
            material0Index,
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
            simulation.createCircle({ x, y }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, material0Index, ballRadius);
        }
    }
}

static int gameFunc()
{
    // Window.
    WindowManager wnd({
        .width = 1600,
        .height = 900,
        .title = "Physics simulation - AGONY",
        .nativeFullscreen = false,
        .bolderlessFullscreen = false,
        .resizable = true,
        .vsync = true,
        .openglDebug = true,
        .strictAspectRatio = false
        });

    InputManager windowInputManager;
    wnd.linkInputManager(&windowInputManager);

    // Textures.
    {
        Texture::initGlobalData();
    }

    // Create framebuffer.
    FrameBuffer framebuffer;
    {
        TRACY_SCOPE_N("Create framebuffer");

        framebuffer.create(wnd.getWidth(), wnd.getHeight());

        const Texture::Parameters params
        {
            .minFilter = GL_NEAREST,
            .magFilter = GL_NEAREST,
            .wrapS = GL_CLAMP_TO_EDGE,
            .wrapT = GL_CLAMP_TO_EDGE
        };
        const bool bindless = Texture::getExtensions().bindless;

        framebuffer.createColorAttachment("color", GL_RGBA8, params, bindless);

        framebuffer.createDepthAttachment("depth", GL_DEPTH_COMPONENT32F, params, bindless);

        if (!framebuffer.isComplete())
        {
            std::cerr << "[main]: Failed to create framebuffer.\n";
            return 1;
        }

        wnd.linkFramebuffer(&framebuffer);

        framebuffer.bind();
    }

    // Text renderer
    TextRenderer::init();
    TextRenderer::loadFont("RusEngMinecraft", 8);
    TextRenderer::setCurrentFont("RusEngMinecraft");
    TextRenderer::setGlyphInstanceBatchSize(1024);

    // Input settings.
    //glfwSetInputMode(wnd.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Simulation
    PS_AGONY::Simulation simulation{};

    auto& mainBodyHolder = simulation.getMainBodyHolder();

    {
        PS_AGONY::Material material0 = {
            .elasticity = 0.9,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
        };

        PS_AGONY::Material material1 = {
            .elasticity = 0.0,
            .staticFriction = 1.0,
            .dynamicFriction = 1.0
        };

        PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);
        PS_AGONY::MaterialIndex material1Index = simulation.createMaterial(material1);

        Random::setSeed(0);

        if constexpr (false)
        {
            constexpr float boundary = 12.0f;
            constexpr float thickness = 20.0f;
            constexpr float centerBoxSize = boundary * 0.5f;
            {

                constexpr float halfThickness = thickness * 0.5f;
                constexpr float length = boundary * 2.0f + 2.0f;

                simulation.createBox({ -(boundary + halfThickness),  0.0 }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, material0Index, { thickness, length });
                simulation.createBox({ (boundary + halfThickness),  0.0 }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, material0Index, { thickness, length });
                simulation.createBox({ 0.0, -(boundary + halfThickness) }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, material0Index, { length, thickness });
                //simulation.createBox({ 0.0,  (boundary + halfThickness) }, { 0.0, 0.0 }, 0.0, 0.0, 0.0, material0Index, { length, thickness });

                //simulation.createBox({ 0.0, 0.0 }, { 0.0, 0.0 }, 3.14 * 0.25, 0.0, 0.0, material0Index, { centerBoxSize, centerBoxSize });
            }

            const int circleCount = 500;
            const int boxCount = 500;
            for (int i = 0; i < circleCount; i++)
            {
                const float x = Random::real<float>(-5.0f, 5.0f);
                const float y = Random::real<float>(-5.0f, 5.0f);
                const float vx = Random::real<float>(-2.0f, 2.0f);
                const float vy = Random::real<float>(-2.0f, 2.0f);
                const float r = Random::real<float>(0.1f, 0.2f);
                const float mass = 3.14f * r * r;

                simulation.createCircle({ x, y }, { vx, vy }, 0.0f, 0.0f, mass, material0Index, r);
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

                simulation.createBox({ x, y }, { vx, vy }, rotation, 0.0f, mass, material0Index, { width, height });
            }

            {
                constexpr float radius = 4.0f;
                //simulation.createCircle({ 0.0f, boundary + thickness + radius + 0.5f }, { 0.0f, 0.0f }, 0.0f, 1.0f, 20000.0f, material1Index, radius);
            }
        }
        else if constexpr (true)
        {
            constexpr int objectCount = 500;
            constexpr float boxSize = 0.1f;
            constexpr float boxPadding = 0.03f;

            constexpr float paddingY = 5.0f;

            constexpr float floorThickness = 1.0f;
            constexpr float floorWidth = objectCount * (boxSize + boxPadding);

            simulation.createBox({ 0.0f,  -floorThickness * 0.5f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, material0Index, { floorWidth, floorThickness });
            simulation.createBox({ 0.0f,  -floorThickness * 0.5f + paddingY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 0.0f, material0Index, { floorWidth, floorThickness });

            constexpr float dx = boxSize + boxPadding;
            for (int i = 0; i < objectCount; i++)
            {
                const float x = (-floorWidth + boxSize + boxPadding) * 0.5f + i * dx;

                simulation.createBox({ x, boxSize * 0.5f }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, material0Index, { boxSize, boxSize });
                simulation.createCircle({ x, boxSize * 0.5f + paddingY }, { 0.0f, 0.0f }, 0.0f, 0.0f, 1.0f, material0Index, boxSize * 0.5f);
            }
        }
        else
        {
            setupGaltonBoard(simulation);
        }
    }

    PS_AGONY::SimulationRenderer simulationRenderer;
    simulationRenderer.init();

    auto& camera = simulationRenderer.getCamera();
	camera.setViewRangeH(20.0f, wnd.getAspectRatio());

    // OpenGL states.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    // Debug data.
    DebugData debugData;

    // Main loop.
    double lastTime = glfwGetTime();
    while (!wnd.shouldClose())
    {
        // Poll events.
        wnd.pollEvents();
        windowInputManager.processInput();

        // Check if window is visible.
        const bool iconified = wnd.isZeroSize();

        // Time logic.
        const double time = glfwGetTime();
        const double deltaTime = time - lastTime;
        lastTime = time;

        debugData.deltaTime = deltaTime;

		// Handle input.
        {
            const PS_AGONY::Real cameraSpeed = 1.0 * deltaTime * camera.viewRange;
			const PS_AGONY::Real zoomSpeed = 2.0 * deltaTime;

            if (wnd.isKeyPressed(GLFW_KEY_W))
                camera.position.y += cameraSpeed;
            if (wnd.isKeyPressed(GLFW_KEY_S))
                camera.position.y -= cameraSpeed;
            if (wnd.isKeyPressed(GLFW_KEY_A))
                camera.position.x -= cameraSpeed;
            if (wnd.isKeyPressed(GLFW_KEY_D))
                camera.position.x += cameraSpeed;
            if (wnd.isKeyPressed(GLFW_KEY_Q))
                camera.setViewRangeH(camera.viewRange * (1.0 + zoomSpeed), wnd.getAspectRatio());
            if (wnd.isKeyPressed(GLFW_KEY_E))
				camera.setViewRangeH(camera.viewRange / (1.0 + zoomSpeed), wnd.getAspectRatio());
        }
        {
            PS_AGONY::Vec2 mouseWorldPosition;
            PS_AGONY::Vec2 mouseWorldVelocity;
            {
                const glm::dvec2 mousePosition = windowInputManager.getMousePosition();
                const glm::dvec2 mouseNextPosition = mousePosition + windowInputManager.getMouseDelta();

                const glm::dvec2 wndSize = { wnd.getWidth(), wnd.getHeight() };

                glm::dvec2 positionNDC = (mousePosition / wndSize) * 2.0 - 1.0;
                positionNDC.y = -positionNDC.y;

                glm::dvec2 nextPositionNDC = (mouseNextPosition / wndSize) * 2.0 - 1.0;
                nextPositionNDC.y = -nextPositionNDC.y;

                mouseWorldPosition = camera.screenToWorldSpace(positionNDC);
                PS_AGONY::Vec2 mouseWorldNextPosition = camera.screenToWorldSpace(nextPositionNDC);

                mouseWorldVelocity = mouseWorldNextPosition - mouseWorldPosition;
            }

            mainBodyHolder.holderPosition = mouseWorldPosition;
            mainBodyHolder.holderVelocity = mouseWorldVelocity / PS_AGONY::Real(deltaTime);

            if (windowInputManager.isMouseButtonJustReleased(0))
            {
                simulation.mainBodyHolderRelease();
            }
            else if (windowInputManager.isMouseButtonJustPressed(0))
            {
                simulation.mainBodyHolderGrabAt(mouseWorldPosition);
            }

            if (mainBodyHolder.heldBody.has_value())
            {
                if (windowInputManager.isMouseButtonPressed(1))
                {
                    simulation.mainBodyHolderIncreaseAngularVelocity(-30.0 * deltaTime);
                }
            }
        }

        // Simulation.
        simulation.update(deltaTime);
        debugData.simulationDebugData = simulation.getDebugData();

        // Smooth debug data.
        {
            constexpr float targetAlpha = 0.5f;

            float oneMinusAlpha = pow(1.0 - targetAlpha, deltaTime * 60.0);
            float alpha = 1.0 - oneMinusAlpha;
            debugData.smoothedDelta = alpha * debugData.deltaTime + oneMinusAlpha * debugData.smoothedDelta;
        }

        // Render.
        if (iconified)
        {
            // Force app to 20 fps. Stop rendering and swapping buffers.
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 20));
        }
        else if (framebuffer.isComplete()) [[likely]]
        {
            framebuffer.setDrawBuffers({ "color" });

            // Clean screen.
            const float black[4] = { 0.0f, 0.0f ,0.0f ,0.0f };
            framebuffer.clearDrawBuffer("color", black);

            // Render simulation.
            simulationRenderer.render(simulation);

            // Render debug data.
            renderDebugText(wnd.getAspectRatio(), debugData);

            // Blitting FBO to default FBO.
            framebuffer.setReadBuffer("color");
            framebuffer.blitToDefaultFramebuffer(wnd.getWidth(), wnd.getHeight());

            // Swap buffers.
            wnd.swapBuffers();
        }
        else [[unlikely]]
        {
            std::cerr << "[main]: FBO is not complete!\n";
        }
    }

    return 0;
}

int main()
{
    int result = 0;

    result = gameFunc();

    /*try
    {
        result = gameFunc();
    }
    catch (const std::exception& e)
    {
        FileLogger logger("log/crash.txt");

        std::string message = "EXCEPTION: " + std::string(e.what());

        logger.add(message);
        result = -1;
    }*/

    return result;
}