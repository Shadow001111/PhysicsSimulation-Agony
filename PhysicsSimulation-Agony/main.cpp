#include "WindowManager.h"

#include "Core/FileLogger.h"
#include "Core/TracyProfiler.h"
#include "Core/Random.h"

#include "Physics/Simulation.h"
#include "Physics/SimulationRenderer.h"

#include <iostream>

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

    // Input settings.
    //glfwSetInputMode(wnd.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Simulation
    PS_AGONY::Simulation simulation{};

    {
        PS_AGONY::Material material0 = {
            .elasticity = 0.9,
            .staticFriction = 0.2,
            .dynamicFriction = 0.2
        };

        PS_AGONY::MaterialIndex material0Index = simulation.createMaterial(material0);

        {
            constexpr float boundary = 9.0f;
            constexpr float radius = 100.0f;

            simulation.createCircle({ -(boundary + radius),  0.0 }, { 0.0, 0.0 }, radius, 0.0, 0.0, 0.0, material0Index);
            simulation.createCircle({  (boundary + radius),  0.0 }, { 0.0, 0.0 }, radius, 0.0, 0.0, 0.0, material0Index);
            simulation.createCircle({  0.0, -(boundary + radius) }, { 0.0, 0.0 }, radius, 0.0, 0.0, 0.0, material0Index);
            simulation.createCircle({  0.0,  (boundary + radius) }, { 0.0, 0.0 }, radius, 0.0, 0.0, 0.0, material0Index);
        }

        const int bodyCount = 1'406 - 4 - 1;// 2'500;
        for (int i = 0; i < bodyCount; i++)
        {
            const float x = Random::real<float>(-5.0f, 5.0f);
            const float y = Random::real<float>(-5.0f, 5.0f);
            const float vx = Random::real<float>(-2.0f, 2.0f);
            const float vy = Random::real<float>(-2.0f, 2.0f);
            const float r = Random::real<float>(0.1f, 0.2f);
            const float mass = r * r;

            simulation.createCircle({ x, y }, { vx, vy }, r, 0.0, 0.0, mass, material0Index);
        }
    }

    PS_AGONY::SimulationRenderer simulationRenderer;
    simulationRenderer.init();

    auto& camera = simulationRenderer.getCamera();
	camera.setViewRangeH(20.0f, wnd.getAspectRatio());

    // OpenGL states.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

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

        // Simulation.
        simulation.update(deltaTime);

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
    int result;

    try
    {
        result = gameFunc();
    }
    catch (const std::exception& e)
    {
        FileLogger logger("log/crash.txt");

        std::string message = "EXCEPTION: " + std::string(e.what());

        logger.add(message);
        result = -1;
    }

    return result;
}