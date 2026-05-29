#include "WindowManager.h"

#include "Core/FileLogger.h"
#include "Core/TracyProfiler.h"
#include "Core/UpdateTimer.h"

#include "AudioEngine/Player.h"

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
        .resizable = false,
        .vsync = true,
        .openglDebug = true,
        .strictAspectRatio = true
        });

    InputManager windowInputManager;
    wnd.linkInputManager(&windowInputManager);

    // Init audio engine.
    {
        TRACY_SCOPE_N("Init audio engine");
        AudioEngine::Player::getGlobalInstance().init();
    }

    // Init texture global data.
    {
        TRACY_SCOPE_N("Init texture global data");
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

    // Input settings.
    glfwSetInputMode(wnd.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Simulation
    PS_AGONY::Simulation simulation;

    PS_AGONY::SimulationRenderer simulationRenderer;
    simulationRenderer.init();

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

            // Blitting FBO to default FBO.
            framebuffer.setReadBuffer("color");
            framebuffer.blitToDefaultFramebuffer(wnd.getWidth(), wnd.getHeight());

            // Render simulation.
            simulationRenderer.render(simulation);

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