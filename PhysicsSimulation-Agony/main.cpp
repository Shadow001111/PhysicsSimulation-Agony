#include "WindowManager.h"
#include "Scenes.h"

#include "Core/FileLogger.h"
#include "EcstasyCore/TracyProfiler.h"

#include "Physics/Simulation.h"
#include "Physics/SimulationRenderer.h"

#include "EcstasyGraphics/TextRenderer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <iomanip>


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

static void renderDebugData(const DebugData& debugData, bool& pauseSimulation)
{
    ImGui::Begin("Simulation Diagnostics");

    const auto& simulationData = debugData.simulationDebugData;

    // App/Performance Section
    if (ImGui::CollapsingHeader("Application Performance", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const float smoothedDelta = debugData.smoothedDelta;
        const float FPS = smoothedDelta > 0.0f ? 1.0f / smoothedDelta : 0.0f;
        ImGui::Text("FPS: %.1f (%.1f ms)", FPS, smoothedDelta * 1000.0f);

        float upsPercent = (simulationData.updatesSupposedToHappen > 0)
            ? (float)simulationData.updatesHappened / (float)simulationData.updatesSupposedToHappen
            : 0.0f;
        upsPercent = std::fmin(upsPercent, 1.0f);

        ImGui::Text("UPS: %u / %u (%.1f%%)",
            simulationData.updatesHappened,
            simulationData.updatesSupposedToHappen,
            upsPercent * 100.0f);

        ImGui::Separator();
        ImGui::Checkbox("Pause Simulation (P)", &pauseSimulation);
    }

    // Memory Hierarchy Section
    const size_t shapeTotal =
        simulationData.circleDataMemoryUsage +
        simulationData.boxDataMemoryUsage +
        simulationData.polygonDataMemoryUsage;

    const size_t constraintTotal =
        simulationData.springDataMemoryUsage;

    const size_t solvingTotal =
        simulationData.bodyCollisionSolverMemoryUsage +
        simulationData.bodyCollisionPlannerMemoryUsage +
        simulationData.springSolverMemoryUsage +
        simulationData.springPlannerMemoryUsage;

    const size_t totalMemory =
        simulationData.bodyDataMemoryUsage +
        shapeTotal +
        constraintTotal +
        simulationData.broadPhaseDetectorMemoryUsage +
        simulationData.narrowPhaseDetectorMemoryUsage +
        solvingTotal;

    if (ImGui::CollapsingHeader("Memory Metrics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("Total System Footprint: %s", formatSizeBinary(totalMemory).c_str());
        ImGui::Separator();

        ImGui::Text("Bodies: %s", formatSizeBinary(simulationData.bodyDataMemoryUsage).c_str());

        if (ImGui::TreeNode("Shapes"))
        {
            ImGui::Text("Total Shapes: %s", formatSizeBinary(shapeTotal).c_str());
            ImGui::BulletText("Circles: %s", formatSizeBinary(simulationData.circleDataMemoryUsage).c_str());
            ImGui::BulletText("Boxes: %s", formatSizeBinary(simulationData.boxDataMemoryUsage).c_str());
            ImGui::BulletText("Polygons: %s", formatSizeBinary(simulationData.polygonDataMemoryUsage).c_str());
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Constraints"))
        {
            ImGui::Text("Total Constraints: %s", formatSizeBinary(constraintTotal).c_str());
            ImGui::BulletText("Springs: %s", formatSizeBinary(simulationData.springDataMemoryUsage).c_str());
            ImGui::TreePop();
        }

        ImGui::Text("Broad Phase Detector: %s", formatSizeBinary(simulationData.broadPhaseDetectorMemoryUsage).c_str());
        ImGui::Text("Narrow Phase Detector: %s", formatSizeBinary(simulationData.narrowPhaseDetectorMemoryUsage).c_str());

        if (ImGui::TreeNode("Solver Allocations"))
        {
            ImGui::Text("Total Solving: %s", formatSizeBinary(solvingTotal).c_str());
            ImGui::BulletText("Body Collision Solver: %s", formatSizeBinary(simulationData.bodyCollisionSolverMemoryUsage).c_str());
            ImGui::BulletText("Body Collision Planner: %s", formatSizeBinary(simulationData.bodyCollisionPlannerMemoryUsage).c_str());
            ImGui::BulletText("Spring Solver: %s", formatSizeBinary(simulationData.springSolverMemoryUsage).c_str());
            ImGui::BulletText("Spring Planner: %s", formatSizeBinary(simulationData.springPlannerMemoryUsage).c_str());
            ImGui::TreePop();
        }
    }

    ImGui::End();
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

    // ImGui context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(wnd.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // Textures.
    Texture::initGlobalData();

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

    // Simulation
    PS_AGONY::Simulation simulation{};
    bool pauseSimulation = false;

    auto& mainBodyHolder = simulation.getMainBodyHolder();

    loadScene(simulation, 2);

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

        // Start the ImGui frame.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Check if window is visible.
        const bool iconified = wnd.isZeroSize();

        // Time logic.
        const double time = glfwGetTime();
        const double deltaTime = time - lastTime;
        lastTime = time;

        debugData.deltaTime = deltaTime;

		// Handle input.
        if (!io.WantCaptureKeyboard)
        {
            const PS_AGONY::Real cameraSpeed = 1.0 * deltaTime * camera.viewRange;
			const PS_AGONY::Real zoomSpeed = 2.0 * deltaTime;

            if (windowInputManager.isKeyPressed(GLFW_KEY_W))
                camera.position.y += cameraSpeed;
            if (windowInputManager.isKeyPressed(GLFW_KEY_S))
                camera.position.y -= cameraSpeed;
            if (windowInputManager.isKeyPressed(GLFW_KEY_A))
                camera.position.x -= cameraSpeed;
            if (windowInputManager.isKeyPressed(GLFW_KEY_D))
                camera.position.x += cameraSpeed;
            if (windowInputManager.isKeyPressed(GLFW_KEY_Q))
                camera.setViewRangeH(camera.viewRange * (1.0 + zoomSpeed), wnd.getAspectRatio());
            if (windowInputManager.isKeyPressed(GLFW_KEY_E))
				camera.setViewRangeH(camera.viewRange / (1.0 + zoomSpeed), wnd.getAspectRatio());

            pauseSimulation ^= windowInputManager.isKeyJustPressed(GLFW_KEY_P);
        }
        const bool isHoldingBody = mainBodyHolder.heldBody.has_value();
        if (!io.WantCaptureMouse || isHoldingBody)
        {
            PS_AGONY::Vec2 mouseWorldPosition;
            {
                const glm::dvec2 mousePosition = windowInputManager.getMousePosition();

                const glm::dvec2 wndSize = { wnd.getWidth(), wnd.getHeight() };

                glm::dvec2 positionNDC = (mousePosition / wndSize) * 2.0 - 1.0;
                positionNDC.y = -positionNDC.y;

                mouseWorldPosition = camera.screenToWorldSpace(positionNDC);
            }

            mainBodyHolder.setPosition(mouseWorldPosition, deltaTime);

            if (windowInputManager.isMouseButtonJustReleased(0))
            {
                simulation.mainBodyHolderRelease();
            }
            else if (windowInputManager.isMouseButtonJustPressed(0) && !io.WantCaptureMouse)
            {
                simulation.mainBodyHolderGrabAt(mouseWorldPosition);
            }

            if (isHoldingBody)
            {
                if (windowInputManager.isMouseButtonPressed(1))
                {
                    simulation.mainBodyHolderIncreaseAngularVelocity(-30.0 * deltaTime);
                }

                if (windowInputManager.isKeyJustPressed(GLFW_KEY_SPACE))
                {
                    simulation.destroyBody(mainBodyHolder.heldBody.value());
                    mainBodyHolder.heldBody.reset();
                }
            }
        }

        // Simulation.
        if (!pauseSimulation)
        {
            simulation.update(deltaTime);
            debugData.simulationDebugData = simulation.getDebugData();
        }

        // Smooth debug data.
        {
            constexpr float targetAlpha = 0.8f;

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
            const float bgColor[4] = { 0.0f, 0.0f, 0.1f, 0.0f };
            framebuffer.clearDrawBuffer("color", bgColor);

            // Render simulation.
            simulationRenderer.render(simulation);

            // Render debug data.
            renderDebugData(debugData, pauseSimulation);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

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

    // Cleanup ImGui contexts.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

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