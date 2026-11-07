#include "Scenes.h"
#include "EcstasyThreadingBackend.h"

#include "PS_AGONY2D/Simulation.h"

#include "Editor/Editor.h"

#include "Render/SimulationRenderer.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Graphics/WindowManager.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <sstream>
#include <format>

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

    PS_AGONY2D::Simulation::DebugData simulationDebugData;
};

static void renderGUI(
    PS_AGONY2D::Simulation& simulation,
    PS_AGONY_RENDER::SimulationRenderer& simulationRenderer,
    PS_AGONY_RENDER::RenderOptions& renderOptions,
    const DebugData& debugData
)
{
    ImGui::Begin("Simulation Diagnostics");

    const auto& simulationData = debugData.simulationDebugData;

    // App performance.
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
        bool paused = simulation.isPaused();
        if (ImGui::Checkbox("Pause Simulation (P)", &paused))
            simulation.setPause(paused);
    }

    // Simulation settings.
    auto& settings = simulation.getSimulationSettings();
    if (ImGui::CollapsingHeader("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        float timeScale = static_cast<float>(settings.clockSettings.timeScale);
        if (ImGui::SliderFloat("Time scale", &timeScale, 0.0f, 1.0f, "%.3f"))
        {
            settings.clockSettings.timeScale = static_cast<PS_AGONY2D::Real>(timeScale);
        }

        float hz = static_cast<float>(1.0 / settings.clockSettings.updateInterval);
        if (ImGui::SliderFloat("Update Rate (Hz)", &hz, 10.0f, 1000.0f, "%.0f Hz"))
        {
            settings.clockSettings.updateInterval = static_cast<PS_AGONY2D::Real>(1.0f / hz);
        }

        int velIter = static_cast<int>(settings.collisionVelocitySolvingIterations);
        if (ImGui::SliderInt("Velocity Iterations", &velIter, 1, 50))
        {
            settings.collisionVelocitySolvingIterations = static_cast<uint32_t>(velIter);
        }

        int posIter = static_cast<int>(settings.collisionPositionSolvingIterations);
        if (ImGui::SliderInt("Position Iterations", &posIter, 1, 50))
        {
            settings.collisionPositionSolvingIterations = static_cast<uint32_t>(posIter);
        }

        for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
        {
            auto& system = simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i));
            int iters = int(system.iterations);

            const std::string label = std::string(system.getName()) + " Iterations";
            if (ImGui::SliderInt(label.c_str(), &iters, 1, 50))
            {
                system.iterations = static_cast<uint32_t>(iters);
            }
        }

        ImGui::Checkbox("Collision Events", &settings.collisionEventsEnabled);

        auto integratorType = settings.integratorSettings.integratorType;
        const char* previewValue = PS_AGONY2D::Integrator::integratorTypeToString(integratorType);
        if (ImGui::BeginCombo("Integrator type", previewValue))
        {
            // Loop through all values of the enum.
            for (int i = 0; i < static_cast<int>(PS_AGONY2D::Integrator::IntegratorType::COUNT); i++)
            {
                auto enumValue = PS_AGONY2D::Integrator::IntegratorType(i);
                bool isSelected = (integratorType == enumValue);

                // Draw the selectable item.
                if (ImGui::Selectable(PS_AGONY2D::Integrator::integratorTypeToString(enumValue), isSelected))
                {
                    integratorType = enumValue;
                }
            }

            ImGui::EndCombo();
        }
        settings.integratorSettings.integratorType = integratorType;
    }

    // Physics diagnostics.
    if (ImGui::CollapsingHeader("Physics Diagnostics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Track body collision solver constraint errors", &settings.trackBodyCollisionSolverConstraintErrors);
        if (settings.trackBodyCollisionSolverConstraintErrors)
        {
            ImGui::Text("Velocity E: %.4f", simulationData.bodyCollisionSolverVelocityError);
            ImGui::Text("Position E: %.4f", simulationData.bodyCollisionSolverPositionError);
        }
        else
        {
            ImGui::TextDisabled("Disabled.");
        }
    }

    // Render options.
    if (ImGui::CollapsingHeader("Render Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Collider AABBs", &renderOptions.colliderAABBs);

        ImGui::Text("Broad-phase AABBs:");
        ImGui::Indent();
        {
            ImGui::Checkbox("Dynamic ", &renderOptions.broadPhaseDynamicAABBs);
            if (renderOptions.broadPhaseDynamicAABBs)
            {
                ImGui::SameLine();
                ImGui::ColorEdit3("##DynamicAABBColor", &renderOptions.broadPhaseDynamicAABBColor.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip);
            }

            ImGui::Checkbox("Static  ", &renderOptions.broadPhaseStaticAABBs);
            if (renderOptions.broadPhaseStaticAABBs)
            {
                ImGui::SameLine();
                ImGui::ColorEdit3("##StaticAABBColor", &renderOptions.broadPhaseStaticAABBColor.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip);
            }

            ImGui::Checkbox("Sleeping", &renderOptions.broadPhaseSleepingAABBs);
            if (renderOptions.broadPhaseSleepingAABBs)
            {
                ImGui::SameLine();
                ImGui::ColorEdit3("##SleepingAABBColor", &renderOptions.broadPhaseSleepingAABBColor.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip);
            }
        }
        ImGui::Unindent();

        ImGui::Checkbox("Contact Points", &renderOptions.contactPoints);

        ImGui::Checkbox("Outline Colliders", &renderOptions.outlineColliders);
        if (renderOptions.outlineColliders)
        {
            ImGui::SameLine();
            ImGui::ColorEdit3("##OutlineColor", &renderOptions.outlineColor.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip);
        }
    }

    // Renderer memory hierarchy.
    if (ImGui::CollapsingHeader("Renderer Memory Metrics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("RAM:  %s", formatSizeBinary(simulationRenderer.getMemoryUsage()).c_str());
        ImGui::Text("VRAM: %s", formatSizeBinary(simulationRenderer.getVideoMemoryUsage()).c_str());
    }

    // Simulation memory hierarchy.
    if (ImGui::CollapsingHeader("Simulation Memory Metrics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const auto& memory = simulationData.memoryUsage;

        const size_t shapeTotal =
            memory.circleData +
            memory.boxData +
            memory.polygonData +
            memory.capsuleData;

        size_t constraintsTotal = 0;
        size_t solversTotal = memory.bodyCollisionSolver;

        for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
        {
            constraintsTotal += memory.constraintData[i];

            solversTotal += simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i)).getSolverMemoryUsage();
        }

        const size_t totalMemory =
            memory.bodyData +
            memory.colliderData +
            shapeTotal +
            memory.materialData +
            memory.collisionCallback +
            memory.broadPhaseDetector +
            memory.narrowPhaseDetector +
            solversTotal +
            constraintsTotal +
            memory.solvingPlanner +
            memory.islandManager +
            memory.other;

        ImGui::Text("Total: %s", formatSizeBinary(totalMemory).c_str());
        ImGui::Separator();

        if (ImGui::BeginTable("MemoryTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable))
        {
            ImGui::TableSetupColumn("Component", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Allocated Size", ImGuiTableColumnFlags_WidthFixed, 110.0f);
            ImGui::TableSetupColumn("% Total", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            // Helper to quickly format table rows cleanly with support for hierarchical indenting.
            auto addMemoryRow = [&](const char* name, size_t bytes, bool indent = false) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (indent)
                {
                    ImGui::Indent(12.0f);
                    ImGui::TextDisabled("* %s", name);
                    ImGui::Unindent(12.0f);
                }
                else
                {
                    ImGui::TextUnformatted(name);
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(formatSizeBinary(bytes).c_str());

                ImGui::TableSetColumnIndex(2);
                double percentage = totalMemory > 0 ? (static_cast<double>(bytes) / totalMemory) * 100.0 : 0.0;
                ImGui::Text("%.1f%%", percentage);
                };

            addMemoryRow("Bodies", memory.bodyData);
            addMemoryRow("Colliders", memory.colliderData);

            addMemoryRow("Shapes", shapeTotal);
            addMemoryRow("Circles", memory.circleData, true);
            addMemoryRow("Boxes", memory.boxData, true);
            addMemoryRow("Polygons", memory.polygonData, true);
            addMemoryRow("Capsules", memory.capsuleData, true);

            {
                addMemoryRow("Constraints", constraintsTotal);
                for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
                {
                    const auto& system = simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i));
                    const std::string label = std::string(system.getName()) + "s";

                    addMemoryRow(label.c_str(), memory.constraintData[i], true);
                }
            }

            addMemoryRow("Broad Phase Detector", memory.broadPhaseDetector);
            addMemoryRow("Narrow Phase Detector", memory.narrowPhaseDetector);
            addMemoryRow("Materials", memory.materialData);
            addMemoryRow("Collision Callbacks", memory.collisionCallback);

            {
                addMemoryRow("Solvers", solversTotal);
                addMemoryRow("Body Collision Solver", memory.bodyCollisionSolver, true);
                for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
                {
                    const auto& system = simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i));
                    const std::string label = std::string(system.getName()) + " Solver";

                    addMemoryRow(label.c_str(), system.getSolverMemoryUsage(), true);
                }
            }

            addMemoryRow("Solving Planner", memory.solvingPlanner);

            addMemoryRow("Island manager", memory.islandManager);

            addMemoryRow("Other", memory.other);

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

static bool createFramebuffer(
    Ecstasy::Graphics::WindowManager& wnd,
    Ecstasy::OpenGL::FrameBuffer& framebuffer
)
{
    TRACY_SCOPE_N("Create framebuffer");

    framebuffer.create(wnd.getWidth(), wnd.getHeight());

    const bool bindless = Ecstasy::OpenGL::Texture::getExtensions().bindless;

    const  Ecstasy::OpenGL::Texture::Parameters params
    {
        .minFilter = GL_NEAREST,
        .magFilter = GL_NEAREST,
        .wrapS = GL_CLAMP_TO_EDGE,
        .wrapT = GL_CLAMP_TO_EDGE
    };

    const Ecstasy::OpenGL::Texture::Parameters idParams
    {
        .minFilter = GL_NEAREST,
        .magFilter = GL_NEAREST,
        .wrapS = GL_CLAMP_TO_EDGE,
        .wrapT = GL_CLAMP_TO_EDGE
    };

    framebuffer.createColorAttachment("color", GL_RGBA8, params, bindless);
    framebuffer.createDepthAttachment("depth", GL_DEPTH_COMPONENT32F, params, bindless);
    framebuffer.createColorAttachment("id", GL_R32UI, idParams, /*bindless*/ false);

    if (!framebuffer.isComplete())
    {
        std::cerr << "[main]: Failed to create framebuffer.\n";
        return false;
    }

    wnd.linkFramebuffer(&framebuffer);

    framebuffer.bind();
    return true;
}

static void createThreadingBackend()
{
    auto onThreadStart = [](const Ecstasy::Core::Threading::WorkerInfo& info)
        {
            std::string threadName;

            switch (info.type)
            {
            case Ecstasy::Core::Threading::CoreType::Unknown:
                threadName = std::format("Worker_{}", info.globalIndex);
                break;
            case Ecstasy::Core::Threading::CoreType::Performance:
                threadName = std::format("Worker_{}_P{}", info.globalIndex, info.typeIndex);
                break;
            case Ecstasy::Core::Threading::CoreType::Efficiency:
                threadName = std::format("Worker_{}_E{}", info.globalIndex, info.typeIndex);
                break;
            }

            tracy::SetThreadName(threadName.c_str());
        };

    Ecstasy::Core::Threading::ThreadPoolConfig cfg = {
        .numThreads = 0, // Every. Maybe should clamp to some count, idk. Who will run this will 64 cores anyway? No difference, just a lot of empty threads.
        .coreSet = Ecstasy::Core::Threading::CoreSet::Performance,
        .smtMode = Ecstasy::Core::Threading::SmtMode::Physical,
        .pin = true,
        .onThreadStart = onThreadStart,
        .priority = Ecstasy::Core::Threading::ThreadPriority::Highest,
        .reservedCpus = 1 // First CPU is for the main thread, which also runs tasks.
    };

    static Ecstasy::Core::Threading::ThreadPool pool(cfg);
    static PS_AGONY2D::Threading::EcstasyThreadingBackend backend(pool);

    PS_AGONY2D::Threading::setThreadingBackend(&backend);

    // Pin main thread to the reserved CPU.
    if (const auto reserved = pool.getReservedCpus(); !reserved.empty())
    {
        if (!Ecstasy::Core::Threading::pinCurrentThreadToCpu(reserved[0]))
            std::cerr << "Failed to pin main thread to reserved CPU.\n";
    }

    // Set main thread priority.
    if (!Ecstasy::Core::Threading::setThreadPriority(Ecstasy::Core::Threading::ThreadPriority::Highest))
        std::cerr << "Failed to set main thread priority.\n";

    pool.printTopology();
}

static int gameFunc()
{
    // Window.
    Ecstasy::Graphics::WindowManager wnd({
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

    Ecstasy::Graphics::Input::InputManager windowInputManager;
    wnd.linkInputManager(&windowInputManager);

    // ImGui context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(wnd.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // Textures.
    Ecstasy::OpenGL::Texture::initGlobalData();

    // Create framebuffer.
    Ecstasy::OpenGL::FrameBuffer framebuffer;
    if (!createFramebuffer(wnd, framebuffer)) return 1;

    // Setup threading backend.
    createThreadingBackend();

    // Simulation.
    PS_AGONY2D::Simulation simulation{};
    //simulation.getSimulationSettings().clockSettings.updateInterval = 1.0 / 40.0;

    PS_AGONY_EDITOR::Editor editor;

    loadScene(simulation, 2);

    // Simulation renderer.
    PS_AGONY_RENDER::SimulationRenderer simulationRenderer;
    simulationRenderer.init();

    auto& camera = simulationRenderer.getCamera();
    camera.setViewRangeH(20.0f, wnd.getAspectRatio());

    PS_AGONY_RENDER::RenderOptions renderOptions{}; // Default.

    // OpenGL states.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DITHER);

    // Debug data.
    DebugData debugData;

    // Main loop.
    double lastTime = glfwGetTime();
    while (!wnd.shouldClose())
    {
        // Poll events.
        wnd.pollEvents();
        windowInputManager.processInput();

        // Update camera aspect ratio.
        camera.aspectRatio = wnd.getAspectRatio();

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
            const PS_AGONY2D::Real cameraSpeed = 1.0 * deltaTime * camera.viewRange;
            const PS_AGONY2D::Real zoomSpeed = 2.0 * deltaTime;

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

            if (windowInputManager.isKeyJustPressed(GLFW_KEY_P))
                simulation.setPause(!simulation.isPaused());
        }

        // Editor: picking, dragging, creation, camera pan/zoom.
        editor.update(simulation, simulationRenderer, deltaTime);

        // Simulation.
        simulation.update(deltaTime);
        if (!simulation.isPaused())
            debugData.simulationDebugData = simulation.getDebugData();

        // Smooth debug data.
        {
            constexpr float targetAlpha = 0.8f;

            float oneMinusAlpha = std::pow<double>(1.0 - targetAlpha, deltaTime * 60.0);
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
            framebuffer.setDrawBuffers({ "color", "id" });

            glDisable(GL_BLEND);
            glColorMaski(PS_AGONY_RENDER::RenderAttachments::idIndex, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

            // Clean screen and other buffers.
            const float bgColor[4] = { 0.0f, 0.0f, 0.1f, 0.0f };
            framebuffer.clearDrawBufferFloat("color", bgColor);

            const uint32_t idClear[4] = { 0, 0, 0, 0 };
            framebuffer.clearDrawBufferUint("id", idClear);

            // Get camera AABB.
            PS_AGONY2D::AABB cameraAABB;
            {
                const PS_AGONY2D::Vec2 minCorner = camera.screenToWorldSpace({ -1.0, -1.0 });
                const PS_AGONY2D::Vec2 maxCorner = camera.screenToWorldSpace({ 1.0,  1.0 });

                cameraAABB = {
                    .minX = std::fmin(minCorner.x, maxCorner.x),
                    .minY = std::fmin(minCorner.y, maxCorner.y),
                    .maxX = std::fmax(minCorner.x, maxCorner.x),
                    .maxY = std::fmax(minCorner.y, maxCorner.y)
                };
            }

            // Build GUI window commands.
            renderGUI(simulation, simulationRenderer, renderOptions, debugData);
            editor.drawUI(simulation, simulationRenderer);

            // Render simulation to FBO.
            renderOptions.interpolation = deltaTime < simulation.getSimulationSettings().clockSettings.updateInterval;
            const GLuint idTex = framebuffer.getTexture("id").value()->getID();
            simulationRenderer.renderSimulation(simulation, simulation.getRenderAlpha(), cameraAABB, renderOptions, idTex);

            editor.renderPreview(simulationRenderer);

            // Blit scene color attachment.
            framebuffer.blitToDefaultFramebuffer(wnd.getWidth(), wnd.getHeight());

            // Unbind offscreen FBO containing integer attachment before rendering UI.
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            // Render ImGui draw data to default backbuffer
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // Rebind offscreen FBO for next frame initialization.
            framebuffer.bind();

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
    return gameFunc();
}