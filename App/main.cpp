#include "Scenes.h"
#include "EcstasyThreadingBackend.h"

#include "PS_AGONY2D/Simulation.h"
#include "PS_AGONY2D/MemoryUsageTracking.h"
#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include "Editor/Editor.h"

#include "Render/SimulationRenderer.h"

#include "Ecstasy/Graphics/WindowManager.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <sstream>
#include <format>
#include <string_view>
#include <cstdio>
#include <cmath>

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

static void helpMarker(const char* text)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("%s", text);
    }
}

static bool sliderReal(const char* label, PS_AGONY2D::Real& value, float min, float max, const char* format)
{
    float v = static_cast<float>(value);
    if (ImGui::SliderFloat(label, &v, min, max, format))
    {
        value = static_cast<PS_AGONY2D::Real>(v);
        return true;
    }
    return false;
}

template<typename Color>
static void colorToggle(const char* label, bool& value, const char* colorId, Color& color)
{
    ImGui::Checkbox(label, &value);
    if (value)
    {
        ImGui::SameLine(160.0f);
        ImGui::ColorEdit3(colorId, &color.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip);
    }
}

static void renderPerformancePanel(PS_AGONY2D::Simulation& simulation, const DebugData& debugData)
{
    const auto& simulationData = debugData.simulationDebugData;

    const float smoothedDelta = debugData.smoothedDelta;
    const float FPS = smoothedDelta > 0.0f ? 1.0f / smoothedDelta : 0.0f;
    ImGui::Text("FPS: %.1f (%.1f ms)", FPS, smoothedDelta * 1000.0f);

    float upsPercent = (simulationData.updatesSupposedToHappen > 0)
        ? (float)simulationData.updatesHappened / (float)simulationData.updatesSupposedToHappen
        : 0.0f;
    upsPercent = std::fmin(upsPercent, 1.0f);

    const bool lagging = upsPercent < 0.8f;
    if (lagging)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    }
    ImGui::Text("UPS: %u / %u (%.1f%%)",
        simulationData.updatesHappened,
        simulationData.updatesSupposedToHappen,
        upsPercent * 100.0f);
    if (lagging)
    {
        ImGui::PopStyleColor();
    }

    bool paused = simulation.isPaused();
    if (ImGui::Checkbox("Pause Simulation (P)", &paused))
    {
        simulation.setPause(paused);
    }
}

static void renderTimingSettings(PS_AGONY2D::Simulation::SimulationSettings& settings)
{
    ImGui::SeparatorText("Timing");

    sliderReal("Time scale", settings.clockSettings.timeScale, 0.0f, 1.0f, "%.3f");

    float hz = static_cast<float>(1.0 / settings.clockSettings.updateInterval);
    if (ImGui::SliderFloat("Update rate", &hz, 10.0f, 1000.0f, "%.0f Hz"))
    {
        settings.clockSettings.updateInterval = static_cast<PS_AGONY2D::Real>(1.0f / hz);
    }
}

static void renderSolverSettings(PS_AGONY2D::Simulation& simulation, PS_AGONY2D::Simulation::SimulationSettings& settings)
{
    ImGui::SeparatorText("Solver");

    int velIter = static_cast<int>(settings.collisionVelocitySolvingIterations);
    if (ImGui::SliderInt("Velocity iterations", &velIter, 1, 50))
    {
        settings.collisionVelocitySolvingIterations = static_cast<uint32_t>(velIter);
    }

    int posIter = static_cast<int>(settings.collisionPositionSolvingIterations);
    if (ImGui::SliderInt("Position iterations", &posIter, 1, 50))
    {
        settings.collisionPositionSolvingIterations = static_cast<uint32_t>(posIter);
    }

    for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
    {
        auto& system = simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i));
        int iters = static_cast<int>(system.iterations);

        const std::string_view name(system.getName());
        char label[64];
        std::snprintf(label, sizeof(label), "%.*s iterations", static_cast<int>(name.size()), name.data());

        if (ImGui::SliderInt(label, &iters, 1, 50))
        {
            system.iterations = static_cast<uint32_t>(iters);
        }
    }
}

template<typename CCDSettings>
static void renderCCDSettings(CCDSettings& ccd)
{
    ImGui::Indent();

    sliderReal("Fast factor", ccd.fastFactor, 0.1f, 10.0f, "%.2f");
    helpMarker("Body is 'fast' when displacement > factor * min extent.");

    sliderReal("Core fraction", ccd.coreFraction, 0.05f, 1.0f, "%.2f");
    helpMarker("Core radius / min extent. Below 1, sliding on a surface never 'hits'.");

    ImGui::Unindent();
}

static void renderCollisionSettings(PS_AGONY2D::Simulation::SimulationSettings& settings)
{
    ImGui::SeparatorText("Collision");

    ImGui::Checkbox("Collision events", &settings.collisionEventsEnabled);
    ImGui::Checkbox("CCD", &settings.continuousCollisionEnabled);
    if (settings.continuousCollisionEnabled)
    {
        renderCCDSettings(settings.ccdSettings);
    }
}

static void renderIntegratorSettings(PS_AGONY2D::Simulation::SimulationSettings& settings)
{
    ImGui::SeparatorText("Integrator");

    const auto current = settings.integratorSettings.integratorType;
    if (ImGui::BeginCombo("Type", PS_AGONY2D::Integrator::integratorTypeToString(current)))
    {
        for (int i = 0; i < static_cast<int>(PS_AGONY2D::Integrator::IntegratorType::COUNT); i++)
        {
            const auto value = PS_AGONY2D::Integrator::IntegratorType(i);
            const bool isSelected = (current == value);

            if (ImGui::Selectable(PS_AGONY2D::Integrator::integratorTypeToString(value), isSelected))
            {
                settings.integratorSettings.integratorType = value;
            }
            if (isSelected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

static void renderSimulationSettings(PS_AGONY2D::Simulation& simulation)
{
    auto& settings = simulation.getSimulationSettings();

    renderTimingSettings(settings);
    renderSolverSettings(simulation, settings);
    renderCollisionSettings(settings);
    renderIntegratorSettings(settings);
}

static void renderPhysicsDiagnostics(PS_AGONY2D::Simulation& simulation, const DebugData& debugData)
{
    auto& settings = simulation.getSimulationSettings();
    const auto& simulationData = debugData.simulationDebugData;

    ImGui::Checkbox("Track body collision solver errors", &settings.trackBodyCollisionSolverConstraintErrors);
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

static void renderRenderOptions(PS_AGONY_RENDER::RenderOptions& renderOptions)
{
    ImGui::SeparatorText("Colliders");
    ImGui::Checkbox("Collider AABBs", &renderOptions.colliderAABBs);
    ImGui::Checkbox("Contact points", &renderOptions.contactPoints);
    colorToggle("Outline colliders", renderOptions.outlineColliders, "##OutlineColor", renderOptions.outlineColor);

    ImGui::SeparatorText("Broad-phase AABBs");
    colorToggle("Dynamic", renderOptions.broadPhaseDynamicAABBs, "##DynamicAABBColor", renderOptions.broadPhaseDynamicAABBColor);
    colorToggle("Static", renderOptions.broadPhaseStaticAABBs, "##StaticAABBColor", renderOptions.broadPhaseStaticAABBColor);
    colorToggle("Sleeping", renderOptions.broadPhaseSleepingAABBs, "##SleepingAABBColor", renderOptions.broadPhaseSleepingAABBColor);
}

struct MemoryTotals
{
    size_t shapes = 0;
    size_t constraints = 0;
    size_t solvers = 0;
    size_t total = 0;
};

static MemoryTotals computeMemoryTotals(PS_AGONY2D::Simulation& simulation, const auto& memory)
{
    MemoryTotals t;

    t.shapes = memory.circleData + memory.boxData + memory.polygonData + memory.capsuleData;
    t.solvers = memory.bodyCollisionSolver;

    for (int i = 0; i < int(PS_AGONY2D::ConstraintType::COUNT); i++)
    {
        t.constraints += memory.constraintData[i];
        t.solvers += simulation.getConstraintSystem(PS_AGONY2D::ConstraintType(i)).getSolverMemoryUsage();
    }

    t.total =
        memory.bodyData +
        memory.colliderData +
        t.shapes +
        memory.materialData +
        memory.collisionCallback +
        memory.broadPhaseDetector +
        memory.narrowPhaseDetector +
        t.solvers +
        t.constraints +
        memory.solvingPlanner +
        memory.islandManager +
        memory.ccd +
        memory.other;

    return t;
}

static void addMemoryRow(const char* name, size_t bytes, size_t total, bool indent = false)
{
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
    const double percentage = total > 0 ? (static_cast<double>(bytes) / static_cast<double>(total)) * 100.0 : 0.0;
    ImGui::Text("%.1f%%", percentage);
}

static void renderMemoryExperimental(const PS_AGONY2D::MemoryUsageTracking& tracking)
{
    if (!ImGui::BeginTable("MemoryExperimentalTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable))
    {
        return;
    }

    ImGui::TableSetupColumn("Component", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Own", ImGuiTableColumnFlags_WidthFixed, 110.0f);
    ImGui::TableSetupColumn("Total", ImGuiTableColumnFlags_WidthFixed, 110.0f);
    ImGui::TableSetupColumn("% Total", ImGuiTableColumnFlags_WidthFixed, 60.0f);
    ImGui::TableHeadersRow();

    double percentScale = 0.0; // 100 / root total. First item is the root.
    bool isRoot = true;

    for (const auto& item : tracking)
    {
        const auto& node = item.node;
        const size_t total = node.ownBytes + node.childrenSumBytes;
        if (isRoot)
        {
            percentScale = total > 0 ? 100.0 / static_cast<double>(total) : 0.0;
            isRoot = false;
        }

        const char* name = node.string ? node.string : "?";

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (item.depth > 0)
        {
            const float indent = 12.0f * static_cast<float>(item.depth);
            ImGui::Indent(indent);
            ImGui::TextDisabled("* %s", name);
            ImGui::Unindent(indent);
        }
        else
        {
            ImGui::TextUnformatted(name);
        }

        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(formatSizeBinary(node.ownBytes).c_str());

        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(formatSizeBinary(total).c_str());

        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.1f%%", std::fmin(static_cast<double>(total) * percentScale, 100.0));
    }

    ImGui::EndTable();
}

static void renderGUI(
    PS_AGONY2D::Simulation& simulation,
    PS_AGONY_RENDER::SimulationRenderer& simulationRenderer,
    PS_AGONY_RENDER::RenderOptions& renderOptions,
    const DebugData& debugData,
    const PS_AGONY2D::MemoryUsageTracking& memoryTracking
)
{
    if (!ImGui::Begin("Simulation Diagnostics"))
    {
        ImGui::End();
        return;
    }

    renderPerformancePanel(simulation, debugData);
    ImGui::Spacing();

    if (ImGui::BeginTabBar("##MainTabs"))
    {
        if (ImGui::BeginTabItem("Simulation"))
        {
            renderSimulationSettings(simulation);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Diagnostics"))
        {
            renderPhysicsDiagnostics(simulation, debugData);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Rendering"))
        {
            renderRenderOptions(renderOptions);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Memory"))
        {
            renderMemoryExperimental(memoryTracking);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
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

    loadScene(simulation, 1);

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
    PS_AGONY2D::MemoryUsageTracking memoryTracking;

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

        // Reset debug data.
        memoryTracking.reset();

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
        simulation.collectMemoryUsage(memoryTracking, 0);

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
            memoryTracking.finalize();
            renderGUI(simulation, simulationRenderer, renderOptions, debugData, memoryTracking);
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