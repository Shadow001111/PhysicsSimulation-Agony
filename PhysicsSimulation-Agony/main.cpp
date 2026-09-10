#include "Scenes.h"

#include "Ecstasy/Core/TracyProfiler.h"

#include "Physics/Simulation.h"
#include "Physics/SimulationRenderer.h"

#include "Ecstasy/Graphics/WindowManager.h"
#include "Ecstasy/Graphics/TextRenderer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <sstream>


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

struct ObjectCreatorState
{
    bool isFocused = false;
    PS_AGONY::BodyType bodyType;
    PS_AGONY::CircleCreateParams circleParams;
    PS_AGONY::BoxCreateParams boxParams;
    PS_AGONY::PolygonCreateParams polygonParams;
};


static void renderGUI(PS_AGONY::Simulation& simulation, const DebugData& debugData, bool& pauseSimulation)
{
    ImGui::Begin("Simulation Diagnostics");

    const auto& simulationData = debugData.simulationDebugData;

    // App/Performance Section.
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

    // Interactive Simulation Settings.
    auto& settings = simulation.getSimulationSettings();
    if (ImGui::CollapsingHeader("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        float timeScale = static_cast<float>(settings.clockSettings.timeScale);
        if (ImGui::SliderFloat("Time scale", &timeScale, 0.0f, 1.0f, "%.3f"))
        {
            settings.clockSettings.timeScale = static_cast<PS_AGONY::Real>(timeScale);
        }

        float hz = static_cast<float>(1.0 / settings.clockSettings.updateInterval);
        if (ImGui::SliderFloat("Update Rate (Hz)", &hz, 10.0f, 1000.0f, "%.0f Hz"))
        {
            settings.clockSettings.updateInterval = static_cast<PS_AGONY::Real>(1.0f / hz);
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

        for (int i = 0; i < int(PS_AGONY::ConstraintType::COUNT); i++)
        {
            auto& iterationSetting = settings.constraintIterations[i];
            int iters = int(iterationSetting);

            const auto* system = simulation.getConstraintSystemAndSolver(PS_AGONY::ConstraintType(i)).first;
            if (!system) continue;

            const std::string label = std::string(system->getName()) + " Iterations";
            if (ImGui::SliderInt(label.c_str(), &iters, 1, 50))
            {
                iterationSetting = iters;
            }
        }
    }

    // Physics diagnostics.
    if (ImGui::CollapsingHeader("Physics Diagnostics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Track body collision solver constraint errors", &settings.trackBodyCollisionSolverConstraintErrors);
        if (settings.trackBodyCollisionSolverConstraintErrors)
        {
            ImGui::Text("Total velocity constraint error: %.4f", simulationData.bodyCollisionSolverVelocityError);
            ImGui::Text("Total position constraint error: %.4f", simulationData.bodyCollisionSolverPositionError);
        }
        else
        {
            ImGui::TextDisabled("Disabled.");
        }

        ImGui::Checkbox("Track Kinetic Energy", &settings.trackBodyKineticEnergySum);
        if (settings.trackBodyKineticEnergySum)
        {
            ImGui::Text("Total kinetic knergy: %.4f", simulationData.bodyKineticEnergySum);
        }
        else
        {
            ImGui::TextDisabled("Disabled.");
        }
    }

    // Memory hierarchy.
    if (ImGui::CollapsingHeader("Memory Metrics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const size_t shapeTotal =
            simulationData.circleDataMemoryUsage +
            simulationData.boxDataMemoryUsage +
            simulationData.polygonDataMemoryUsage;

        const size_t colliderTotal = simulationData.colliderDataMemoryUsage;

        size_t constraintsTotal = 0;
        size_t solversTotal = simulationData.bodyCollisionSolverMemoryUsage;

        for (int i = 0; i < int(PS_AGONY::ConstraintType::COUNT); i++)
        {
            constraintsTotal += simulationData.constraintDataMemoryUsage[i];

            const auto pair = simulation.getConstraintSystemAndSolver(PS_AGONY::ConstraintType(i));
            const auto* solver = pair.second;
            if (!solver) continue;
            solversTotal += solver->getMemoryUsage();
        }

        const size_t totalMemory =
            simulationData.bodyDataMemoryUsage +
            colliderTotal +
            shapeTotal +
            constraintsTotal +
            simulationData.broadPhaseDetectorMemoryUsage +
            simulationData.narrowPhaseDetectorMemoryUsage +
            simulationData.bodyCollisionSolverMemoryUsage;

        ImGui::Text("Total System Footprint: %s", formatSizeBinary(totalMemory).c_str());
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

            addMemoryRow("Bodies", simulationData.bodyDataMemoryUsage);
            addMemoryRow("Colliders", simulationData.colliderDataMemoryUsage);

            addMemoryRow("Shapes", shapeTotal);
            addMemoryRow("Circles", simulationData.circleDataMemoryUsage, true);
            addMemoryRow("Boxes", simulationData.boxDataMemoryUsage, true);
            addMemoryRow("Polygons", simulationData.polygonDataMemoryUsage, true);

            {
                addMemoryRow("Constraints", constraintsTotal);
                for (int i = 0; i < int(PS_AGONY::ConstraintType::COUNT); i++)
                {
                    const auto pair = simulation.getConstraintSystemAndSolver(PS_AGONY::ConstraintType(i));

                    const auto* system = pair.first;
                    std::string label;
                    if (system)
                    {
                        label = std::string(system->getName()) + "s";
                    }
                    else
                    {
                        label = std::string("Unnamed constraint type");
                    }

                    addMemoryRow(label.c_str(), simulationData.constraintDataMemoryUsage[i], true);
                }
            }

            addMemoryRow("Broad Phase Detector", simulationData.broadPhaseDetectorMemoryUsage);
            addMemoryRow("Narrow Phase Detector", simulationData.narrowPhaseDetectorMemoryUsage);

            {
                addMemoryRow("Solvers", solversTotal);
                addMemoryRow("Body Collision Solver", simulationData.bodyCollisionSolverMemoryUsage, true);
                for (int i = 0; i < int(PS_AGONY::ConstraintType::COUNT); i++)
                {
                    const auto pair = simulation.getConstraintSystemAndSolver(PS_AGONY::ConstraintType(i));
                    const auto* solver = pair.second;
                    if (!solver) continue;

                    const auto* system = pair.first;
                    std::string label;
                    if (system)
                    {
                        label = std::string(system->getName()) + " Solver";
                    }
                    else
                    {
                        label = std::string("Unnamed Solver");
                    }

                    addMemoryRow(label.c_str(), solver->getMemoryUsage(), true);
                }
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

ObjectCreatorState renderObjectCreatorUI(
    PS_AGONY::Simulation& simulation,
    const PS_AGONY::Camera2D camera
)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, 20.0f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.0f));

    ImGui::Begin("Object Creator");

    bool isFocused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    static int selectedShape = 0;

    static float vel[2] = { 0.0f, 0.0f };
    static float rotation = 0.0f;
    static float angularVelocity = 0.0f;
    static float mass = 1.0f;
    static int materialIndex = 0;

    static float circleRadius = 1.0f;
    static float boxWidth = 1.0f, boxHeight = 1.0f;

    static std::vector<PS_AGONY::Vec2> polyVertices = {
        { -1.0f, -1.0f },
        {  1.0f, -1.0f },
        {  0.0f,  1.0f }
    };

    ImGui::Combo("Shape Type", &selectedShape, "Circle\0Box\0Polygon\0");
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Common Physics Parameters", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::DragFloat2("Initial Velocity", vel, 0.1f);
        ImGui::DragFloat("Rotation (rad)", &rotation, 0.05f);
        ImGui::DragFloat("Angular Velocity", &angularVelocity, 0.05f);
        ImGui::DragFloat("Mass", &mass, 0.1f, 0.0f, 1000.0f, "%.3f");
        ImGui::InputInt("Material ID", &materialIndex);
    }
    ImGui::Separator();

    auto buildBaseParams = [&]() {
        PS_AGONY::BodyCreateParams base;
        base.position = camera.screenToWorldSpace({ 0.0, 0.0 });
        base.velocity = { static_cast<PS_AGONY::Real>(vel[0]), static_cast<PS_AGONY::Real>(vel[1])};
        base.rotation = static_cast<PS_AGONY::Real>(rotation);
        base.angularVelocity = static_cast<PS_AGONY::Real>(angularVelocity);
        base.mass = static_cast<PS_AGONY::Real>(mass);
        base.materialIndex = static_cast<PS_AGONY::MaterialIndex>(materialIndex);
        return base;
        };

    ObjectCreatorState state;
    state.isFocused = isFocused;

    if (selectedShape == 0)
    {
        if (ImGui::CollapsingHeader("Circle Parameters", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::DragFloat("Radius", &circleRadius, 0.05f, 0.01f, 100.0f);
        }

        ImGui::Spacing();
        if (ImGui::Button("Spawn Circle", ImVec2(-1, 30)))
        {
            PS_AGONY::CircleCreateParams params;
            params.base = buildBaseParams();
            params.radius = static_cast<PS_AGONY::Real>(circleRadius);
            simulation.createCircle(params);
        }

        state.bodyType = PS_AGONY::BodyType::Circle;
        state.circleParams.base = buildBaseParams();
        state.circleParams.radius = static_cast<PS_AGONY::Real>(circleRadius);
    }
    else if (selectedShape == 1)
    {
        if (ImGui::CollapsingHeader("Box Parameters", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::DragFloat("Width", &boxWidth, 0.05f, 0.01f, 100.0f);
            ImGui::DragFloat("Height", &boxHeight, 0.05f, 0.01f, 100.0f);
        }

        ImGui::Spacing();
        if (ImGui::Button("Spawn Box", ImVec2(-1, 30)))
        {
            PS_AGONY::BoxCreateParams params;
            params.base = buildBaseParams();
            params.size = { static_cast<PS_AGONY::Real>(boxWidth), static_cast<PS_AGONY::Real>(boxHeight) };
            simulation.createBox(params);
        }

        state.bodyType = PS_AGONY::BodyType::Box;
        state.boxParams.base = buildBaseParams();
        state.boxParams.size = { static_cast<PS_AGONY::Real>(boxWidth), static_cast<PS_AGONY::Real>(boxHeight) };
    }
    else if (selectedShape == 2)
    {
        if (ImGui::CollapsingHeader("Polygon Interactive Canvas", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("Click Grid: Add Point | Left-Drag: Move Point | Right-Click: Delete Point");

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_sz(260.0f, 260.0f);

            draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_sz.x, canvas_pos.y + canvas_sz.y), IM_COL32(25, 25, 30, 255));
            draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_sz.x, canvas_pos.y + canvas_sz.y), IM_COL32(80, 80, 90, 255));

            ImGui::InvisibleButton("poly_canvas", canvas_sz);
            const bool is_hovered = ImGui::IsItemHovered();
            const bool is_active = ImGui::IsItemActive();

            ImVec2 center(canvas_pos.x + canvas_sz.x * 0.5f, canvas_pos.y + canvas_sz.y * 0.5f);
            const float grid_scale = canvas_sz.x / 10.0f;

            draw_list->AddLine(ImVec2(canvas_pos.x, center.y), ImVec2(canvas_pos.x + canvas_sz.x, center.y), IM_COL32(60, 60, 70, 255));
            draw_list->AddLine(ImVec2(center.x, canvas_pos.y), ImVec2(center.x, canvas_pos.y + canvas_sz.y), IM_COL32(60, 60, 70, 255));

            static int dragging_node_idx = -1;
            ImVec2 mouse_pos = ImGui::GetIO().MousePos;

            PS_AGONY::Vec2 mouse_local;
            mouse_local.x = static_cast<PS_AGONY::Real>((mouse_pos.x - center.x) / grid_scale);
            mouse_local.y = static_cast<PS_AGONY::Real>(-(mouse_pos.y - center.y) / grid_scale);

            if (mouse_local.x < -5.0f) mouse_local.x = -5.0f;
            if (mouse_local.x > 5.0f) mouse_local.x = 5.0f;
            if (mouse_local.y < -5.0f) mouse_local.y = -5.0f;
            if (mouse_local.y > 5.0f) mouse_local.y = 5.0f;

            if (is_hovered && ImGui::IsMouseClicked(0))
            {
                dragging_node_idx = -1;
                for (int i = 0; i < (int)polyVertices.size(); i++)
                {
                    float vx = center.x + static_cast<float>(polyVertices[i].x) * grid_scale;
                    float vy = center.y - static_cast<float>(polyVertices[i].y) * grid_scale;
                    float dx = mouse_pos.x - vx;
                    float dy = mouse_pos.y - vy;
                    if (dx * dx + dy * dy < 64.0f)
                    {
                        dragging_node_idx = i;
                        break;
                    }
                }

                if (dragging_node_idx == -1)
                {
                    polyVertices.push_back(mouse_local);
                }
            }

            if (is_active && dragging_node_idx != -1 && ImGui::IsMouseDragging(0))
            {
                polyVertices[dragging_node_idx] = mouse_local;
            }

            if (ImGui::IsMouseReleased(0))
            {
                dragging_node_idx = -1;
            }

            if (is_hovered && ImGui::IsMouseClicked(1))
            {
                for (int i = 0; i < (int)polyVertices.size(); i++)
                {
                    float vx = center.x + static_cast<float>(polyVertices[i].x) * grid_scale;
                    float vy = center.y - static_cast<float>(polyVertices[i].y) * grid_scale;
                    float dx = mouse_pos.x - vx;
                    float dy = mouse_pos.y - vy;
                    if (dx * dx + dy * dy < 64.0f)
                    {
                        polyVertices.erase(polyVertices.begin() + i);
                        break;
                    }
                }
            }

            if (polyVertices.size() >= 2)
            {
                for (size_t i = 0; i < polyVertices.size(); i++)
                {
                    size_t next = (i + 1) % polyVertices.size();
                    ImVec2 p1(center.x + static_cast<float>(polyVertices[i].x) * grid_scale, center.y - static_cast<float>(polyVertices[i].y) * grid_scale);
                    ImVec2 p2(center.x + static_cast<float>(polyVertices[next].x) * grid_scale, center.y - static_cast<float>(polyVertices[next].y) * grid_scale);
                    draw_list->AddLine(p1, p2, IM_COL32(0, 255, 0, 255), 2.0f);
                }
            }

            for (size_t i = 0; i < polyVertices.size(); i++)
            {
                ImVec2 p(center.x + static_cast<float>(polyVertices[i].x) * grid_scale, center.y - static_cast<float>(polyVertices[i].y) * grid_scale);
                ImU32 node_color = (dragging_node_idx == (int)i) ? IM_COL32(255, 255, 0, 255) : IM_COL32(0, 190, 255, 255);
                draw_list->AddCircleFilled(p, 4.0f, node_color);
            }
        }

        ImGui::Spacing();
        if (polyVertices.size() < 3)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Polygons require at least 3 vertices!");
        }

        ImGui::BeginDisabled(polyVertices.size() < 3);
        if (ImGui::Button("Spawn Polygon", ImVec2(-1, 30)))
        {
            PS_AGONY::PolygonCreateParams params;
            params.base = buildBaseParams();
            params.localVertices = polyVertices.data();
            params.verticesCount = polyVertices.size();
            simulation.createPolygon(params);
        }
        ImGui::EndDisabled();

        state.bodyType = PS_AGONY::BodyType::Polygon;
        state.polygonParams.base = buildBaseParams();
        state.polygonParams.localVertices = polyVertices.data();
        state.polygonParams.verticesCount = polyVertices.size();
    }

    ImGui::End();
    return state;
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
    {
        TRACY_SCOPE_N("Create framebuffer");

        framebuffer.create(wnd.getWidth(), wnd.getHeight());

        const  Ecstasy::OpenGL::Texture::Parameters params
        {
            .minFilter = GL_NEAREST,
            .magFilter = GL_NEAREST,
            .wrapS = GL_CLAMP_TO_EDGE,
            .wrapT = GL_CLAMP_TO_EDGE
        };
        const bool bindless = Ecstasy::OpenGL::Texture::getExtensions().bindless;

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

    // Text renderer.
    Ecstasy::Graphics::TextRenderer::init();
    Ecstasy::Graphics::TextRenderer::loadFont("RusEngMinecraft", 8);
    Ecstasy::Graphics::TextRenderer::setCurrentFont("RusEngMinecraft");
    Ecstasy::Graphics::TextRenderer::setGlyphInstanceBatchSize(1024);

    // Simulation.
    PS_AGONY::Simulation simulation{};
    bool pauseSimulation = false;

    auto& mainBodyHolder = simulation.getMainBodyHolder();

    loadScene(simulation, 3);

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
            framebuffer.setDrawBuffers({ "color" });

            // Clean screen.
            const float bgColor[4] = { 0.0f, 0.0f, 0.1f, 0.0f };
            framebuffer.clearDrawBuffer("color", bgColor);

            // Get camera AABB.
            PS_AGONY::AABB cameraAABB;
            {
                const PS_AGONY::Vec2 minCorner = camera.screenToWorldSpace({ -1.0, -1.0 });
                const PS_AGONY::Vec2 maxCorner = camera.screenToWorldSpace({ 1.0,  1.0 });

                cameraAABB = {
                    .minX = std::fmin(minCorner.x, maxCorner.x),
                    .minY = std::fmin(minCorner.y, maxCorner.y),
                    .maxX = std::fmax(minCorner.x, maxCorner.x),
                    .maxY = std::fmax(minCorner.y, maxCorner.y)
                };
            }

            // Render simulation.
            simulationRenderer.renderSimulation(simulation, simulation.getRenderAlpha(), cameraAABB);

            // Render debug data.
            renderGUI(simulation, debugData, pauseSimulation);

            // Render object creator.
            ObjectCreatorState creatorState = renderObjectCreatorUI(simulation, camera);

            if (creatorState.isFocused)
            {
                const void* previewParams = nullptr;
                if (creatorState.bodyType == PS_AGONY::BodyType::Circle)
                {
                    previewParams = &creatorState.circleParams;
                }
                else if (creatorState.bodyType == PS_AGONY::BodyType::Box)
                {
                    previewParams = &creatorState.boxParams;
                }
                else if (creatorState.bodyType == PS_AGONY::BodyType::Polygon)
                {
                    previewParams = &creatorState.polygonParams;
                }

                simulationRenderer.renderObjectPreview(previewParams, creatorState.bodyType);
            }

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

    return result;
}