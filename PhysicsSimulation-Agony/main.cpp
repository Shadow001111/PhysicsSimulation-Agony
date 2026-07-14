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

struct ObjectCreatorState
{
    bool isFocused = false;
    PS_AGONY::BodyType bodyType;
    PS_AGONY::Simulation::CircleCreateParams circleParams;
    PS_AGONY::Simulation::BoxCreateParams boxParams;
    PS_AGONY::Simulation::PolygonCreateParams polygonParams;
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
        PS_AGONY::Simulation::BodyCreateParams base;
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
            PS_AGONY::Simulation::CircleCreateParams params;
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
            PS_AGONY::Simulation::BoxCreateParams params;
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
            PS_AGONY::Simulation::PolygonCreateParams params;
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
            simulationRenderer.renderSimulation(simulation);

            // Render debug data.
            renderDebugData(debugData, pauseSimulation);

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