#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
	// Rounds p to the grid if snapping is enabled, otherwise returns p.
	Vec2 Editor::snap(Vec2 p) const noexcept
	{
		if (!snapEnabled) return p;
		const Real g = static_cast<Real>(gridSize);
		const Real invG = Real(1) / g;
		return glm::round(p * invG) * g;
	}

	// Sets the status line text and restarts its 3 s display timer.
	void Editor::setStatus(const char* text)
	{
		status = text;
		statusTimer = 3.0f;
	}

	// Converts window pixel coordinates to world space (pixel -> NDC -> camera).
	Vec2 Editor::screenToWorld(PS_AGONY2D::Camera2D& cam, float px, float py) const
	{
		const ImVec2 d = ImGui::GetIO().DisplaySize;
		const Vec2 ndc(
			static_cast<double>(px) / d.x * 2.0 - 1.0,
			-(static_cast<double>(py) / d.y * 2.0 - 1.0)
		);
		return cam.screenToWorldSpace(ndc);
	}

	// Projects a world point to window pixel coordinates using the cached viewProjection.
	ImVec2 Editor::toScreen(Vec2 world) const
	{
		const ImVec2 d = ImGui::GetIO().DisplaySize;
		const auto clip = viewProjection * Mat4::col_type(world.x, world.y, 0, 1);
		const float nx = static_cast<float>(clip.x / clip.w);
		const float ny = static_cast<float>(clip.y / clip.w);
		return ImVec2((nx * 0.5f + 0.5f) * d.x, (0.5f - ny * 0.5f) * d.y);
	}

	// Returns the index of body in `selected` (compared by dense index), or -1.
	int Editor::findSelected(const ObjectAPI& api, Handle body) const
	{
		const ObjectDenseIndex idx = api.getBodyIndex(body);
		for (size_t i = 0; i < selected.size(); i++)
		{
			if (api.getBodyIndex(selected[i]) == idx) return static_cast<int>(i);
		}
		return -1;
	}

	// Updates the selection. additive: toggles body. Otherwise replaces the selection with body,
	// unless body is already selected (keeps a multi-selection so it can be dragged).
	void Editor::select(const ObjectAPI& api, Handle body, bool additive)
	{
		const int found = findSelected(api, body);
		if (additive)
		{
			if (found >= 0) selected.erase(selected.begin() + found);
			else selected.push_back(body);
			return;
		}
		if (found < 0)
		{
			selected.clear();
			selected.push_back(body);
		}
	}

	// Destroys all selected bodies. Ends any drag first.
	void Editor::deleteSelected(Simulation& sim)
	{
		ObjectAPI& api = sim.objects();
		endDrag(sim);
		for (const Handle h : selected) api.destroyBody(h);
		selected.clear();
		activeCollider = Handle{};
		hasFirstAnchor = false;
	}

	// Per-frame input pass. Drops stale handles, updates mouse/camera cache, camera pan and zoom,
	// hotkeys, then runs the active tool.
	void Editor::update(Simulation& sim, PS_AGONY_RENDER::SimulationRenderer& renderer, Real deltaTime)
	{
		ImGuiIO& io = ImGui::GetIO();
		PS_AGONY2D::Camera2D& cam = renderer.getCamera();
		ObjectAPI& api = sim.objects();

		statusTimer -= static_cast<float>(deltaTime);

		// Drop stale handles (bodies can die from callbacks, deletes, etc.).
		std::erase_if(selected, [&](Handle h) { return !api.isBodyValid(h); });
		if (hasFirstAnchor && !api.isBodyValid(firstBody)) hasFirstAnchor = false;
		if (!api.isColliderValid(activeCollider)) activeCollider = Handle{};

		auto& holder = sim.getMainBodyHolder();
		std::erase_if(holder.heldBodies, [&](const auto& held) { return !api.isBodyValid(held.body); });

		// Mouse.
		mouseWorld = screenToWorld(cam, io.MousePos.x, io.MousePos.y);
		const Vec2 a = screenToWorld(cam, 0.0f, 0.0f);
		const Vec2 b = screenToWorld(cam, 1.0f, 0.0f);
		worldPerPixel = std::fabs(b.x - a.x);

		const bool busy = dragMode != DragMode::None || boxSelecting || creating;
		const bool overUI = io.WantCaptureMouse && !busy;

		// Camera: wheel zoom at cursor, RMB/MMB pan.
		if (!overUI && dragMode == DragMode::None)
		{
			if (io.MouseWheel != 0.0f)
			{
				cam.setViewRangeH(cam.viewRange * std::pow(0.9, io.MouseWheel), cam.aspectRatio);
				const Vec2 after = screenToWorld(cam, io.MousePos.x, io.MousePos.y);
				cam.position.x += mouseWorld.x - after.x;
				cam.position.y += mouseWorld.y - after.y;
			}
			if (io.MouseDown[1] || io.MouseDown[2])
			{
				const Vec2 prev = screenToWorld(cam, io.MousePos.x - io.MouseDelta.x, io.MousePos.y - io.MouseDelta.y);
				cam.position.x -= mouseWorld.x - prev.x;
				cam.position.y -= mouseWorld.y - prev.y;
			}
		}
		viewProjection = cam.getProjectionMatrix() * cam.getViewMatrix();

		// Hotkeys.
		if (!io.WantCaptureKeyboard)
		{
			if (ImGui::IsKeyPressed(ImGuiKey_1, false)) tool = Tool::Select;
			if (ImGui::IsKeyPressed(ImGuiKey_2, false)) tool = Tool::Create;
			if (ImGui::IsKeyPressed(ImGuiKey_3, false)) tool = Tool::Constraint;
			if (ImGui::IsKeyPressed(ImGuiKey_G, false)) snapEnabled = !snapEnabled;
			if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) && dragMode == DragMode::None) deleteSelected(sim);
			if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) && tool == Tool::Select) selected.clear();
		}

		if (tool != Tool::Create) { creating = false; ghostVisible = false; }
		if (tool != Tool::Constraint) hasFirstAnchor = false;
		if (tool != Tool::Select && dragMode != DragMode::None) endDrag(sim);

		switch (tool)
		{
		case Tool::Select: updateSelect(sim, overUI, deltaTime); break;
		case Tool::Create: updateCreate(sim, overUI); break;
		case Tool::Constraint: updateConstraint(sim, overUI); break;
		}
	}

	// Builds constraint names and the combo item list once from the registered systems.
	void Editor::cacheConstraintNames(Simulation& sim)
	{
		if (namesCached) return;
		namesCached = true;
		constraintCombo.clear();
		for (int i = 0; i < int(ConstraintType::COUNT); i++)
		{
			auto& system = sim.getConstraintSystem(ConstraintType(i));
			constraintNames[i] = std::string(system.getName());
			constraintCombo += constraintNames[i];
			constraintCombo.push_back('\0');
		}
	}
}
