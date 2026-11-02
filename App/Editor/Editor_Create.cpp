#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
	// Fills circleP / boxP / polyP from the create settings and the ghost transform.
	// Regular polygon vertices are regenerated, free polygon vertices are taken from polyScratch as is.
	void Editor::rebuildGhost()
	{
		BodyCreateParams base;
		base.position = ghostPos;
		base.velocity = ghostVel;
		base.rotation = rotation;
		base.angularVelocity = angularVelocity;
		base.mass = isStatic ? Real(0) : static_cast<Real>(mass);
		base.materialIndex = static_cast<MaterialIndex>(std::max(materialIndex, 0));

		circleP.base = base;
		circleP.radius = radius;
		boxP.base = base;
		boxP.size = Vec2(size[0], size[1]);

		polyP.base = base;
		if (shape == CreateShape::RegularPolygon)
		{
			const int n = std::max(sides, 3);
			polyScratch.resize(static_cast<size_t>(n));
			for (int i = 0; i < n; i++)
			{
				const double a = 2.0 * std::numbers::pi * i / n;
				polyScratch[i] = Vec2(std::cos(a) * radius, std::sin(a) * radius);
			}
		}
		polyP.localVertices = std::span<Vec2>(polyScratch);
	}

	// Creates a body from the current ghost params (not for free polygon) and selects it.
	void Editor::spawn(Simulation& sim)
	{
		ObjectAPI& api = sim.objects();
		std::optional<Handle> h;

		switch (shape)
		{
		case CreateShape::Circle: h = api.createCircleH(circleP); break;
		case CreateShape::Box: h = api.createBoxH(boxP); break;
		case CreateShape::RegularPolygon: h = api.createPolygonH(polyP); break;
		case CreateShape::Capsule:
		{
			CapsuleCreateParams p;
			p.base = circleP.base;
			p.length = capsuleLength;
			p.radius = capsuleRadius;
			h = api.createCapsuleH(p);
			break;
		}
		default: break;
		}

		if (!h) { setStatus("Creation failed."); return; }
		selected.clear();
		selected.push_back(*h);
	}

	// Creates a polygon from freePoints, recentered on their mean, and selects it. Needs >= 3 points.
	void Editor::spawnFreePolygon(Simulation& sim)
	{
		if (freePoints.size() < 3) { setStatus("Need at least 3 points."); return; }

		Real cx = 0, cy = 0;
		for (const Vec2& p : freePoints) { cx += p.x; cy += p.y; }
		cx /= static_cast<Real>(freePoints.size());
		cy /= static_cast<Real>(freePoints.size());

		polyScratch.clear();
		for (const Vec2& p : freePoints) polyScratch.emplace_back(p.x - cx, p.y - cy);

		rebuildGhost();
		polyP.base.position = Vec2(cx, cy);
		polyP.base.velocity = Vec2(0, 0);

		const auto h = sim.objects().createPolygonH(polyP);
		if (!h) { setStatus("Polygon rejected (degenerate or self-intersecting)."); return; }

		freePoints.clear();
		selected.clear();
		selected.push_back(*h);
	}

	// Create tool, per frame. Ghost placement, Esc cancel, click-drag launch
	// (velocity = (start - mouse) * launchScale, zero inside the dead zone), spawn on release.
	// Free polygon: click adds a point, Backspace removes the last, Enter finishes.
	void Editor::updateCreate(Simulation& sim, bool overUI)
	{
		ImGuiIO& io = ImGui::GetIO();
		const Vec2 snapped = snap(mouseWorld);

		ghostVisible = !overUI || creating;
		if (creating)
		{
			ghostPos = createStart;
			const Real dx = createStart.x - mouseWorld.x;
			const Real dy = createStart.y - mouseWorld.y;
			const Real dead = worldPerPixel * kMinDragPx;
			// 1 if the mouse left the dead zone around createStart, else 0 (a plain click does not launch).
			const Real moved = static_cast<Real>(dx * dx + dy * dy >= dead * dead);
			const Real k = launchScale * moved * static_cast<Real>(!isStatic); // Static bodies never launch.
			ghostVel = Vec2(dx * k, dy * k);
		}
		else
		{
			ghostPos = snapped;
			ghostVel = Vec2(0, 0);
		}
		if (shape != CreateShape::FreePolygon) rebuildGhost();

		if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
		{
			creating = false;
			freePoints.clear();
		}

		if (shape == CreateShape::FreePolygon)
		{
			if (!overUI && ImGui::IsMouseClicked(0)) freePoints.push_back(snapped);
			if (!io.WantCaptureKeyboard)
			{
				if (ImGui::IsKeyPressed(ImGuiKey_Backspace) && !freePoints.empty()) freePoints.pop_back();
				if (ImGui::IsKeyPressed(ImGuiKey_Enter, false)) spawnFreePolygon(sim);
			}
			return;
		}

		if (!overUI && ImGui::IsMouseClicked(0))
		{
			creating = true;
			createStart = snapped;
		}
		if (creating && !io.MouseDown[0])
		{
			rebuildGhost();
			spawn(sim);
			creating = false;
		}
	}

	// Draws the ghost body for the active create shape. Stores the renderer's error for drawUI.
	void Editor::renderPreview(PS_AGONY_RENDER::SimulationRenderer& renderer)
	{
		previewError = nullptr;
		if (tool != Tool::Create || !ghostVisible) return;

		switch (shape)
		{
		case CreateShape::Circle: previewError = renderer.renderObjectPreview(&circleP, BodyType::Circle); break;
		case CreateShape::Box: previewError = renderer.renderObjectPreview(&boxP, BodyType::Box); break;
		case CreateShape::RegularPolygon: previewError = renderer.renderObjectPreview(&polyP, BodyType::Polygon); break;
		default: break; // Capsule and free polygon are drawn in the overlay.
		}
	}

	// Edits create tool parameters.
	void Editor::drawCreatePanel()
	{
		ImGui::Combo("Shape", reinterpret_cast<int*>(&shape), "Circle\0Box\0Capsule\0Regular polygon\0Free polygon\0");

		switch (shape)
		{
		case CreateShape::Circle:
			ImGui::DragFloat("Radius", &radius, 0.02f, 0.01f, 100.0f);
			break;
		case CreateShape::Box:
			ImGui::DragFloat2("Size", size, 0.02f, 0.01f, 100.0f);
			break;
		case CreateShape::Capsule:
			ImGui::DragFloat("Length", &capsuleLength, 0.02f, 0.01f, 100.0f);
			ImGui::DragFloat("Radius", &capsuleRadius, 0.02f, 0.01f, 100.0f);
			break;
		case CreateShape::RegularPolygon:
			ImGui::SliderInt("Sides", &sides, 3, 16);
			ImGui::DragFloat("Radius", &radius, 0.02f, 0.01f, 100.0f);
			break;
		case CreateShape::FreePolygon:
			ImGui::Text("Points: %zu (click to add, Enter = finish, Backspace = undo)", freePoints.size());
			break;
		}

		ImGui::Checkbox("Static", &isStatic);
		ImGui::BeginDisabled(isStatic);
		ImGui::DragFloat("Mass", &mass, 0.1f, 0.001f, 1e6f);
		ImGui::EndDisabled();
		ImGui::DragFloat("Rotation", &rotation, 0.02f);
		ImGui::DragFloat("Angular velocity", &angularVelocity, 0.05f);
		ImGui::InputInt("Material", &materialIndex);
		materialIndex = std::max(materialIndex, 0);
		ImGui::BeginDisabled(isStatic);
		ImGui::DragFloat("Launch scale", &launchScale, 0.05f, 0.0f, 50.0f);
		ImGui::EndDisabled();
		ImGui::TextDisabled("Click to place, hold and drag backwards to launch.");
	}
}
