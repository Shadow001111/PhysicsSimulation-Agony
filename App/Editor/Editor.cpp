#include "Editor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace PS_AGONY_EDITOR
{
	namespace
	{
		constexpr ImU32 kSelectColor = IM_COL32(255, 220, 40, 255);
		constexpr ImU32 kGhostColor = IM_COL32(60, 255, 90, 255);
		constexpr ImU32 kHintColor = IM_COL32(255, 255, 255, 200);

		const char* const kConstraintNames = "Spring\0Revolute joint\0Rod\0Prismatic joint\0Slider joint\0Motor\0";

		// TODO: express the pixel thresholds below as fractions of the screen resolution.
		constexpr Real kPickTolerancePx = 3;
		constexpr Real kMinDragPx = 4; // Dead zone for box select and launch.
		constexpr int kMotorKind = 5;  // Index of "Motor" in kConstraintNames.

		// Wrapped, dimmed help row.
		void hintRow(const char* text)
		{
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			ImGui::TextWrapped("%s", text);
			ImGui::PopStyleColor();
		}
	}

	// Rounds p to the grid if snapping is enabled, otherwise returns p.
	Vec2 Editor::snap(Vec2 p) const noexcept
	{
		if (!snapEnabled) return p;
		const Real g = static_cast<Real>(gridSize);
		return Vec2(std::round(p.x / g) * g, std::round(p.y / g) * g);
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
		const glm::dvec2 ndc(
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
		for (size_t i = 0; i < selected.size(); ++i)
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

	// Finds the collider under p within a pixel tolerance. BVH circle query, then an exact test for
	// circle/box/capsule (polygons: AABB hit only). Smallest AABB area wins, so small bodies on large ones stay pickable.
	// Returns false if nothing was hit.
	bool Editor::pickCollider(Simulation& sim, Vec2 p, ColliderDenseIndex& out)
	{
		const Real tol = worldPerPixel * kPickTolerancePx;

		foundScratch.clear();
		sim.getBroadPhaseCollisionDetector().queryCollidersInShape(
			BroadPhaseInternal::CircleQuery(p, tol),
			BroadPhaseInternal::BvhTreeTypeFlag::All,
			foundScratch
		);

		ObjectAPI& api = sim.objects();
		const ColliderSoAViewer cv = api.getColliders();

		Real bestArea = std::numeric_limits<Real>::max();
		bool found = false;

		for (const ColliderDenseIndex ci : foundScratch)
		{
			const Real dx = p.x - cv.worldPosX[ci];
			const Real dy = p.y - cv.worldPosY[ci];
			const Real c = cv.worldRotationCos[ci];
			const Real s = cv.worldRotationSin[ci];
			const Real lx = dx * c + dy * s;
			const Real ly = -dx * s + dy * c;

			bool hit = true; // Polygons: broad-phase AABB hit only (TODO: exact test once PolygonGeometry is known here).
			const BodyType type = cv.shapeType[ci];
			if (type == BodyType::Circle || type == BodyType::Box || type == BodyType::Capsule)
			{
				const auto data = api.getColliderDataH(api.getColliderHandle(ci));
				if (!data) continue;

				if (type == BodyType::Circle)
				{
					const Real r = data->radius + tol;
					hit = lx * lx + ly * ly <= r * r;
				}
				else if (type == BodyType::Box)
				{
					hit = std::fabs(lx) <= data->halfExtents.x + tol && std::fabs(ly) <= data->halfExtents.y + tol;
				}
				else // Capsule, assumed to extend along local X.
				{
					const Real cx = std::fmax(-data->halfLength, std::fmin(data->halfLength, lx));
					const Real r = data->radius + tol;
					hit = (lx - cx) * (lx - cx) + ly * ly <= r * r;
				}
			}
			if (!hit) continue;

			const Real area = (cv.aabbMaxX[ci] - cv.aabbMinX[ci]) * (cv.aabbMaxY[ci] - cv.aabbMinY[ci]);
			if (area < bestArea)
			{
				bestArea = area;
				out = ci;
				found = true;
			}
		}
		return found;
	}

	// Adds body to the main body holder with its anchor at anchorWorld (same math as Simulation::mainBodyHolderGrabAt).
	// The anchor target is the holder position + (anchorWorld - grabWorld), so every held body keeps its offset
	// from the grab point. Fails for invalid or static bodies.
	bool Editor::holdBody(Simulation& sim, Handle body, Vec2 anchorWorld, Vec2 grabWorld)
	{
		ObjectAPI& api = sim.objects();
		const ObjectDenseIndex bi = api.getBodyIndex(body);
		if (bi == Handle::kNull) return false;

		const BodySoAViewer bv = api.getBodies();
		if (bv.isStatic[bi] != 0) return false;

		const Real wx = anchorWorld.x - bv.worldCenterX[bi];
		const Real wy = anchorWorld.y - bv.worldCenterY[bi];
		const Real c = bv.rotationCos[bi];
		const Real s = bv.rotationSin[bi];

		sim.getMainBodyHolder().heldBodies.push_back({
			body,
			Vec2(wx * c + wy * s, -wx * s + wy * c),
			Vec2(anchorWorld.x - grabWorld.x, anchorWorld.y - grabWorld.y)
			});
		return true;
	}

	// Starts dragging. The clicked body is held at the click point, every other selected body is held at its
	// world center. The holder is reset to the click point with zero velocity (stale target would fling bodies).
	// Does nothing for a static body or a busy holder.
	void Editor::beginDrag(Simulation& sim, Handle body, Vec2 grabWorld)
	{
		constexpr bool kAllowSingleBodyRotation = true;

		auto& holder = sim.getMainBodyHolder();
		if (holder.isHolding()) return;

		holder.teleport(grabWorld);

		ObjectAPI& api = sim.objects();
		const BodySoAViewer bv = api.getBodies();
		const ObjectDenseIndex primary = api.getBodyIndex(body);
		if (primary == Handle::kNull) return;

		const bool multiple = selected.size() > 1;
		const Vec2 primaryAnchor = (multiple || !kAllowSingleBodyRotation)
			? Vec2(bv.worldCenterX[primary], bv.worldCenterY[primary])
			: grabWorld;

		if (!holdBody(sim, body, primaryAnchor, grabWorld)) return;

		dragBody = body;
		dragMode = DragMode::Holder;

		for (const Handle h : selected)
		{
			const ObjectDenseIndex bi = api.getBodyIndex(h);
			if (bi == Handle::kNull || bi == primary) continue;
			holdBody(sim, h, Vec2(bv.worldCenterX[bi], bv.worldCenterY[bi]), grabWorld);
		}
	}

	// Per-frame drag step. Ends on mouse release or when the clicked body dies.
	// Drops dead bodies from the holder, moves the holder to the mouse. Wheel adds angular velocity to all held bodies.
	void Editor::updateDrag(Simulation& sim, Real dt)
	{
		ObjectAPI& api = sim.objects();
		ImGuiIO& io = ImGui::GetIO();

		if (!api.isBodyValid(dragBody) || !io.MouseDown[0])
		{
			endDrag(sim);
			return;
		}

		std::erase_if(sim.getMainBodyHolder().heldBodies, [&](const auto& held) { return !api.isBodyValid(held.body); });

		sim.getMainBodyHolder().setPosition(mouseWorld, dt);
		if (io.MouseWheel != 0.0f) sim.mainBodyHolderIncreaseAngularVelocity(io.MouseWheel * 3.0);
	}

	// Ends the drag and releases all held bodies.
	void Editor::endDrag(Simulation& sim)
	{
		if (dragMode == DragMode::Holder) sim.getMainBodyHolder().heldBodies.clear();
		dragMode = DragMode::None;
		dragBody = Handle{};
	}

	// Ends box selection. Broad-phase AABB query over the box, hit bodies are added to the selection
	// (selection is cleared first unless additive). Boxes smaller than the dead zone are ignored.
	void Editor::finishBoxSelect(Simulation& sim, Vec2 end, bool additive)
	{
		boxSelecting = false;

		const Real minDrag = worldPerPixel * kMinDragPx;
		if (std::fabs(end.x - boxStart.x) < minDrag && std::fabs(end.y - boxStart.y) < minDrag) return;

		const AABB box{
			.minX = std::fmin(boxStart.x, end.x), .minY = std::fmin(boxStart.y, end.y),
			.maxX = std::fmax(boxStart.x, end.x), .maxY = std::fmax(boxStart.y, end.y)
		};

		foundScratch.clear();
		sim.getBroadPhaseCollisionDetector().queryCollidersInShape(
			BroadPhaseInternal::AABBQuery(box), BroadPhaseInternal::BvhTreeTypeFlag::All, foundScratch);

		ObjectAPI& api = sim.objects();
		if (!additive) selected.clear();

		const ColliderSoAViewer cv = api.getColliders();
		for (const ColliderDenseIndex ci : foundScratch)
		{
			const Handle body = api.getBodyHandle(cv.bodyIndex[ci]);
			if (findSelected(api, body) < 0) selected.push_back(body);
		}
	}

	// Select tool, per frame. Continues an active drag or box select, otherwise handles a click:
	// hit -> select + begin drag, miss -> clear selection + start box select.
	void Editor::updateSelect(Simulation& sim, bool overUI, Real dt)
	{
		ObjectAPI& api = sim.objects();
		ImGuiIO& io = ImGui::GetIO();

		if (dragMode != DragMode::None)
		{
			updateDrag(sim, dt);
			return;
		}
		if (boxSelecting)
		{
			if (!io.MouseDown[0]) finishBoxSelect(sim, mouseWorld, io.KeyShift);
			return;
		}
		if (overUI || !ImGui::IsMouseClicked(0)) return;

		ColliderDenseIndex ci;
		if (pickCollider(sim, mouseWorld, ci))
		{
			const Handle body = api.getBodyHandle(api.getColliders().bodyIndex[ci]);
			activeCollider = api.getColliderHandle(ci);
			select(api, body, io.KeyShift);
			if (!io.KeyShift) beginDrag(sim, body, mouseWorld);
		}
		else
		{
			if (!io.KeyShift) selected.clear();
			boxSelecting = true;
			boxStart = mouseWorld;
		}
	}

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
			for (int i = 0; i < n; ++i)
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
			const Real k = launchScale * moved;
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

	// Constraint tool, per frame. Esc cancels the first anchor. A click picks a body and an anchor
	// (local to the body's center of mass). Motor needs one click, other types two.
	void Editor::updateConstraint(Simulation& sim, bool overUI)
	{
		if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) hasFirstAnchor = false;
		if (overUI || !ImGui::IsMouseClicked(0)) return;

		ObjectAPI& api = sim.objects();
		ColliderDenseIndex ci;
		if (!pickCollider(sim, mouseWorld, ci)) { hasFirstAnchor = false; return; }

		const ObjectDenseIndex bi = api.getColliders().bodyIndex[ci];
		const BodySoAViewer bv = api.getBodies();
		const Real wx = mouseWorld.x - bv.worldCenterX[bi];
		const Real wy = mouseWorld.y - bv.worldCenterY[bi];
		const Real c = bv.rotationCos[bi];
		const Real s = bv.rotationSin[bi];
		const Vec2 local(wx * c + wy * s, -wx * s + wy * c); // Local to the center of mass.
		const Handle body = api.getBodyHandle(bi);

		if (constraintKind == kMotorKind)
		{
			createConstraintStub(sim, body, local, body, local);
			hasFirstAnchor = false;
			return;
		}

		if (!hasFirstAnchor)
		{
			hasFirstAnchor = true;
			firstBody = body;
			firstLocalAnchor = local;
			return;
		}
		createConstraintStub(sim, firstBody, firstLocalAnchor, body, local);
		hasFirstAnchor = false;
	}

	// Placeholder for constraint creation, only reports status. Anchors are local to each body's center of mass.
	void Editor::createConstraintStub(Simulation& sim, Handle a, Vec2 localA, Handle b, Vec2 localB)
	{
		(void)sim; (void)a; (void)localA; (void)b; (void)localB;
		// TODO(future API): needs the CreateParams of each constraint system.
		//
		// ObjectAPI& api = sim.objects();
		// const ObjectDenseIndex ia = api.getBodyIndex(a), ib = api.getBodyIndex(b);
		// switch (constraintKind)
		// {
		// case 0: { SpringConstraintSystem::CreateParams p; p.bodyA = ia; p.bodyB = ib; p.localAnchorA = localA; p.localAnchorB = localB; sim.constraints().createSpring(p); break; }
		// case 1: ... createRevoluteJoint
		// case 2: ... createRod
		// case 3: ... createPrismaticJoint
		// case 4: ... createSliderJoint
		// case 5: ... createMotor (single body: a / localA only)
		// }
		setStatus("Constraint creation not wired yet (see createConstraintStub).");
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

	// Copies selected bodies with their colliders (offset by 0.5, 0.5) and selects the copies.
	// Polygon colliders are skipped.
	void Editor::duplicateSelected(Simulation& sim)
	{
		ObjectAPI& api = sim.objects();
		const std::vector<Handle> sources = selected; // Creation invalidates dense indices, handles stay valid.
		std::vector<Handle> copies;
		bool skippedPolygon = false;

		for (const Handle src : sources)
		{
			const auto bd = api.getBodyDataH(src);
			if (!bd) continue;

			BodyCreateParams bp;
			bp.position = Vec2(bd->offset.x + 0.5, bd->offset.y + 0.5);
			bp.velocity = bd->velocity;
			bp.rotation = bd->rotation;
			bp.angularVelocity = bd->angularVelocity;
			bp.mass = bd->mass;
			bp.inertia = bd->inertia;
			bp.centerOfMass = bd->localCenterOfMass;

			const auto copy = api.createBodyH(bp);
			if (!copy) continue;
			copies.push_back(*copy);

			colliderScratch.clear();
			for (const ColliderDenseIndex ci : api.getBodyColliders(api.getBodyIndex(src))) colliderScratch.push_back(ci);

			for (const ColliderDenseIndex ci : colliderScratch)
			{
				const auto cd = api.getColliderDataH(api.getColliderHandle(ci));
				if (!cd) continue;

				switch (cd->shapeType)
				{
				case BodyType::Circle:
				{
					CircleColliderCreateParams p{};
					p.localOffset = cd->localOffset; p.localRotation = cd->localRotation;
					p.materialIndex = cd->materialIndex; p.radius = cd->radius;
					api.createCircleColliderH(*copy, p);
					break;
				}
				case BodyType::Box:
				{
					BoxColliderCreateParams p{};
					p.localOffset = cd->localOffset; p.localRotation = cd->localRotation;
					p.materialIndex = cd->materialIndex; p.size = Vec2(cd->halfExtents.x * 2, cd->halfExtents.y * 2);
					api.createBoxColliderH(*copy, p);
					break;
				}
				case BodyType::Capsule:
				{
					CapsuleColliderCreateParams p{};
					p.localOffset = cd->localOffset; p.localRotation = cd->localRotation;
					p.materialIndex = cd->materialIndex; p.radius = cd->radius; p.length = cd->halfLength * 2;
					api.createCapsuleColliderH(*copy, p);
					break;
				}
				default:
					skippedPolygon = true; // TODO: copy PolygonGeometry vertices once the layout is known.
					break;
				}
			}
		}

		selected = copies;
		if (skippedPolygon) setStatus("Polygon colliders were not duplicated.");
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
			if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D, false)) duplicateSelected(sim);
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

	// Draws the editor window (toolbar, tool panel, inspector in Select tool, status) and the world overlay.
	void Editor::drawUI(Simulation& sim, PS_AGONY_RENDER::SimulationRenderer& renderer)
	{
		(void)renderer;
		ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(340.0f, 600.0f), ImGuiCond_FirstUseEver);
		ImGui::Begin("Editor");

		drawToolbar(sim);
		ImGui::Separator();

		if (tool == Tool::Create) drawCreatePanel();
		if (tool == Tool::Constraint) drawConstraintPanel(sim);
		if (tool == Tool::Select) drawInspector(sim);

		if (statusTimer > 0.0f) ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", status.c_str());
		if (previewError != nullptr) ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Polygon: %s", previewError);

		ImGui::End();
		drawOverlay(sim);
	}

	// Tool selection, snap, pause, duplicate/delete buttons.
	void Editor::drawToolbar(Simulation& sim)
	{
		if (ImGui::RadioButton("Select (1)", tool == Tool::Select)) tool = Tool::Select;
		ImGui::SameLine();
		if (ImGui::RadioButton("Create (2)", tool == Tool::Create)) tool = Tool::Create;
		ImGui::SameLine();
		if (ImGui::RadioButton("Constraint (3)", tool == Tool::Constraint)) tool = Tool::Constraint;

		ImGui::Checkbox("Snap (G)", &snapEnabled);
		ImGui::SameLine();
		ImGui::SetNextItemWidth(80.0f);
		ImGui::DragFloat("Grid", &gridSize, 0.01f, 0.01f, 100.0f, "%.2f");

		bool paused = sim.isPaused();
		if (ImGui::Checkbox("Paused (P)", &paused)) sim.setPause(paused);
		ImGui::SameLine();
		// TODO(future API): if (ImGui::Button("Step")) sim.stepOnce();
		ImGui::BeginDisabled(selected.empty());
		if (ImGui::Button("Duplicate")) duplicateSelected(sim);
		ImGui::SameLine();
		if (ImGui::Button("Delete")) deleteSelected(sim);
		ImGui::EndDisabled();

		// Help rows: general controls, then Select tool controls (each on its own row).
		hintRow("Pan: RMB/MMB | Zoom: wheel | Tools: 1/2/3 | Snap: G");
		if (tool == Tool::Select)
		{
			hintRow("Select: LMB, Shift+LMB toggles, drag on empty space = box select. Del = delete, Ctrl+D = duplicate, Esc = clear.");
			hintRow("Grab: hold LMB on a body. All selected bodies are dragged together.");
			hintRow("Rotate grabbed body: wheel while holding LMB (the wheel does not zoom during a grab).");
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
		ImGui::DragFloat("Launch scale", &launchScale, 0.05f, 0.0f, 50.0f);
		ImGui::TextDisabled("Click to place, hold and drag backwards to launch.");
	}

	// Edits the single selected body: body properties, colliders, attachments.
	void Editor::drawInspector(Simulation& sim)
	{
		ObjectAPI& api = sim.objects();
		ImGui::Text("Selected: %zu", selected.size());
		if (selected.empty()) return;
		if (selected.size() > 1)
		{
			ImGui::TextDisabled("Inspector shows a single body only.");
			return;
		}

		const Handle h = selected.front();
		const auto bd = api.getBodyDataH(h);
		if (!bd) return;

		if (ImGui::CollapsingHeader("Body", ImGuiTreeNodeFlags_DefaultOpen))
		{
			float pos[2] = { static_cast<float>(bd->offset.x), static_cast<float>(bd->offset.y) };
			if (ImGui::DragFloat2("Position", pos, 0.02f)) api.setBodyPosition(h, Vec2(pos[0], pos[1]));

			float rot = static_cast<float>(bd->rotation);
			if (ImGui::DragFloat("Rotation", &rot, 0.01f)) api.setBodyRotation(h, rot);

			float vel[2] = { static_cast<float>(bd->velocity.x), static_cast<float>(bd->velocity.y) };
			if (ImGui::DragFloat2("Velocity", vel, 0.05f)) api.setBodyVelocity(h, Vec2(vel[0], vel[1]));

			float av = static_cast<float>(bd->angularVelocity);
			if (ImGui::DragFloat("Angular velocity", &av, 0.05f)) api.setBodyAngularVelocity(h, av);

			bool st = bd->isStatic;
			if (ImGui::Checkbox("Static", &st))
			{
				if (st)
				{
					lastDynamicMass = static_cast<float>(std::fmax(bd->mass, 0.001));
					api.setBodyMass(h, 0);
				}
				else
				{
					api.setBodyMass(h, lastDynamicMass);
					api.recomputeBodyInertia(h);
				}
			}

			if (!st)
			{
				float m = static_cast<float>(bd->mass);
				if (ImGui::DragFloat("Mass", &m, 0.05f, 0.001f, 1e6f)) api.setBodyMass(h, m);

				float inertia = static_cast<float>(bd->inertia);
				if (ImGui::DragFloat("Inertia", &inertia, 0.05f, 0.0f, 1e7f)) api.setBodyInertia(h, inertia);

				ImGui::SetNextItemWidth(80.0f);
				ImGui::DragFloat("##density", &density, 0.05f, 0.0f, 1e5f);
				ImGui::SameLine();
				if (ImGui::Button("Mass from density")) { api.setBodyMassFromDensity(h, density); api.recomputeBodyInertia(h); }
				if (ImGui::Button("Recompute inertia")) api.recomputeBodyInertia(h);
			}

			float com[2] = { static_cast<float>(bd->localCenterOfMass.x), static_cast<float>(bd->localCenterOfMass.y) };
			if (ImGui::DragFloat2("Local COM", com, 0.01f)) api.setBodyLocalCenterOfMass(h, Vec2(com[0], com[1]));

			if (ImGui::Button("Stop")) { api.setBodyVelocity(h, Vec2(0, 0)); api.setBodyAngularVelocity(h, 0); }
		}

		if (ImGui::CollapsingHeader("Colliders", ImGuiTreeNodeFlags_DefaultOpen))
		{
			const ObjectDenseIndex bi = api.getBodyIndex(h);
			colliderScratch.clear();
			for (const ColliderDenseIndex ci : api.getBodyColliders(bi)) colliderScratch.push_back(ci);

			Handle toDelete{};
			bool deleteRequested = false;

			for (const ColliderDenseIndex ci : colliderScratch)
			{
				const Handle ch = api.getColliderHandle(ci);
				const auto cd = api.getColliderDataH(ch);
				if (!cd) continue;

				const char* names[] = { "Circle", "Box", "Polygon", "Capsule" };
				const size_t typeIdx = std::min<size_t>(static_cast<size_t>(cd->shapeType), 3);

				ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_DefaultOpen;
				if (api.getColliderIndex(ch) == api.getColliderIndex(activeCollider)) flags |= ImGuiTreeNodeFlags_Selected;

				ImGui::PushID(static_cast<int>(ci));
				if (ImGui::TreeNodeEx("##c", flags, "%s #%u", names[typeIdx], static_cast<unsigned>(ci)))
				{
					float off[2] = { static_cast<float>(cd->localOffset.x), static_cast<float>(cd->localOffset.y) };
					if (ImGui::DragFloat2("Offset", off, 0.01f)) api.setColliderLocalOffset(ch, Vec2(off[0], off[1]));

					float lr = static_cast<float>(cd->localRotation);
					if (ImGui::DragFloat("Local rotation", &lr, 0.01f)) api.setColliderLocalRotation(ch, lr);

					int mat = static_cast<int>(cd->materialIndex);
					if (ImGui::InputInt("Material", &mat)) api.setColliderMaterial(ch, static_cast<MaterialIndex>(std::max(mat, 0)));

					if (cd->shapeType == BodyType::Circle)
					{
						float r = static_cast<float>(cd->radius);
						if (ImGui::DragFloat("Radius", &r, 0.01f, 0.001f, 1000.0f)) api.setCircleRadius(ch, r);
					}
					else if (cd->shapeType == BodyType::Box)
					{
						float sz[2] = { static_cast<float>(cd->halfExtents.x * 2), static_cast<float>(cd->halfExtents.y * 2) };
						if (ImGui::DragFloat2("Size", sz, 0.01f, 0.001f, 1000.0f)) api.setBoxSize(ch, Vec2(sz[0], sz[1]));
					}
					else if (cd->shapeType == BodyType::Capsule)
					{
						float len = static_cast<float>(cd->halfLength * 2);
						float r = static_cast<float>(cd->radius);
						const bool c1 = ImGui::DragFloat("Length", &len, 0.01f, 0.001f, 1000.0f);
						const bool c2 = ImGui::DragFloat("Radius", &r, 0.01f, 0.001f, 1000.0f);
						if (c1 || c2) api.setCapsuleShape(ch, len, r);
					}
					else
					{
						ImGui::TextDisabled("Polygon vertex editing: TODO.");
					}

					int cbId = static_cast<int>(cd->callbackId);
					float thr = static_cast<float>(cd->eventThreshold);
					ImGui::SetNextItemWidth(60.0f);
					ImGui::InputInt("Callback", &cbId, 0);
					ImGui::SameLine();
					ImGui::SetNextItemWidth(80.0f);
					ImGui::DragFloat("Thr.", &thr, 0.1f, 0.0f, 1e6f);
					ImGui::SameLine();
					if (ImGui::Button("Apply"))
					{
						if (!sim.setColliderCollisionCallback(ci, static_cast<CollisionCallbackIndex>(std::max(cbId, 0)), thr))
							setStatus("Callback rejected (dead id or non-positive threshold).");
					}

					if (ImGui::Button("Delete collider")) { toDelete = ch; deleteRequested = true; }
					ImGui::TreePop();
				}
				ImGui::PopID();
			}

			if (ImGui::Button("+ Circle"))
			{
				CircleColliderCreateParams p{};
				p.radius = 0.5;
				api.createCircleColliderH(h, p);
			}
			ImGui::SameLine();
			if (ImGui::Button("+ Box"))
			{
				BoxColliderCreateParams p{};
				p.size = Vec2(1, 1);
				api.createBoxColliderH(h, p);
			}
			ImGui::SameLine();
			if (ImGui::Button("+ Capsule"))
			{
				CapsuleColliderCreateParams p{};
				p.length = 1; p.radius = 0.25;
				api.createCapsuleColliderH(h, p);
			}
			ImGui::TextDisabled("Mass/inertia are not updated by collider edits.");

			if (deleteRequested) api.destroyCollider(toDelete);
		}

		if (ImGui::CollapsingHeader("Attachments"))
		{
			const ObjectDenseIndex bi = api.getBodyIndex(h);
			ImGui::Text("Constraints attached: %zu", api.getBodies().attachments[bi].size());
			// TODO(future API): list each attachment (type, other body), edit params, destroy:
			// for (const BodyAttachment& at : api.getBodies().attachments[bi]) { ... sim.constraints().destroyX(at.index); }
		}
	}

	// Constraint type selection and click hint.
	void Editor::drawConstraintPanel(Simulation& sim)
	{
		(void)sim;
		if (ImGui::Combo("Type", &constraintKind, kConstraintNames)) hasFirstAnchor = false;
		if (constraintKind == kMotorKind) ImGui::TextUnformatted("Click the body.");
		else if (hasFirstAnchor) ImGui::TextUnformatted("Click the second body (Esc cancels).");
		else ImGui::TextUnformatted("Click the first body.");
	}

	// Draws selection AABBs and centers, grab line, box select rectangle, create ghost helpers
	// and the pending constraint anchor on the background draw list.
	void Editor::drawOverlay(Simulation& sim)
	{
		ImDrawList* dl = ImGui::GetBackgroundDrawList();
		ObjectAPI& api = sim.objects();
		const BodySoAViewer bv = api.getBodies();
		const ColliderSoAViewer cv = api.getColliders();

		// Selected bodies: collider AABBs + center of mass.
		for (const Handle h : selected)
		{
			const ObjectDenseIndex bi = api.getBodyIndex(h);
			if (bi == Handle::kNull) continue;

			for (const ColliderDenseIndex ci : api.getBodyColliders(bi))
			{
				dl->AddRect(
					toScreen(Vec2(cv.aabbMinX[ci], cv.aabbMaxY[ci])),
					toScreen(Vec2(cv.aabbMaxX[ci], cv.aabbMinY[ci])),
					kSelectColor, 0.0f, 0, 1.5f);
			}
			const ImVec2 c = toScreen(Vec2(bv.worldCenterX[bi], bv.worldCenterY[bi]));
			dl->AddLine(ImVec2(c.x - 5, c.y), ImVec2(c.x + 5, c.y), kSelectColor, 1.5f);
			dl->AddLine(ImVec2(c.x, c.y - 5), ImVec2(c.x, c.y + 5), kSelectColor, 1.5f);
		}

		// Grab lines: from each held anchor to its holder target.
		if (dragMode == DragMode::Holder)
		{
			const auto& holder = sim.getMainBodyHolder();
			for (const auto& held : holder.heldBodies)
			{
				const ObjectDenseIndex bi = api.getBodyIndex(held.body);
				if (bi == Handle::kNull) continue;

				const Real c = bv.rotationCos[bi];
				const Real s = bv.rotationSin[bi];
				const Vec2 o = held.localOffset;
				const Vec2 grab(
					bv.worldCenterX[bi] + o.x * c - o.y * s,
					bv.worldCenterY[bi] + o.x * s + o.y * c);
				const Vec2 target(holder.getPosition().x + held.targetOffset.x, holder.getPosition().y + held.targetOffset.y);
				dl->AddLine(toScreen(grab), toScreen(target), kHintColor, 1.5f);
			}
		}

		// Box select.
		if (boxSelecting)
		{
			dl->AddRectFilled(toScreen(boxStart), toScreen(mouseWorld), IM_COL32(255, 220, 40, 30));
			dl->AddRect(toScreen(boxStart), toScreen(mouseWorld), kSelectColor);
		}

		// Create tool.
		if (tool == Tool::Create)
		{
			if (creating)
			{
				const Vec2 far(2 * createStart.x - mouseWorld.x, 2 * createStart.y - mouseWorld.y);
				dl->AddLine(toScreen(createStart), toScreen(far), kGhostColor, 2.0f);
				dl->AddCircleFilled(toScreen(far), 4.0f, kGhostColor);
			}
			if (ghostVisible && shape == CreateShape::Capsule)
			{
				const Real c = std::cos(rotation), s = std::sin(rotation);
				const Real hl = capsuleLength * 0.5, r = capsuleRadius;
				const Vec2 p0(ghostPos.x - c * hl, ghostPos.y - s * hl);
				const Vec2 p1(ghostPos.x + c * hl, ghostPos.y + s * hl);
				const float rp = static_cast<float>(r / worldPerPixel);
				dl->AddCircle(toScreen(p0), rp, kGhostColor, 24, 2.0f);
				dl->AddCircle(toScreen(p1), rp, kGhostColor, 24, 2.0f);
				dl->AddLine(toScreen(Vec2(p0.x - s * r, p0.y + c * r)), toScreen(Vec2(p1.x - s * r, p1.y + c * r)), kGhostColor, 2.0f);
				dl->AddLine(toScreen(Vec2(p0.x + s * r, p0.y - c * r)), toScreen(Vec2(p1.x + s * r, p1.y - c * r)), kGhostColor, 2.0f);
			}
			if (shape == CreateShape::FreePolygon)
			{
				for (size_t i = 0; i < freePoints.size(); ++i)
				{
					const ImVec2 p = toScreen(freePoints[i]);
					dl->AddCircleFilled(p, 4.0f, kGhostColor);
					if (i > 0) dl->AddLine(toScreen(freePoints[i - 1]), p, kGhostColor, 2.0f);
				}
				if (!freePoints.empty() && !ImGui::GetIO().WantCaptureMouse)
					dl->AddLine(toScreen(freePoints.back()), toScreen(snap(mouseWorld)), kHintColor, 1.0f);
				if (freePoints.size() >= 3)
					dl->AddLine(toScreen(freePoints.back()), toScreen(freePoints.front()), IM_COL32(60, 255, 90, 90), 1.0f);
			}
		}

		// Constraint tool.
		if (tool == Tool::Constraint && hasFirstAnchor)
		{
			const ObjectDenseIndex bi = api.getBodyIndex(firstBody);
			if (bi != Handle::kNull)
			{
				const Real c = bv.rotationCos[bi];
				const Real s = bv.rotationSin[bi];
				const Vec2 w(
					bv.worldCenterX[bi] + firstLocalAnchor.x * c - firstLocalAnchor.y * s,
					bv.worldCenterY[bi] + firstLocalAnchor.x * s + firstLocalAnchor.y * c);
				dl->AddCircleFilled(toScreen(w), 4.0f, kSelectColor);
				dl->AddLine(toScreen(w), toScreen(mouseWorld), kHintColor, 1.5f);
			}
		}
	}
}