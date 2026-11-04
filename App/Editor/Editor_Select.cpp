#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
	// Finds the collider under p within a pixel tolerance. BVH circle query, then an exact test for
	// circle/box/capsule (polygons: AABB hit only). Smallest AABB area wins, so small bodies on large ones stay pickable.
	// Returns false if nothing was hit.
	bool Editor::pickCollider(Simulation& sim, Vec2 p, ObjectDenseIndex& out)
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

		for (const ObjectDenseIndex ci : foundScratch)
		{
			const Real dx = p.x - cv.worldPosX[ci];
			const Real dy = p.y - cv.worldPosY[ci];
			const Real c = cv.worldRotationCos[ci];
			const Real s = cv.worldRotationSin[ci];
			const Real lx = dx * c + dy * s;
			const Real ly = -dx * s + dy * c;

			bool hit = true; // Polygons: broad-phase AABB hit only (TODO: exact test once PolygonGeometry is known here).
			const ColliderShapeType type = cv.shapeType[ci];
			if (type == ColliderShapeType::Circle || type == ColliderShapeType::Box || type == ColliderShapeType::Capsule)
			{
				const auto data = api.getColliderDataH(api.getColliderHandle(ci));
				if (!data) continue;

				if (type == ColliderShapeType::Circle)
				{
					const Real r = data->radius + tol;
					hit = lx * lx + ly * ly <= r * r;
				}
				else if (type == ColliderShapeType::Box)
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
		if (bi == Handle::NULL_DENSE_INDEX) return false;

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
		if (primary == Handle::NULL_DENSE_INDEX) return;

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
			if (bi == Handle::NULL_DENSE_INDEX || bi == primary) continue;
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
		for (const ObjectDenseIndex ci : foundScratch)
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

		ObjectDenseIndex ci;
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
}
