#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
	// Draws selection AABBs and centers, box select rectangle, create ghost helpers
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
			if (bi == Handle::NULL_DENSE_INDEX) continue;

			for (const ObjectDenseIndex ci : api.getBodyColliders(bi))
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

		// Constraints of selected bodies: anchors and links.
		{
			ConstraintAPI& cs = sim.constraints();
			constexpr ImU32 kConstraintColor = IM_COL32(80, 200, 255, 255);

			auto link = [&](Handle ha, Vec2 la, Handle hb, Vec2 lb)
				{
					const ObjectDenseIndex ia = api.getBodyIndex(ha), ib = api.getBodyIndex(hb);
					if (ia == Handle::NULL_DENSE_INDEX || ib == Handle::NULL_DENSE_INDEX) return;
					const ImVec2 p = toScreen(localToWorld(bv, ia, la));
					const ImVec2 q = toScreen(localToWorld(bv, ib, lb));
					dl->AddLine(p, q, kConstraintColor, 1.5f);
					dl->AddCircleFilled(p, 3.5f, kConstraintColor);
					dl->AddCircleFilled(q, 3.5f, kConstraintColor);
				};

			for (const Handle h : selected)
			{
				const ObjectDenseIndex bi = api.getBodyIndex(h);
				if (bi == Handle::NULL_DENSE_INDEX) continue;

				for (const BodyAttachment& at : bv.attachments[bi])
				{
					const uint32_t i = at.objectIndex;
					switch (at.type)
					{
					case ConstraintType::Spring:
						if (const auto d = cs.getSpringData(i)) link(d->bodyHandleA, d->localAnchorA, d->bodyHandleB, d->localAnchorB);
						break;
					case ConstraintType::RevoluteJoint:
						if (const auto d = cs.getRevoluteJointData(i)) link(d->bodyHandleA, d->localAnchorA, d->bodyHandleB, d->localAnchorB);
						break;
					case ConstraintType::PrismaticJoint:
						if (const auto d = cs.getPrismaticJointData(i)) link(d->bodyHandleA, d->localAnchorA, d->bodyHandleB, d->localAnchorB);
						break;
					case ConstraintType::SliderJoint:
						if (const auto d = cs.getSliderJointData(i)) link(d->bodyHandleA, d->localAnchorA, d->bodyHandleB, d->localAnchorB);
						break;
					case ConstraintType::Motor:
						dl->AddCircle(toScreen(Vec2(bv.worldCenterX[bi], bv.worldCenterY[bi])), 9.0f, kConstraintColor, 16, 1.5f);
						break;
					default: break;
					}
				}
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
			if (creating && !isStatic)
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
				for (size_t i = 0; i < freePoints.size(); i++)
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
			if (bi != Handle::NULL_DENSE_INDEX)
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
