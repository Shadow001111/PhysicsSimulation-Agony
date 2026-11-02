#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
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

		if (constraintKind == static_cast<int>(ConstraintType::Motor))
		{
			createConstraint(sim, body, local, body, local);
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
		createConstraint(sim, firstBody, firstLocalAnchor, body, local);
		hasFirstAnchor = false;
	}

	// Creates the constraint of type constraintKind. Anchors are local to each body's center of mass.
	// Motor uses body a only. Spring rest length = current anchor distance. Axis = A->B anchor direction.
	void Editor::createConstraint(Simulation& sim, Handle a, Vec2 localA, Handle b, Vec2 localB)
	{
		ConstraintAPI& cs = sim.constraints();
		const ConstraintType type = static_cast<ConstraintType>(constraintKind);
		std::optional<Handle> res;

		if (type == ConstraintType::Motor)
		{
			MotorConstraintSystem::CreateParams p{};
			p.targetSpeed = static_cast<Real>(motorSpeed);
			p.maxTorque = static_cast<Real>(motorTorque);
			res = cs.createMotorH(a, p);
		}
		else
		{
			ObjectAPI& api = sim.objects();
			const ObjectDenseIndex ia = api.getBodyIndex(a), ib = api.getBodyIndex(b);
			if (ia == Handle::kNull || ib == Handle::kNull) { setStatus("Body no longer exists."); return; }
			if (ia == ib) { setStatus("Pick two different bodies."); return; }

			const BodySoAViewer bv = api.getBodies();
			const Vec2 d = localToWorld(bv, ib, localB) - localToWorld(bv, ia, localA);
			const Real len = std::sqrt(d.x * d.x + d.y * d.y);
			// Branchless: local axis of d, or (1, 0) when the anchors coincide.
			const Real invLen = Real(1) / std::fmax(len, Real(1e-6));
			const Vec2 dirLocal = worldDirToLocal(bv, ia, d * invLen);
			const Real valid = static_cast<Real>(len > Real(1e-6));
			const Vec2 axis(dirLocal.x * valid + (Real(1) - valid), dirLocal.y * valid);

			const Real jk = static_cast<Real>(jointStiffness), jd = static_cast<Real>(jointDamping);
			const Real lo = static_cast<Real>(std::fmin(limitLower, limitUpper));
			const Real hi = static_cast<Real>(std::fmax(limitLower, limitUpper));

			switch (type)
			{
			case ConstraintType::Spring:
			{
				SpringConstraintSystem::CreateParams p{};
				p.localAnchorA = localA; p.localAnchorB = localB;
				p.restLength = len;
				p.stiffness = static_cast<Real>(springStiffness);
				p.damping = static_cast<Real>(springDamping);
				res = cs.createSpringH(a, b, p);
				break;
			}
			case ConstraintType::RevoluteJoint:
			{
				RevoluteJointConstraintSystem::CreateParams p{};
				p.localAnchorA = localA; p.localAnchorB = localB;
				p.stiffness = jk; p.damping = jd;
				res = cs.createRevoluteJointH(a, b, p);
				break;
			}
			case ConstraintType::PrismaticJoint:
			{
				PrismaticJointConstraintSystem::CreateParams p{};
				p.localAnchorA = localA; p.localAnchorB = localB;
				p.localAxisA = axis;
				p.stiffness = jk; p.damping = jd;
				p.lowerLimit = lo; p.upperLimit = hi; p.enableLimit = limitEnabled;
				res = cs.createPrismaticJointH(a, b, p);
				break;
			}
			case ConstraintType::SliderJoint:
			{
				SliderJointConstraintSystem::CreateParams p{};
				p.localAnchorA = localA; p.localAnchorB = localB;
				p.localAxisA = axis;
				p.stiffness = jk; p.damping = jd;
				p.lowerLimit = lo; p.upperLimit = hi; p.enableLimit = limitEnabled;
				res = cs.createSliderJointH(a, b, p);
				break;
			}
			default: break;
			}
		}
		setStatus(res ? "Constraint created." : "Constraint creation failed (stale body).");
	}

	// Edits one constraint through the deferred setters. Reads stored state each frame.
	void Editor::drawConstraintEditor(Simulation& sim, ConstraintType type, Handle h)
	{
		ConstraintAPI& cs = sim.constraints();
		ObjectAPI& api = sim.objects();
		Vec2 v{};
		Real r{};

		auto bodies = [&](Handle a, Handle b)
			{
				ImGui::Text("Bodies: #%u - #%u", static_cast<unsigned>(api.getBodyIndex(a)), static_cast<unsigned>(api.getBodyIndex(b)));
			};
		auto limits = [&](auto& d, auto setLimits, auto setEnable)
			{
				bool en = d.enableLimit;
				if (ImGui::Checkbox("Limit", &en)) setEnable(en);
				float lim[2] = { static_cast<float>(d.lowerLimit), static_cast<float>(d.upperLimit) };
				if (ImGui::DragFloat2("Lower / upper", lim, 0.01f)) setLimits(static_cast<Real>(lim[0]), static_cast<Real>(lim[1]));
			};

		switch (type)
		{
		case ConstraintType::Spring:
		{
			const auto d = cs.getSpringDataH(h);
			if (!d) return;
			bodies(d->bodyHandleA, d->bodyHandleB);
			if (dragVec2("Anchor A", d->localAnchorA, v)) cs.setSpringLocalAnchorA(h, v);
			if (dragVec2("Anchor B", d->localAnchorB, v)) cs.setSpringLocalAnchorB(h, v);
			if (dragReal("Rest length", d->restLength, r, 0.01f, 0.0f, 1e4f)) cs.setSpringRestLength(h, r);
			if (dragReal("Stiffness", d->stiffness, r, 0.5f, 0.0f, 1e8f)) cs.setSpringStiffness(h, r);
			if (dragReal("Damping", d->damping, r, 0.05f, 0.0f, 1e6f)) cs.setSpringDamping(h, r);
			break;
		}
		case ConstraintType::RevoluteJoint:
		{
			const auto d = cs.getRevoluteJointDataH(h);
			if (!d) return;
			bodies(d->bodyHandleA, d->bodyHandleB);
			if (dragVec2("Anchor A", d->localAnchorA, v)) cs.setRevoluteJointLocalAnchorA(h, v);
			if (dragVec2("Anchor B", d->localAnchorB, v)) cs.setRevoluteJointLocalAnchorB(h, v);
			if (dragReal("Stiffness", d->stiffness, r, 0.5f, 0.0f, 1e8f)) cs.setRevoluteJointStiffness(h, r);
			if (dragReal("Damping", d->damping, r, 0.05f, 0.0f, 1e6f)) cs.setRevoluteJointDamping(h, r);
			break;
		}
		case ConstraintType::PrismaticJoint:
		{
			const auto d = cs.getPrismaticJointDataH(h);
			if (!d) return;
			bodies(d->bodyHandleA, d->bodyHandleB);
			if (dragVec2("Anchor A", d->localAnchorA, v)) cs.setPrismaticJointLocalAnchorA(h, v);
			if (dragVec2("Anchor B", d->localAnchorB, v)) cs.setPrismaticJointLocalAnchorB(h, v);
			if (dragVec2("Axis A", d->localAxisA, v, 0.005f)) cs.setPrismaticJointLocalAxisA(h, v);
			if (dragReal("Stiffness", d->stiffness, r, 0.5f, 0.0f, 1e8f)) cs.setPrismaticJointStiffness(h, r);
			if (dragReal("Damping", d->damping, r, 0.05f, 0.0f, 1e6f)) cs.setPrismaticJointDamping(h, r);
			limits(*d,
				[&](Real lo, Real hi) { cs.setPrismaticJointLimits(h, lo, hi); },
				[&](bool en) { cs.setPrismaticJointEnableLimit(h, en); });
			if (ImGui::Button("Reset reference angle")) cs.resetPrismaticJointReferenceAngle(h);
			break;
		}
		case ConstraintType::SliderJoint:
		{
			const auto d = cs.getSliderJointDataH(h);
			if (!d) return;
			bodies(d->bodyHandleA, d->bodyHandleB);
			if (dragVec2("Anchor A", d->localAnchorA, v)) cs.setSliderJointLocalAnchorA(h, v);
			if (dragVec2("Anchor B", d->localAnchorB, v)) cs.setSliderJointLocalAnchorB(h, v);
			if (dragVec2("Axis A", d->localAxisA, v, 0.005f)) cs.setSliderJointLocalAxisA(h, v);
			if (dragReal("Stiffness", d->stiffness, r, 0.5f, 0.0f, 1e8f)) cs.setSliderJointStiffness(h, r);
			if (dragReal("Damping", d->damping, r, 0.05f, 0.0f, 1e6f)) cs.setSliderJointDamping(h, r);
			limits(*d,
				[&](Real lo, Real hi) { cs.setSliderJointLimits(h, lo, hi); },
				[&](bool en) { cs.setSliderJointEnableLimit(h, en); });
			break;
		}
		case ConstraintType::Motor:
		{
			const auto d = cs.getMotorDataH(h);
			if (!d) return;
			if (dragReal("Target speed", d->targetSpeed, r, 0.05f, -1e4f, 1e4f)) cs.setMotorTargetSpeed(h, r);
			if (dragReal("Max torque", d->maxTorque, r, 0.5f, 0.0f, 1e8f)) cs.setMotorMaxTorque(h, r);
			ImGui::Text("Impulse: %.3f", static_cast<double>(d->accumulatedImpulse));
			break;
		}
		default: break;
		}
	}

	// Constraint type selection, default parameters and click hint.
	void Editor::drawConstraintPanel(Simulation& sim)
	{
		(void)sim;
		if (ImGui::Combo("Type", &constraintKind, constraintCombo.c_str())) hasFirstAnchor = false;

		const ConstraintType type = static_cast<ConstraintType>(constraintKind);
		switch (type)
		{
		case ConstraintType::Spring:
			ImGui::DragFloat("Stiffness", &springStiffness, 0.5f, 0.0f, 1e8f);
			ImGui::DragFloat("Damping", &springDamping, 0.05f, 0.0f, 1e6f);
			ImGui::TextDisabled("Rest length = distance between the clicked points.");
			break;
		case ConstraintType::RevoluteJoint:
			ImGui::DragFloat("Stiffness", &jointStiffness, 0.5f, 0.0f, 1e8f);
			ImGui::DragFloat("Damping", &jointDamping, 0.05f, 0.0f, 1e6f);
			break;
		case ConstraintType::PrismaticJoint:
		case ConstraintType::SliderJoint:
			ImGui::DragFloat("Stiffness", &jointStiffness, 0.5f, 0.0f, 1e8f);
			ImGui::DragFloat("Damping", &jointDamping, 0.05f, 0.0f, 1e6f);
			ImGui::Checkbox("Limit", &limitEnabled);
			ImGui::BeginDisabled(!limitEnabled);
			ImGui::DragFloat("Lower", &limitLower, 0.01f);
			ImGui::DragFloat("Upper", &limitUpper, 0.01f);
			ImGui::EndDisabled();
			ImGui::TextDisabled("Axis = direction from the first to the second point.");
			break;
		case ConstraintType::Motor:
			ImGui::DragFloat("Target speed", &motorSpeed, 0.05f);
			ImGui::DragFloat("Max torque", &motorTorque, 0.5f, 0.0f, 1e8f);
			break;
		default: break;
		}

		if (constraintKind == static_cast<int>(ConstraintType::Motor)) ImGui::TextUnformatted("Click the body.");
		else if (hasFirstAnchor) ImGui::TextUnformatted("Click the second body (Esc cancels).");
		else ImGui::TextUnformatted("Click the first body.");
	}
}
