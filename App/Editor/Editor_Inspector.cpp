#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
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

			ImGui::BeginDisabled(bd->isStatic);
			float vel[2] = { static_cast<float>(bd->velocity.x), static_cast<float>(bd->velocity.y) };
			if (ImGui::DragFloat2("Velocity", vel, 0.05f)) api.setBodyVelocity(h, Vec2(vel[0], vel[1]));

			float av = static_cast<float>(bd->angularVelocity);
			if (ImGui::DragFloat("Angular velocity", &av, 0.05f)) api.setBodyAngularVelocity(h, av);
			ImGui::EndDisabled();

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
			for (const ObjectDenseIndex ci : api.getBodyColliders(bi)) colliderScratch.push_back(ci);

			Handle toDelete{};
			bool deleteRequested = false;

			for (const ObjectDenseIndex ci : colliderScratch)
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

					if (cd->shapeType == ColliderShapeType::Circle)
					{
						float r = static_cast<float>(cd->radius);
						if (ImGui::DragFloat("Radius", &r, 0.01f, 0.001f, 1000.0f)) api.setCircleRadius(ch, r);
					}
					else if (cd->shapeType == ColliderShapeType::Box)
					{
						float sz[2] = { static_cast<float>(cd->halfExtents.x * 2), static_cast<float>(cd->halfExtents.y * 2) };
						if (ImGui::DragFloat2("Size", sz, 0.01f, 0.001f, 1000.0f)) api.setBoxSize(ch, Vec2(sz[0], sz[1]));
					}
					else if (cd->shapeType == ColliderShapeType::Capsule)
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
			ConstraintAPI& cs = sim.constraints();

			// Snapshot as handles: destroying swaps dense indices.
			attachScratch.clear();
			for (const BodyAttachment& at : api.getBodies().attachments[bi])
			{
				Handle ch{};
				switch (at.type)
				{
				case ConstraintType::Spring: ch = cs.getSpringHandle(at.objectIndex); break;
				case ConstraintType::RevoluteJoint: ch = cs.getRevoluteJointHandle(at.objectIndex); break;
				case ConstraintType::PrismaticJoint: ch = cs.getPrismaticJointHandle(at.objectIndex); break;
				case ConstraintType::SliderJoint: ch = cs.getSliderJointHandle(at.objectIndex); break;
				case ConstraintType::Motor: ch = cs.getMotorHandle(at.objectIndex); break;
				default: continue;
				}
				attachScratch.push_back({ at.type, ch });
			}

			ImGui::Text("Constraints attached: %zu", attachScratch.size());

			ConstraintRef toDestroy{ ConstraintType::COUNT, Handle{} };
			for (size_t i = 0; i < attachScratch.size(); i++)
			{
				const ConstraintRef ref = attachScratch[i];
				ImGui::PushID(static_cast<int>(i));
				if (ImGui::TreeNodeEx("##k", ImGuiTreeNodeFlags_DefaultOpen, "%s", constraintNames[static_cast<size_t>(ref.type)].c_str()))
				{
					drawConstraintEditor(sim, ref.type, ref.handle);
					if (ImGui::Button("Destroy")) toDestroy = ref;
					ImGui::TreePop();
				}
				ImGui::PopID();
			}

			switch (toDestroy.type)
			{
			case ConstraintType::Spring: cs.destroySpring(toDestroy.handle); break;
			case ConstraintType::RevoluteJoint: cs.destroyRevoluteJoint(toDestroy.handle); break;
			case ConstraintType::PrismaticJoint: cs.destroyPrismaticJoint(toDestroy.handle); break;
			case ConstraintType::SliderJoint: cs.destroySliderJoint(toDestroy.handle); break;
			case ConstraintType::Motor: cs.destroyMotor(toDestroy.handle); break;
			default: break;
			}
		}
	}
}
