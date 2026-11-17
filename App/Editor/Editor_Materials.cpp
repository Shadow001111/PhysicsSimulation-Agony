#include "Editor_Internal.h"

#include <cstdio>

namespace PS_AGONY_EDITOR
{
	// Combo over alive materials. A dead current index is shown as such but stays selected until changed.
	bool Editor::materialCombo(Simulation& sim, const char* label, int& materialIndex)
	{
		const MaterialIndex current = static_cast<MaterialIndex>(std::max(materialIndex, 0));
		char preview[32];
		std::snprintf(preview, sizeof(preview), sim.isMaterialAlive(current) ? "#%d" : "#%d (dead)", materialIndex);

		bool changed = false;
		if (ImGui::BeginCombo(label, preview))
		{
			const int count = static_cast<int>(sim.getMaterialCount());
			for (int i = 0; i < count; i++)
			{
				if (!sim.isMaterialAlive(static_cast<MaterialIndex>(i))) continue;

				char item[32];
				std::snprintf(item, sizeof(item), i == 0 ? "#0 (default)" : "#%d", i);
				const bool isSelected = i == materialIndex;
				if (ImGui::Selectable(item, isSelected)) { materialIndex = i; changed = true; }
				if (isSelected) ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		return changed;
	}

	// Material list: edit values, create, delete. Edits wake the affected islands (Simulation::setMaterial).
	void Editor::drawMaterials(Simulation& sim)
	{
		const int count = static_cast<int>(sim.getMaterialCount());
		int toDestroy = -1;
		for (int i = 0; i < count; i++)
		{
			const MaterialIndex index = static_cast<MaterialIndex>(i);
			if (!sim.isMaterialAlive(index)) continue;

			const Material cur = sim.getMaterial(index);
			Material m = cur;

			ImGui::PushID(i);
			if (ImGui::TreeNodeEx("##m", ImGuiTreeNodeFlags_DefaultOpen, "Material #%d%s", i, i == 0 ? " (default)" : ""))
			{
				bool changed = false;
				changed |= dragReal("Elasticity", cur.elasticity, m.elasticity, 0.01f, 0.0f, 1.0f);
				changed |= dragReal("Static friction", cur.staticFriction, m.staticFriction, 0.01f, 0.0f, 10.0f);
				changed |= dragReal("Dynamic friction", cur.dynamicFriction, m.dynamicFriction, 0.01f, 0.0f, 10.0f);
				if (changed) sim.setMaterial(index, m);

				if (i != 0 && ImGui::Button("Delete")) toDestroy = i;
				ImGui::TreePop();
			}
			ImGui::PopID();
		}

		if (ImGui::Button("New material")) sim.createMaterial(Material{});
		if (toDestroy >= 0) sim.destroyMaterial(static_cast<MaterialIndex>(toDestroy));
	}
}