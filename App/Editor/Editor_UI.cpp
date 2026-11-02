#include "Editor_Internal.h"

namespace PS_AGONY_EDITOR
{
	// Draws the editor window (toolbar, tool panel, inspector in Select tool, status) and the world overlay.
	void Editor::drawUI(Simulation& sim, PS_AGONY_RENDER::SimulationRenderer& renderer)
	{
		(void)renderer;
		cacheConstraintNames(sim);
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
		if (ImGui::Button("Delete")) deleteSelected(sim);
		ImGui::EndDisabled();

		// Help rows: general controls, then Select tool controls (each on its own row).
		hintRow("Pan: RMB/MMB | Zoom: wheel | Tools: 1/2/3 | Snap: G");
		if (tool == Tool::Select)
		{
			hintRow("Select: LMB, Shift+LMB toggles, drag on empty space = box select. Del = delete, Esc = clear.");
			hintRow("Grab: hold LMB on a body. All selected bodies are dragged together.");
			hintRow("Rotate grabbed body: wheel while holding LMB (the wheel does not zoom during a grab).");
		}
	}
}
