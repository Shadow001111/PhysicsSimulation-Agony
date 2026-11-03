#pragma once
#include "PS_AGONY2D/Simulation.h"
#include "PS_AGONY2D/Types/Types.h"

#include "Render/SimulationRenderer.h"

#include <string>
#include <vector>

struct ImVec2;

namespace PS_AGONY_EDITOR
{
	using namespace PS_AGONY2D;

	// Sandbox editor. Talks to the simulation only through ObjectAPI / Simulation public API, selection is stored as handles.
	// Per frame: ImGui::NewFrame -> update() -> simulation.update() -> drawUI() -> renderSimulation() -> renderPreview().
	class Editor
	{
	public:
		enum class Tool : uint8_t { Select, Create, Constraint };
		enum class CreateShape : uint8_t { Circle, Box, Capsule, RegularPolygon, FreePolygon };
		enum class DragMode : uint8_t { None, Holder };

		void update(Simulation& sim, PS_AGONY_RENDER::SimulationRenderer& renderer, Real deltaTime);
		void drawUI(Simulation& sim, PS_AGONY_RENDER::SimulationRenderer& renderer);
		void renderPreview(PS_AGONY_RENDER::SimulationRenderer& renderer);
	private:
		Tool tool = Tool::Select;

		// Frame cache.
		Vec2 mouseWorld{ 0, 0 };
		Real worldPerPixel = 0.01;
		Mat4 viewProjection{};
		std::vector<ObjectDenseIndex> foundScratch;
		std::vector<ObjectDenseIndex> colliderScratch;

		// Snap.
		bool snapEnabled = false;
		float gridSize = 0.5f;

		// Selection.
		std::vector<Handle> selected;
		Handle activeCollider{};

		// Drag / box select.
		DragMode dragMode = DragMode::None;
		Handle dragBody{};
		bool boxSelecting = false;
		Vec2 boxStart{ 0, 0 };

		// Create tool.
		CreateShape shape = CreateShape::Circle;
		float radius = 0.5f;
		float size[2] = { 1.0f, 1.0f };
		float capsuleLength = 1.0f;
		float capsuleRadius = 0.25f;
		int sides = 5;
		float mass = 1.0f;
		bool isStatic = false;
		float rotation = 0.0f;
		float angularVelocity = 0.0f;
		float launchScale = 3.0f;
		int materialIndex = 0;
		bool creating = false;
		bool ghostVisible = false;
		Vec2 createStart{ 0, 0 };
		Vec2 ghostPos{ 0, 0 };
		Vec2 ghostVel{ 0, 0 };
		std::vector<Vec2> freePoints;
		std::vector<Vec2> polyScratch;
		CircleCreateParams circleP;
		BoxCreateParams boxP;
		PolygonCreateParams polyP;
		const char* previewError = nullptr;

		// Constraint tool.
		int constraintKind = 0;
		bool hasFirstAnchor = false;
		Handle firstBody{};
		Vec2 firstLocalAnchor{ 0, 0 };
		// Defaults for new constraints. Kind index == ConstraintType.
		float springStiffness = 100.0f;
		float springDamping = 1.0f;
		float jointStiffness = 0.0f;
		float jointDamping = 0.0f;
		float motorSpeed = 1.0f;
		float motorTorque = 100.0f;
		bool limitEnabled = false;
		float limitLower = -1.0f;
		float limitUpper = 1.0f;

		struct ConstraintRef { ConstraintType type; Handle handle; };
		std::vector<ConstraintRef> attachScratch;

		// Inspector.
		float lastDynamicMass = 1.0f;
		float density = 1.0f;

		// Constraint system names, cached once from the simulation.
		std::string constraintNames[static_cast<size_t>(ConstraintType::COUNT)];
		std::string constraintCombo; // Null-separated, for ImGui::Combo.
		bool namesCached = false;

		// Feedback.
		std::string status;
		float statusTimer = 0.0f;

		// Helpers.
		Vec2 snap(Vec2 p) const noexcept;
		void setStatus(const char* text);
		void cacheConstraintNames(Simulation& sim);
		Vec2 screenToWorld(PS_AGONY2D::Camera2D& cam, float px, float py) const;
		ImVec2 toScreen(Vec2 world) const;
		int findSelected(const ObjectAPI& api, Handle body) const;
		void select(const ObjectAPI& api, Handle body, bool additive);

		bool pickCollider(Simulation& sim, Vec2 p, ObjectDenseIndex& out);
		void beginDrag(Simulation& sim, Handle body, Vec2 grabWorld);
		void updateDrag(Simulation& sim, Real dt);
		void endDrag(Simulation& sim);
		bool holdBody(Simulation& sim, Handle body, Vec2 anchorWorld, Vec2 grabWorld);
		void finishBoxSelect(Simulation& sim, Vec2 end, bool additive);

		void updateSelect(Simulation& sim, bool overUI, Real dt);
		void updateCreate(Simulation& sim, bool overUI);
		void updateConstraint(Simulation& sim, bool overUI);
		void rebuildGhost();
		void spawn(Simulation& sim);
		void spawnFreePolygon(Simulation& sim);
		void deleteSelected(Simulation& sim);
		void createConstraint(Simulation& sim, Handle a, Vec2 localA, Handle b, Vec2 localB);
		void drawConstraintEditor(Simulation& sim, ConstraintType type, Handle h);

		void drawToolbar(Simulation& sim);
		void drawCreatePanel();
		void drawInspector(Simulation& sim);
		void drawConstraintPanel(Simulation& sim);
		void drawOverlay(Simulation& sim);
	};
}