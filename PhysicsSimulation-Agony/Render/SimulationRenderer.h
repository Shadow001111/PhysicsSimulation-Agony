#pragma once
#include "ShapeRenderer.h"

#include "Physics/Camera2D.h"
#include "Physics/ObjectSoA.h"

namespace PS_AGONY
{
	class Simulation;
}

namespace Render
{
	using namespace PS_AGONY;

	// Defined in SimulationRenderer_Internal.h; only forward-declared here so
	// the public header stays free of private implementation detail.
	struct BodyTransformContext;

	struct RenderOptions
	{
		bool colliderAABBs = false;
		bool broadPhaseAABBs = false;
		bool contactPoints = false;
		bool outlineColliders = true;
		glm::vec3 outlineColor = { 0.0f, 0.0f, 0.0f };

		// When false, all body transforms used for rendering this frame snap
		// straight to the current simulation state instead of blending from
		// the previous state via simRenderAlpha. Skips the lerp and the
		// rotation wrap/trunc math - use when the render rate matches the
		// sim rate and interpolation isn't needed.
		bool interpolation = true;
	};

	class SimulationRenderer
	{
		// Resources.
		std::vector<ColliderIndex> foundColliders;
		std::vector<ColliderIndex> foundColliderShapes[(size_t)BodyType::COUNT];

		std::vector<AABB> queriedAABBs;

		// CPU-side per-frame draw data, owned here; ShapeRenderer only owns GPU resources.
		std::vector<ShapeRenderer::CircleInstanceData> circleInstanceData;
		std::vector<ShapeRenderer::BoxInstanceData> boxInstanceData;
		std::vector<glm::vec2> polygonVertexData;
		std::vector<ShapeRenderer::PolygonInstanceData> polygonInstanceData;
		std::vector<DrawArraysIndirectCommand> polygonDrawCommands;
		std::vector<ShapeRenderer::FloatAABB> aabbInstanceData;
		std::vector<ShapeRenderer::LineVertex> lineVertexData; // Reused scratch buffer for springs/joints/rods.

		ShapeRenderer shapeRenderer;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;
		ColliderSoAViewer colliders;

		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		PolygonSoAViewer polygons;
	public:
		SimulationRenderer() = default;
		~SimulationRenderer() = default;
		SimulationRenderer(const SimulationRenderer&) = delete;
		SimulationRenderer& operator=(const SimulationRenderer&) = delete;
		SimulationRenderer(SimulationRenderer&&) = delete;
		SimulationRenderer& operator=(SimulationRenderer&&) = delete;

		void init();

		void renderSimulation(
			const Simulation& simulation,
			Real simRenderAlpha,
			const AABB& cameraAABB,
			const RenderOptions& renderOptions
		);

		void renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor);

		void renderObjectPreview(const void* params, BodyType type);

		Camera2D& getCamera() noexcept { return camera; }

		size_t getMemoryUsage() const noexcept;
		size_t getVideoMemoryUsage() const noexcept;
	private:
		void queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB);

		// Collect data and render.

		void renderColliders(const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyPositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);

		void renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderPrismaticJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderRods(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
	};
}