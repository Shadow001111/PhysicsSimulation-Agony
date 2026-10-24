#pragma once
#include "ShapeRenderer.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Types/Camera2D.h"
#include "PS_AGONY2D/SimulationImpl/PolygonBuild.h"

#include <span>
#include <optional>

namespace PS_AGONY2D
{
	class Simulation;
}

namespace PS_AGONY_RENDER
{
	using namespace PS_AGONY2D;

	// Defined in SimulationRenderer_Internal.h; only forward-declared here so
	// the public header stays free of private implementation detail.
	struct BodyTransformContext;

	// Must match the framebuffer attachment order.
	namespace RenderAttachments
	{
		inline constexpr GLuint colorIndex = 0;
		inline constexpr GLuint idIndex = 1;
		inline constexpr GLenum colorDrawBuffer = GL_COLOR_ATTACHMENT0 + colorIndex;
	}

	struct RenderOptions
	{
		bool colliderAABBs = false;
		bool broadPhaseDynamicAABBs = false;
		bool broadPhaseStaticAABBs = false;
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
		std::vector<ShapeRenderer::SimplePointInstanceData> simplePointInstanceData; // Position/COM/contact markers, no rotation needed.
		std::vector<ShapeRenderer::BoxInstanceData> boxInstanceData;
		std::vector<glm::vec2> polygonVertexData;
		std::vector<ShapeRenderer::PolygonInstanceData> polygonInstanceData;
		std::vector<DrawArraysIndirectCommand> polygonDrawCommands;
		std::vector<ShapeRenderer::FloatAABB> aabbInstanceData;
		std::vector<ShapeRenderer::LineVertex> lineVertexData; // Reused scratch buffer for springs/joints/rods.

		// Polygon creator preview: rebuilt only when the inputs change.
		struct PolygonPreviewCache
		{
			bool valid = false;
			std::vector<Vec2> sourceVertices;
			std::optional<Vec2> centerOfMass;
			PhysicsGeometry::ConcaveDecompositionMethod concaveMethod{};
			PhysicsGeometry::ConvexDecompositionMethod convexMethod{};
			PhysicsGeometry::PolygonBuildError error = PhysicsGeometry::PolygonBuildError::None;
			std::vector<glm::vec2> vertices;                    // All pieces, original (creator) frame.
			std::vector<DrawArraysIndirectCommand> commands;    // One per convex piece.
		} polygonPreviewCache;

		ShapeRenderer shapeRenderer;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;
		ColliderSoAViewer colliders;

		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		CapsuleSoAViewer capsules;
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
			const RenderOptions& renderOptions,
			GLuint idTextureID
		);

		// Returns nullptr if the preview is valid, otherwise an error message to show the user.
		const char* renderObjectPreview(const void* params, BodyType type);

		Camera2D& getCamera() noexcept { return camera; }

		size_t getMemoryUsage() const noexcept;
		size_t getVideoMemoryUsage() const noexcept;
	private:
		void renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor);
		void queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB);

		// Collect data and render.

		void renderColliders(const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyPositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderCapsuleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);

		void renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix);
		void renderLeafAABBs(std::span<const AABB> aabbs, const glm::vec3& color, const Mat4& viewProjectionMatrix);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderPrismaticJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
		void renderRods(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB);
	};
}