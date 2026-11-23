#pragma once
#include "ShapeRenderer.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Types/Camera2D.h"
#include "PS_AGONY2D/SimulationImpl/PolygonBuild.h"
#include "PS_AGONY2D/Simulation.h"

#include <span>
#include <array>
#include <vector>
#include <optional>

namespace PS_AGONY_RENDER
{
	using namespace PS_AGONY2D;

	// Defined in SimulationRenderer_Internal.h; only forward-declared here so
	// the public header stays free of private implementation detail.
	struct BodyTransformContext;

	template<class T>
	using ConstraintGroupArray = std::array<T, size_t(ConstraintGroup::Unused)>;

	// Must match the framebuffer attachment order.
	namespace RenderAttachments
	{
		inline constexpr GLuint colorIndex = 0;
		inline constexpr GLuint idIndex = 1;
		inline constexpr GLenum colorDrawBuffer = GL_COLOR_ATTACHMENT0 + colorIndex;
	}

	struct RenderOptions
	{
		struct Bodies
		{
			// When false, all body transforms used for rendering this frame snap
			// straight to the current simulation state instead of blending from
			// the previous state via simRenderAlpha. Skips the lerp and the
			// rotation wrap/trunc math - use when the render rate matches the
			// sim rate and interpolation isn't needed.
			bool interpolation = true;

			// Tint per body group, indexed by BodyGroup.
			std::array<glm::vec3, size_t(BodyGroup::Unused)> groupColors = {
				glm::vec3{ 1.0f, 1.0f, 1.0f },
				glm::vec3{ 0.2f, 0.4f, 1.0f },
				glm::vec3{ 0.2f, 0.2f, 0.2f }
			};
		};

		struct Colliders
		{
			bool aabbs = false;

			bool outline = true;
			glm::vec3 outlineColor = { 0.0f, 0.0f, 0.0f };
		
			bool minExtent = false;
			bool maxExtent = false;
			glm::vec3 minExtentColor = { 1.0f, 0.8f, 0.0f };
			glm::vec3 maxExtentColor = { 0.8f, 1.0f, 0.0f };
		};

		struct BroadPhaseAABBs
		{
			std::array<bool, size_t(BroadPhaseInternal::BvhTreeType::COUNT)> enabled{ false };
			std::array<glm::vec3, size_t(BroadPhaseInternal::BvhTreeType::COUNT)> colors = {
				glm::vec3{ 0.0f, 1.0f, 0.0f },
				glm::vec3{ 0.2f, 0.4f, 1.0f },
				glm::vec3{ 0.2f, 0.2f, 0.2f }
			};
		};

		struct Constraints
		{
			// Constraint line and icon tint per constraint group.
			ConstraintGroupArray<glm::vec3> groupColors = []
				{
					ConstraintGroupArray<glm::vec3> colors;
					colors[size_t(ConstraintGroup::Inactive)] = { 0.2f, 0.4f, 1.0f };
					colors[size_t(ConstraintGroup::Sleeping)] = { 0.4f, 0.4f, 0.4f };
					colors[size_t(ConstraintGroup::Active)] = { 0.0f, 1.0f, 1.0f };
					return colors;
				}();

			// Draws an icon at the midpoint of every constraint.
			bool icons = false;

			// Constraint icon side length in world units.
			float iconSize = 0.5f;
		};

		//
		Bodies bodies;
		Colliders colliders;
		BroadPhaseAABBs broadphaseAABBs;
		Constraints constraints;

		bool contactPoints = false;
	};

	inline uint32_t packColor(const glm::vec3& c) noexcept
	{
		const auto to8 = [](float v) -> uint32_t
			{
				return static_cast<uint32_t>(std::fmax(0.0f, std::fmin(1.0f, v)) * 255.0f + 0.5f);
			};
		return (to8(c.x) << 16) | (to8(c.y) << 8) | to8(c.z);
	}

	// TODO: Remove "const Mat4& viewProjectionMatrix". Just add member field.
	// TODO: Remove a lot of tracy scopes.
	// TODO: Have single buffer for draw data.
	class SimulationRenderer
	{
		// Colliders in camera view, bucketed by (shape, body group).
		struct FoundColliders
		{
			static constexpr size_t groupCount = size_t(BodyGroup::Unused);
			static constexpr size_t shapeCount = size_t(ColliderShapeType::COUNT);
			static constexpr size_t binCount = shapeCount * groupCount;

			std::vector<ObjectDenseIndex> raw;     // Query output, contiguous per group.
			std::vector<ObjectDenseIndex> sorted;  // Ordered by shape, then group.
			std::vector<uint8_t> shapes;           // Scratch: shape per raw entry.
			std::array<uint32_t, binCount + 1> offsets{};     // Bin = shape * groupCount + group.
			std::array<uint32_t, groupCount + 1> groupBegin{}; // Group ranges in raw.

			void build(const ColliderSoAViewer& colliders);

			std::span<const ObjectDenseIndex> get(ColliderShapeType shape) const noexcept
			{
				const size_t b = size_t(shape) * groupCount;
				return { sorted.data() + offsets[b], offsets[b + groupCount] - offsets[b] };
			}

			// All colliders of a shape plus per-group boundaries relative to that span.
			struct ShapeRange
			{
				std::span<const ObjectDenseIndex> colliders;
				std::array<uint32_t, groupCount + 1> groupBegin;
			};

			ShapeRange getRange(ColliderShapeType shape) const noexcept
			{
				const size_t b = size_t(shape) * groupCount;
				ShapeRange r;
				r.colliders = get(shape);
				for (size_t g = 0; g <= groupCount; g++)
					r.groupBegin[g] = offsets[b + g] - offsets[b];
				return r;
			}

			std::span<const ObjectDenseIndex> get(ColliderShapeType shape, BodyGroup group) const noexcept
			{
				const size_t b = size_t(shape) * groupCount + size_t(group);
				return { sorted.data() + offsets[b], offsets[b + 1] - offsets[b] };
			}
		};

		using GroupColors = std::array<uint32_t, FoundColliders::groupCount>; // Packed 0xRRGGBB per BodyGroup.
		using ConstraintGroupColors = ConstraintGroupArray<uint32_t>; // Packed 0xRRGGBB per ConstraintGroup.

		// Resources.
		FoundColliders foundColliders;

		std::vector<AABB> queriedAABBs;

		// CPU-side per-frame draw data, owned here; ShapeRenderer only owns GPU resources.
		std::vector<ShapeRenderer::CircleInstanceData> circleInstanceData;
		std::vector<ShapeRenderer::SimplePointInstanceData> simplePointInstanceData; // Position/COM/contact markers, no rotation needed.
		std::vector<ShapeRenderer::BoxInstanceData> boxInstanceData;
		std::vector<glm::vec2> polygonVertexData;
		std::vector<ShapeRenderer::PolygonInstanceData> polygonInstanceData;
		std::vector<DrawArraysIndirectCommand> polygonDrawCommands;
		std::vector<ShapeRenderer::FloatAABB> aabbInstanceData;
		std::vector<ShapeRenderer::LineVertex> lineVertexData; // Reused scratch buffer for all constraint lines.
		std::vector<ShapeRenderer::ConstraintIconInstanceData> constraintIconData;
		std::vector<ShapeRenderer::HollowCircleInstanceData> hollowCircleData;

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
		const char* renderObjectPreview(const void* params, ColliderShapeType type);

		Camera2D& getCamera() noexcept { return camera; }

		size_t getMemoryUsage() const noexcept;
		size_t getVideoMemoryUsage() const noexcept;
	private:
		void renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor);
		void queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB);

		// Collect data and render.

		void renderColliders(const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const GroupColors& groupColors);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyPositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderCircleColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderBoxColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderCapsuleColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);
		void renderPolygonColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);

		void renderColliderAABBs(std::span<const ObjectDenseIndex> givenColliders, const Mat4& viewProjectionMatrix);
		void renderLeafAABBs(std::span<const AABB> aabbs, const glm::vec3& color, const Mat4& viewProjectionMatrix);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderColliderExtents(std::span<const ObjectDenseIndex> givenColliders, const glm::vec3& color, const Real* extentPtr, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext);

		void renderConstraints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB, const ConstraintGroupColors& groupColors, float iconSize);
	};
}