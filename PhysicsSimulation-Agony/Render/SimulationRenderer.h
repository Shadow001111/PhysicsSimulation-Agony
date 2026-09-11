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

	struct RenderOptions
	{
		bool colliderAABBs = false;
		bool broadPhaseAABBs = false;
		bool contactPoints = false;
	};

	class SimulationRenderer
	{
		// Resources.
		std::vector<ColliderIndex> foundColliders;
		std::vector<ColliderIndex> foundColliderShapes[(size_t)BodyType::COUNT];

		std::vector<AABB> queriedAABBs;

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

		void renderObjectPreview(const void* params, BodyType type);

		Camera2D& getCamera() noexcept { return camera; }

		size_t getMemoryUsage() const noexcept;
		size_t getVideoMemoryUsage() const noexcept;
	private:
		void queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB);

		// Collect data and render.

		void renderColliders(const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix);
		void renderBodyPositions(const Mat4& viewProjectionMatrix);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix);

		void renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);

		void renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
	};
}