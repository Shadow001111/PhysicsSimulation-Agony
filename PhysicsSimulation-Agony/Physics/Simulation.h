#pragma once
#include "GlmTypes.h"
#include "BodySoA.h"
#include "Material.h"

#include <vector>

namespace PS_AGONY
{
	struct BodyPair
	{
		BodyIndex a, b;
	};

	struct BvhNode
	{
		static constexpr uint32_t KD_LEAF_SIZE = 8;
		static constexpr uint32_t INVALID_INDEX = -1;

		Real minX, maxX, minY, maxY; // Merged AABB of all bodies in this subtree.
		uint32_t left, right; // Child node indices; INVALID_INDEX for leaves.
		uint32_t start, end; // Range in kdIndices: [start, end).
	};

	class Simulation
	{
		struct SimulationSettings
		{
			Real updateInterval = 1 / 60.0;

			Vec2 gravity{ 0.0, -9.81 };
		};

		// Bodies SoA.
		BodiesSoA bodies;
		CirclesSoA circles;

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Collisions.
		std::vector<BodyPair> broadPhaseCollisions;

		// Other.
		Real updateTimeAccumulator = 0.0;
	public:
		Simulation() = default;
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		BodyIndex createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex);

		MaterialIndex createMaterial(const Material& material);

		const auto& getBodies() const noexcept { return bodies; }
		const auto& getCircles() const noexcept { return circles; }
	private:
		void physicsStep(Real deltaTime);

		void applyExternalForces(size_t bodyCount, Real deltaTime);

		void integrate(size_t bodyCount, Real deltaTime);

		void boundaryCollisionResolution(size_t bodyCount);

		void buildCircleAABBs();

		void broadPhaseCollisionDetection(size_t bodyCount);

		void narrowPhaseCollisionDetection();

		void resolveCollisions();
	private:
		// Broad phase methods

		void justAABB(size_t bodyCount);
		void sweepAndPruneXAxis(size_t bodyCount);
		void boundVolumeHierarchy(size_t bodyCount);
		void uniformSpaceGrid(size_t bodyCount);
	private:
		// BVH methods

		uint32_t buildBvhNode(
			std::vector<BvhNode>& nodes,
			std::vector<BodyIndex>& indices,
			uint32_t start, uint32_t end);

		void queryBvhPairs(
			const std::vector<BvhNode>& nodes,
			const std::vector<BodyIndex>& indices,
			uint32_t nodeA, uint32_t nodeB);
	};
}
