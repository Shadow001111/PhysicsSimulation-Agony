#pragma once
#include "GlmTypes.h"
#include "BodySoA.h"
#include "Material.h"

#include "BroadPhaseCollisionDetector.h"
#include "NarrowPhaseCollisionDetector.h"

#include <vector>

namespace PS_AGONY
{
	class Simulation
	{
		struct SimulationSettings
		{
			//
			Real updateInterval = 1 / 300.0;
			uint32_t collisionSolvingIterations = 8;

			// Environment.
			Real timeScale = 1.0;
			Vec2 gravity{ 0.0, -9.81 };

			// Baumgarte stabilization.
			const Real positionCorrectionPercent = 0.8;
			const Real slop = 0.0;
		};

		// Bodies SoA.
		BodySoA bodies;
		CircleSoA circles;

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Collisions.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;

		// Other.
		Real updateTimeAccumulator = 0.0;
	public:
		Simulation();
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		BodyIndex createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex);

		MaterialIndex createMaterial(const Material& material);

		void fetchBroadPhaseAABBs(std::vector<AABB>& outAABBs) const;

		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(bodies); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(circles); }
	private:
		void physicsStep(Real deltaTime);

		void applyExternalForces(size_t bodyCount, Real deltaTime);

		void integrate(size_t bodyCount, Real deltaTime);

		void iterativeCollisionSolving();

		void buildCircleAABBs();

		void resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions);
	};
}
