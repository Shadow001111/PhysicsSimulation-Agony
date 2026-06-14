#pragma once
#include "GlmTypes.h"
#include "BodySoA.h"
#include "Material.h"
#include "Constants.h"

#include "BroadPhaseCollisionDetector.h"
#include "NarrowPhaseCollisionDetector.h"

#include "Interactivity/BodyHolder.h"

#include <vector>

namespace PS_AGONY
{
	class Simulation
	{
		static constexpr bool ENABLE_VELOCITY_CORRECTION = true;
	public:
		struct DebugData
		{
			uint32_t updatesHappened = 0;
			uint32_t updatesSupposedToHappen = 0;

			uint32_t collisionSolvingIterationsHappened = 0;
			uint32_t maxCollisionSolvingIterations = 0;

			size_t bodyDataMemoryUsage = 0;
			size_t circleDataMemoryUsage = 0;
			size_t boxDataMemoryUsage = 0;

			size_t materialDataMemoryUsage = 0;
			size_t broadPhaseDetectorMemoryUsage = 0;
			size_t narrowPhaseDetectorMemoryUsage = 0;
		};
	private:
		struct SimulationSettings
		{
			//
			Real updateInterval = 1 / 300.0;
			uint32_t collisionSolvingIterations = 3;
			Real maxDeltaTimePerUpdateCall = 1 / 20.0;

			// Environment.
			Real timeScale = 1.0;
			Vec2 gravity{ 0.0, -9.81 };

			// Baumgarte stabilization. No slop.
			const Real positionCorrectionPercent = 1.0;
			const Real velocityCorrectionStrength = 8.0;
		};

		// Bodies SoA.
		BodySoA bodies;
		CircleSoA circles;
		BoxSoA boxes;

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Collisions.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;

		// Timers.
		Real updateTimeAccumulator = 0.0;

		// Debug data.
		Real debugDataResetTimeAccumulator = 0.0;
		mutable DebugData runtimeDebugData;
		mutable DebugData debugDataSnaphot;

		// Interactivity.
		Interactivity::BodyHolder mainBodyHolder;
	public:
		Simulation();
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		void createCircle(Vec2 position, Vec2 velocity, Real rotation, Real angularVelocity, Real mass, Vec2 centerOfMass, MaterialIndex materialIndex, Real radius, BodyTextureId textureId = 0);
		void createBox(Vec2 position, Vec2 velocity, Real rotation, Real angularVelocity, Real mass, Vec2 centerOfMass, MaterialIndex materialIndex, Vec2 size, BodyTextureId textureId = 0);

		void destroyBody(BodyIndex bodyIndex);

		MaterialIndex createMaterial(const Material& material);

		void mainBodyHolderGrabAt(Vec2 grabPosition);
		void mainBodyHolderRelease();
		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		void fetchBroadPhaseAABBs(std::vector<AABB>& outAABBs) const;

		DebugData getDebugData() const noexcept { return debugDataSnaphot; }
		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(bodies); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(boxes); }

		Interactivity::BodyHolder& getMainBodyHolder() noexcept { return mainBodyHolder; }
	private:
		void physicsStep(Real deltaTime);

		void postUpdate();

		void applyExternalForces(size_t bodyCount, Real deltaTime);

		void integrate(size_t bodyCount, Real deltaTime);

		void iterativeCollisionSolving(Real deltaTime);

		void buildBodyAABBs();
		void buildCircleAABBs();
		void buildBoxAABBs();

		void wrapRotation();

		void computeRotationCosSin();

		void computeTruePositions();

		void experimentalBodyCollisionDataGraphColoring(const std::vector<BodyCollisionData>& narrowPhaseCollisions);

		void resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions);

		void applyConstraints();

		void collectMemoryUsage(DebugData& data) const;
	};
}
