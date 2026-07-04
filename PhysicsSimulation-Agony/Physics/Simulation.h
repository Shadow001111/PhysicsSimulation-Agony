#pragma once
#include "GlmTypes.h"
#include "BodySoA.h"
#include "Material.h"
#include "Constants.h"

#include "BroadPhaseCollisionDetector.h"
#include "NarrowPhaseCollisionDetector.h"
#include "Solver.h"

#include "Interactivity/BodyHolder.h"

#include <vector>
#include <optional>

namespace PS_AGONY
{
	class Simulation
	{
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
			size_t polygonDataMemoryUsage = 0;

			size_t materialDataMemoryUsage = 0;
			size_t broadPhaseDetectorMemoryUsage = 0;
			size_t narrowPhaseDetectorMemoryUsage = 0;
			size_t solverMemoryUsage = 0;
		};
	private:
		struct SimulationSettings
		{
			Real updateInterval = 1.0 / 60.0;
			uint32_t collisionSolvingIterations = 6;
			Real maxDeltaTimePerUpdateCall = 1 / 20.0;

			// Environment.
			Real timeScale = 1.0;
			Vec2 gravity{ 0.0, -9.81 };
		};

		enum class BenchmarkDensity
        {
			NoTouching,
			Touching,
			AllTouching,
			COUNT
        };

		struct BodyCreateParams
		{
			Vec2 position{};
			Vec2 velocity{};
			Real rotation{ 0 };
			Real angularVelocity{ 0 };
			Real mass{ 0 };
			std::optional<Vec2> centerOfMass = std::nullopt;
			MaterialIndex materialIndex{ 0 };
			BodyTextureId textureId{ 0 };
		};

		// Bodies SoA.
		BodySoA bodies;
		CircleSoA circles;
		BoxSoA boxes;
		PolygonSoA polygons;

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Phases.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;
		Solver solver;

		// Timers.
		Real updateTimeAccumulator = 0.0;

		// Debug data.
		Real debugDataResetTimeAccumulator = 0.0;
		mutable DebugData runtimeDebugData;
		mutable DebugData debugDataSnaphot;

		// Interactivity.
		Interactivity::BodyHolder mainBodyHolder;
	public:
		struct CircleCreateParams
		{
			BodyCreateParams base; // Would better to just inherit, but field initializer can't work like this :c. For now.
			Real radius{ 0 };
		};

		struct BoxCreateParams
		{
			BodyCreateParams base;
			Vec2 size{ 0 };
		};

		struct PolygonCreateParams
		{
			BodyCreateParams base;
			Vec2* localVertices = nullptr;
			size_t verticesCount = 0;
		};

		Simulation();
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		void createCircle(const CircleCreateParams& params);
		void createBox(const BoxCreateParams& params);
		void createPolygon(const PolygonCreateParams& params);

		void destroyBody(BodyIndex bodyIndex);

		MaterialIndex createMaterial(const Material& material);

		void mainBodyHolderGrabAt(Vec2 grabPosition);
		void mainBodyHolderRelease();
		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		void runBroadPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount);
		void runNarrowPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount);

		void getBroadPhaseAABBs(std::vector<AABB>& outAABBs) const;
		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return narrowPhaseCollisionDetector.getBodyCollisionData(); }

		DebugData getDebugData() const noexcept { return debugDataSnaphot; }
		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(bodies); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(boxes); }
		PolygonSoAViewer getPolygons() const noexcept { return PolygonSoAViewer(polygons); }

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
		void buildPolygonAABBs();

		void wrapRotation();

		void computeRotationCosSin();

		void computeWorldCenters();

		void applyConstraints();

		void collectMemoryUsage(DebugData& data) const;
	};
}
