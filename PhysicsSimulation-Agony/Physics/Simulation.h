#pragma once
#include "GlmTypes.h"
#include "ObjectSoA.h"
#include "ConstraintSystem.h"
#include "Material.h"
#include "Constants.h"

#include "BroadPhaseCollisionDetector.h"
#include "NarrowPhaseCollisionDetector.h"

#include "Solving/SolvingPlanner.h"
#include "Solving/BodyCollisionSolver.h"
#include "Solving/SpringSolver.h"

#include "Interactivity/BodyHolder.h"

#include <vector>
#include <optional>
#include <array>

namespace PS_AGONY
{
	class Simulation
	{
	public:
		struct DebugData
		{
			uint32_t updatesHappened = 0;
			uint32_t updatesSupposedToHappen = 0;

			size_t bodyDataMemoryUsage = 0;
			size_t circleDataMemoryUsage = 0;
			size_t boxDataMemoryUsage = 0;
			size_t polygonDataMemoryUsage = 0;

			size_t springDataMemoryUsage = 0;

			size_t materialDataMemoryUsage = 0;

			size_t broadPhaseDetectorMemoryUsage = 0;
			size_t narrowPhaseDetectorMemoryUsage = 0;

			size_t bodyCollisionSolverMemoryUsage = 0;
			size_t bodyCollisionPlannerMemoryUsage = 0;
			size_t springSolverMemoryUsage = 0;
			size_t springPlannerMemoryUsage = 0;
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
		};

		struct SpringCreateParams
		{
			ObjectIndex bodyIndexA;
			ObjectIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Real restLength;
			Real stiffness;
			Real damping;
		};

		struct SimulationSettings
		{
			Real updateInterval = 1.0 / 60.0;
			uint32_t collisionVelocitySolvingIterations = 6;
			uint32_t collisionPositionSolvingIterations = 3;
			uint32_t springSolvingIterations = 6;
			Real maxDeltaTimePerUpdateCall = 1 / 20.0;

			// Environment.
			Real timeScale = 1.0;
			Vec2 gravity{ 0.0, -9.81 };
			Real angularVelocityDamping = 1.0; // Per second.
		};
	private:
		enum class BenchmarkDensity
		{
			NoTouching,
			Touching,
			AllTouching,
			COUNT
		};

		// Bodies.
		BodySoA bodies;
		CircleSoA circles;
		BoxSoA boxes;
		PolygonSoA polygons;

		std::vector<ObjectDeletion> deletedBodies;

		// Constraints.
		SpringSoA springs;
		SpringConstraintSystem springConstraintSystem{ springs };

		std::array<IConstraintSystem*, static_cast<size_t>(ConstraintType::COUNT)> constraintSystems{};

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Phases.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;

		// Solvers and planners.
		BodyCollisionSolver bodyCollisionSolver;
		SolvingPlanner bodyCollisionPlanner;

		SpringSolver springSolver;
		SolvingPlanner springPlanner;

		// Time.
		Real simulationRunTimer = 0;
		Real updateTimeAccumulator = 0;

		// Debug data.
		Real debugDataResetTimeAccumulator = 0;
		mutable DebugData runtimeDebugData;
		mutable DebugData debugDataSnaphot;

		// Interactivity.
		Interactivity::BodyHolder mainBodyHolder;

		// Booleans.
		bool springsWereChanged = false;
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

		std::optional<ObjectIndex> createCircle(const CircleCreateParams& params);
		std::optional<ObjectIndex> createBox(const BoxCreateParams& params);
		std::optional<ObjectIndex> createPolygon(const PolygonCreateParams& params);

		void destroyBody(ObjectIndex bodyIndex);

		void createSpring(const SpringCreateParams& params);

		MaterialIndex createMaterial(const Material& material);

		void mainBodyHolderGrabAt(Vec2 grabPosition);
		void mainBodyHolderRelease();
		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		void runBroadPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount);
		void runNarrowPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount);

		void getBroadPhaseAABBs(std::vector<AABB>& outAABBs) const;
		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return narrowPhaseCollisionDetector.getBodyCollisionData(); }
		
		SimulationSettings& getSimulationSettings() noexcept { return simulationSettings; }
		DebugData getDebugData() const noexcept { return debugDataSnaphot; }

		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(bodies); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(boxes); }
		PolygonSoAViewer getPolygons() const noexcept { return PolygonSoAViewer(polygons); }

		SpringSoAViewer getSprings() const noexcept { return SpringSoAViewer(springs); }

		Interactivity::BodyHolder& getMainBodyHolder() noexcept { return mainBodyHolder; }
	private:
		void physicsStep(Real deltaTime);

		void postUpdate();

		void integrateVelocities(size_t bodyCount, Real deltaTime);

		void integratePositions(size_t bodyCount, Real deltaTime);

		void buildBodyAABBs();
		void buildCircleAABBs();
		void buildBoxAABBs();
		void buildPolygonAABBs();

		void wrapRotation();

		void computeRotationCosSin();

		void computeWorldCenters();

		void applyBodyHolderConstraint(Real deltaTime);

		void collectMemoryUsage(DebugData& data) const;
	};
}