#pragma once
#include "GlmTypes.h"
#include "ConstraintSystem.h"
#include "Material.h"

#include "BroadPhaseCollisionDetector.h"
#include "NarrowPhaseCollisionDetector.h"

#include "Solving/SolvingPlanner.h"
#include "Solving/BodyCollisionSolver.h"
#include "Solving/SpringSolver.h"

#include "Interactivity/BodyHolder.h"

#include "SimulationImpl/Integrator.h"
#include "SimulationImpl/ObjectManager.h"

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
			// UPS.
			uint32_t updatesHappened = 0;
			uint32_t updatesSupposedToHappen = 0;

			// Memory.
			size_t bodyDataMemoryUsage = 0;
			size_t colliderDataMemoryUsage = 0;
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

			// Debug.
			Real bodyCollisionSolverVelocityError = 0;
			Real bodyCollisionSolverPositionError = 0;

			Real bodyKineticEnergySum = 0;
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
			// Orchestration.
			Real updateInterval = 1.0 / 300.0;
			Real maxDeltaTimePerUpdateCall = 1 / 20.0;

			// Solving.
			uint32_t collisionVelocitySolvingIterations = 6;
			uint32_t collisionPositionSolvingIterations = 3;
			uint32_t springSolvingIterations = 6;

			// Integration.
			Integrator::IntegrationSettings integration;

			// Debug.
			bool trackBodyCollisionSolverConstraintErrors = false;
			bool trackBodyKineticEnergySum = false;
		};
	private:
		// Object manager.
		ObjectManager objectManager;

		// Constraints.
		SpringSoA springs;
		SpringConstraintSystem springConstraintSystem{ springs };

		ConstraintSystemArray constraintSystems{};

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

		// Render.
		Real renderAlpha = 0;
		uint32_t lastStepCount = 1;
	public:
		Simulation();
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		__forceinline std::optional<ObjectIndex> createBody(const BodyCreateParams& params)
		{
			return objectManager.createBody(params);
		}

		__forceinline std::optional<ColliderIndex> createCircleCollider(const CircleColliderCreateParams& params)
		{
			return objectManager.createCircleCollider(params);
		}

		__forceinline std::optional<ColliderIndex> createBoxCollider(const BoxColliderCreateParams& params)
		{
			return objectManager.createBoxCollider(params);
		}

		__forceinline std::optional<ColliderIndex> createPolygonCollider(const PolygonColliderCreateParams& params)
		{
			return objectManager.createPolygonCollider(params);
		}

		__forceinline std::optional<ObjectIndex> createCircle(const CircleCreateParams& params)
		{
			return objectManager.createCircle(params);
		}

		__forceinline std::optional<ObjectIndex> createBox(const BoxCreateParams& params)
		{
			return objectManager.createBox(params);
		}

		__forceinline std::optional<ObjectIndex> createPolygon(const PolygonCreateParams& params)
		{
			return objectManager.createPolygon(params);
		}

		void destroyBody(ObjectIndex bodyIndex)
		{
			// Hack. TODO: Integrate it into attackment system or constraint system or whatever!
			{
				if (mainBodyHolder.heldBody.has_value() && mainBodyHolder.heldBody.value() == bodyIndex)
					mainBodyHolderRelease();

				const ObjectIndex oldBackIndex = static_cast<ObjectIndex>(objectManager.bodies.getCount() - 1);
				if (mainBodyHolder.heldBody.has_value() && mainBodyHolder.heldBody.value() == oldBackIndex)
					mainBodyHolder.heldBody = bodyIndex;
			}
			objectManager.destroyBody(bodyIndex, constraintSystems);
		}

		__forceinline void destroyCollider(ColliderIndex colliderIndex)
		{
			return objectManager.destroyCollider(colliderIndex, constraintSystems);
		}

		void createSpring(const SpringCreateParams& params);

		// Destroys a single spring by its index in the spring constraint system.
		void destroySpring(uint32_t springIndex);

		MaterialIndex createMaterial(const Material& material);

		void mainBodyHolderGrabAt(Vec2 grabPosition);
		void mainBodyHolderRelease();
		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		const BroadPhaseCollisionDetector& getBroadPhaseCollisionDetector() const noexcept { return broadPhaseCollisionDetector; };
		const NarrowPhaseCollisionDetector& getNarrowPhaseCollisionDetector() const noexcept { return narrowPhaseCollisionDetector; };

		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return narrowPhaseCollisionDetector.getBodyCollisionData(); }

		SimulationSettings& getSimulationSettings() noexcept { return simulationSettings; }
		DebugData getDebugData() const noexcept { return debugDataSnaphot; }

		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(objectManager.bodies); }
		ColliderSoAViewer getColliders() const noexcept { return ColliderSoAViewer(objectManager.colliders); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(objectManager.circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(objectManager.boxes); }
		PolygonSoAViewer getPolygons() const noexcept { return PolygonSoAViewer(objectManager.polygons); }

		SpringSoAViewer getSprings() const noexcept { return SpringSoAViewer(springs); }

		Interactivity::BodyHolder& getMainBodyHolder() noexcept { return mainBodyHolder; }

		Real getRenderAlpha() const noexcept { return renderAlpha; }
	private:
		void physicsStep(Real deltaTime);

		void preUpdate();

		void postUpdate();

		void buildColliderAABBs();
		void buildCircleAABBs();
		void buildBoxAABBs();
		void buildPolygonAABBs();

		void computeBodyWorldCenters();

		void computeColliderWorldTransforms();
		void computeColliderWorldPositions();
		void computeColliderWorldRotations();
		void wrapColliderRotations();

		void applyBodyHolderConstraint(Real deltaTime);

		Real computeBodyTotalKineticEnergy();

		void collectMemoryUsage(DebugData& data) const;
	};
}