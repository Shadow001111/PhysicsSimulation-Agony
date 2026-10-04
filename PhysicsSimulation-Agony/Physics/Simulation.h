#pragma once
#include "GlmTypes.h"
#include "ConstraintSystem.h"
#include "Material.h"

#include "Physics/CollisionDetection/BroadPhaseCollisionDetector.h"
#include "Physics/CollisionDetection/NarrowPhaseCollisionDetector.h"

#include "Solving/BodyCollisionSolver.h"

#include "Interactivity/BodyHolder.h"

#include "SimulationImpl/Integrator.h"
#include "SimulationImpl/ObjectManager.h"
#include "SimulationImpl/FixedClock.h"
#include "SimulationImpl/PolygonDecomposition.h"

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

			size_t materialDataMemoryUsage = 0;

			size_t broadPhaseDetectorMemoryUsage = 0;
			size_t narrowPhaseDetectorMemoryUsage = 0;

			size_t bodyCollisionSolverMemoryUsage = 0;
			std::array<size_t, size_t(ConstraintType::COUNT)> constraintDataMemoryUsage{};

			size_t solvingPlannerMemoryUsage = 0;

			// Debug.
			Real bodyCollisionSolverVelocityError = 0;
			Real bodyCollisionSolverPositionError = 0;

			Real bodyKineticEnergySum = 0;
		};

		struct SimulationSettings
		{
			// Clock.
			FixedClock::Settings clockSettings;

			// Solving. Constraint iterations live in IConstraintSystem::iterations.
			uint32_t collisionVelocitySolvingIterations = 6;
			uint32_t collisionPositionSolvingIterations = 3;

			// Integration.
			Integrator::IntegrationSettings integratorSettings;

			// Debug.
			bool trackBodyCollisionSolverConstraintErrors = false;
			bool trackBodyKineticEnergySum = false;
		};
	private:
		// Internal helpers.
		ObjectManager objectManager;
		FixedClock clock;

		// Constraints. Each system owns its SoA and solver.
		SpringConstraintSystem springConstraintSystem;
		RevoluteJointConstraintSystem jointConstraintSystem;
		RodConstraintSystem rodConstraintSystem;
		PrismaticJointConstraintSystem prismaticJointConstraintSystem;
		SliderJointConstraintSystem sliderJointConstraintSystem;
		MotorConstraintSystem motorConstraintSystem;

		ConstraintSystemArray constraintSystems{}; // Indexed by ConstraintType, which is also the solving order.

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Phases.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;

		// Solvers.
		BodyCollisionSolver bodyCollisionSolver;

		// Time.
		Real simulationRunTimer = 0;

		// Debug data.
		Real debugDataResetTimeAccumulator = 0;
		mutable DebugData runtimeDebugData;
		mutable DebugData debugDataSnaphot;

		// Interactivity.
		Interactivity::BodyHolder mainBodyHolder;

		// Render.
		Real renderAlpha = 0;
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

		__forceinline std::optional<PolygonColliderCreateResult> createPolygonCollider(const PolygonColliderCreateParams& params)
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
			// Hack. TODO: Integrate it into attachment system or constraint system or whatever!
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

		void createSpring(const SpringConstraintSystem::CreateParams& params) { springConstraintSystem.create(params, objectManager.bodies); }
		void destroySpring(uint32_t index) { springConstraintSystem.removeConstraint(index, objectManager.bodies); }

		void createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& params) { jointConstraintSystem.create(params, objectManager.bodies); }
		void destroyRevoluteJoint(uint32_t index) { jointConstraintSystem.removeConstraint(index, objectManager.bodies); }

		void createRod(const RodConstraintSystem::CreateParams& params) { rodConstraintSystem.create(params, objectManager.bodies); }
		void destroyRod(uint32_t index) { rodConstraintSystem.removeConstraint(index, objectManager.bodies); }

		void createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& params) { prismaticJointConstraintSystem.create(params, objectManager.bodies); }
		void destroyPrismaticJoint(uint32_t index) { prismaticJointConstraintSystem.removeConstraint(index, objectManager.bodies); }

		void createSliderJoint(const SliderJointConstraintSystem::CreateParams& params) { sliderJointConstraintSystem.create(params, objectManager.bodies); }
		void destroySliderJoint(uint32_t index) { sliderJointConstraintSystem.removeConstraint(index, objectManager.bodies); }

		void createMotor(const MotorConstraintSystem::CreateParams& params) { motorConstraintSystem.create(params, objectManager.bodies); }
		void destroySliderMotor(uint32_t index) { motorConstraintSystem.removeConstraint(index, objectManager.bodies); }

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

		SpringSoAViewer getSprings() const noexcept { return springConstraintSystem.getViewer(); }
		JointSoAViewer getJoints() const noexcept { return jointConstraintSystem.getViewer(); }
		RodSoAViewer getRods() const noexcept { return rodConstraintSystem.getViewer(); }
		PrismaticJointSoAViewer getPrismaticJoints() const noexcept { return prismaticJointConstraintSystem.getViewer(); }
		SliderJointSoAViewer getSliderJoints() const noexcept { return sliderJointConstraintSystem.getViewer(); }

		IConstraintSystem& getConstraintSystem(ConstraintType type) noexcept { return *constraintSystems[size_t(type)]; }
		const IConstraintSystem& getConstraintSystem(ConstraintType type) const noexcept { return *constraintSystems[size_t(type)]; }

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