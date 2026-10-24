#pragma once
#include "ConstraintSystem.h"
#include "Constants.h"

#include "Types/Material.h"
#include "Types/Types.h"

#include "CollisionDetection/Broad/BroadPhaseCollisionDetector.h"
#include "CollisionDetection/Narrow/NarrowPhaseCollisionDetector.h"

#include "Solving/ConstraintScheduler.h"
#include "Solving/BodyCollisionSolver.h"

#include "Interactivity/BodyHolder.h"

#include "Integration/Integrator.h"

#include "SimulationImpl/MaterialManager.h"
#include "SimulationImpl/CollisionCallbackManager.h"
#include "SimulationImpl/ObjectManager.h"
#include "SimulationImpl/FixedClock.h"

#include <vector>
#include <optional>
#include <array>

namespace PS_AGONY2D
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
			size_t capsuleDataMemoryUsage = 0;

			size_t materialDataMemoryUsage = 0;
			size_t collisionCallbackMemoryUsage = 0;

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

			// Collision events. Also requires at least one registered collision callback.
			bool collisionEventsEnabled = true;

			// Integration.
			Integrator::IntegrationSettings integratorSettings;

			// Debug.
			bool trackBodyCollisionSolverConstraintErrors = false;
			bool trackBodyKineticEnergySum = false;
		};
	private:
		// Materials. Declared before objectManager, which references its storage.
		MaterialManager materialManager;
		CollisionCallbackManager collisionCallbackManager;

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

		ConstraintScheduler constraintScheduler;
		ConstraintSystemArray constraintSystems{}; // Indexed by ConstraintType, which is also the solving order.

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

		// Pause.
		bool paused = false;

		// Render.
		Real renderAlpha = 0;

		// Other.
		std::vector<ColliderDenseIndex> broadPhaseColliders;
		std::vector<std::pair<ColliderDenseIndex, Real>> narrowPhaseColliders;
	public:
		Simulation();
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		void setPause(bool pause) noexcept { paused = pause; }
		bool isPaused() const noexcept { return paused; }

		__forceinline std::optional<ObjectDenseIndex> createBody(const BodyCreateParams& params)
		{
			return objectManager.createBody(params);
		}

		__forceinline std::optional<ColliderDenseIndex> createCircleCollider(const CircleColliderCreateParams& params)
		{
			return objectManager.createCircleCollider(params);
		}

		__forceinline std::optional<ColliderDenseIndex> createBoxCollider(const BoxColliderCreateParams& params)
		{
			return objectManager.createBoxCollider(params);
		}

		__forceinline std::optional<PolygonColliderCreateResult> createPolygonCollider(const PolygonColliderCreateParams& params)
		{
			return objectManager.createPolygonCollider(params);
		}

		__forceinline std::optional<ColliderDenseIndex> createCapsuleCollider(const CapsuleColliderCreateParams& params)
		{
			return objectManager.createCapsuleCollider(params);
		}

		__forceinline std::optional<ObjectDenseIndex> createCircle(const CircleCreateParams& params)
		{
			return objectManager.createCircle(params);
		}

		__forceinline std::optional<ObjectDenseIndex> createBox(const BoxCreateParams& params)
		{
			return objectManager.createBox(params);
		}

		__forceinline std::optional<ObjectDenseIndex> createPolygon(const PolygonCreateParams& params)
		{
			return objectManager.createPolygon(params);
		}

		__forceinline std::optional<ObjectDenseIndex> createCapsule(const CapsuleCreateParams& params)
		{
			return objectManager.createCapsule(params);
		}

		// Persistent handles. Valid across swap-and-pop; stale after destroy (generation bump).
		// Dense indices are only stable until the next create/destroy; handles survive it.
		Handle getBodyHandle(ObjectDenseIndex bodyIndex) const noexcept
		{
			return bodyIndex < objectManager.bodies.getCount() ? objectManager.bodies.handleAt(bodyIndex) : Handle{};
		}
		Handle getColliderHandle(ColliderDenseIndex colliderIndex) const noexcept
		{
			return colliderIndex < objectManager.colliders.getCount() ? objectManager.colliders.handleAt(colliderIndex) : Handle{};
		}

		bool isBodyValid(Handle h) const noexcept { return objectManager.bodies.isValid(h); }
		bool isColliderValid(Handle h) const noexcept { return objectManager.colliders.isValid(h); }

		// Handle::kNull if stale.
		uint32_t getBodyIndex(Handle h) const noexcept { return objectManager.bodies.indexOf(h); }
		uint32_t getColliderIndex(Handle h) const noexcept { return objectManager.colliders.indexOf(h); }

		// Hot path: no validation. Handle must be alive.
		uint32_t getBodyIndexUnchecked(Handle h) const noexcept { return objectManager.bodies.indexOfUnchecked(h); }
		uint32_t getColliderIndexUnchecked(Handle h) const noexcept { return objectManager.colliders.indexOfUnchecked(h); }

		// Handle-returning creators. Dense-index versions above are unchanged.
		std::optional<Handle> createBodyH(const BodyCreateParams& params) { return toBodyHandle(objectManager.createBody(params)); }
		std::optional<Handle> createCircleH(const CircleCreateParams& params) { return toBodyHandle(objectManager.createCircle(params)); }
		std::optional<Handle> createBoxH(const BoxCreateParams& params) { return toBodyHandle(objectManager.createBox(params)); }
		std::optional<Handle> createPolygonH(const PolygonCreateParams& params) { return toBodyHandle(objectManager.createPolygon(params)); }
		std::optional<Handle> createCapsuleH(const CapsuleCreateParams& params) { return toBodyHandle(objectManager.createCapsule(params)); }

		// Collider creators taking a body handle (params.bodyIndex is ignored). nullopt if the handle is stale.
		std::optional<Handle> createCircleColliderH(Handle body, CircleColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createCircleCollider(params));
		}
		std::optional<Handle> createBoxColliderH(Handle body, BoxColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createBoxCollider(params));
		}
		std::optional<Handle> createCapsuleColliderH(Handle body, CapsuleColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createCapsuleCollider(params));
		}
		// Pieces are contiguous in dense order: use getColliderHandle(first + i) right after the call.
		std::optional<PolygonColliderCreateResult> createPolygonColliderH(Handle body, PolygonColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return objectManager.createPolygonCollider(params);
		}

		// Stale handle: no-op.
		void destroyBody(Handle h)
		{
			const uint32_t i = getBodyIndex(h);
			if (i != Handle::kNull) destroyBody(static_cast<ObjectDenseIndex>(i));
		}
		void destroyCollider(Handle h)
		{
			const uint32_t i = getColliderIndex(h);
			if (i != Handle::kNull) destroyCollider(static_cast<ColliderDenseIndex>(i));
		}

		void destroyBody(ObjectDenseIndex bodyIndex)
		{
			objectManager.destroyBody(bodyIndex, constraintSystems);
		}

		__forceinline void destroyCollider(ColliderDenseIndex colliderIndex)
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

		// Colliders using this material fall back to the default material (0). Returns false if not destroyable.
		bool destroyMaterial(MaterialIndex materialIndex);

		// Collision callbacks. Fired at the end of a physics step, per collider side, when the impact impulse
		// (approach speed * effective mass) reaches the collider's threshold.
		// nullopt if 'function' is null or the table is full.
		std::optional<CollisionCallbackIndex> createCollisionCallback(CollisionCallback function, void* userData = nullptr);

		// Colliders using it are reset to no callback. Returns false for id 0 and dead ids.
		bool destroyCollisionCallback(CollisionCallbackIndex callbackIndex);

		// Id 0 clears the collider's callback (threshold ignored).
		// Returns false for an invalid collider, a dead callback id, or a non-positive threshold.
		bool setColliderCollisionCallback(ColliderDenseIndex colliderIndex, CollisionCallbackIndex callbackIndex, Real impulseThreshold);

		// TODO: Add more like that.
		std::span<const ColliderDenseIndex> getBodyColliders(ObjectDenseIndex bodyIndex);

		void mainBodyHolderGrabAt(Vec2 grabPosition);
		void mainBodyHolderRelease();
		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		const BroadPhaseCollisionDetector& getBroadPhaseCollisionDetector() const noexcept { return broadPhaseCollisionDetector; };
		const NarrowPhaseCollisionDetector& getNarrowPhaseCollisionDetector() const noexcept { return narrowPhaseCollisionDetector; };

		auto getBodyCollisionData() const noexcept { return narrowPhaseCollisionDetector.getBodyCollisionData(); }

		SimulationSettings& getSimulationSettings() noexcept { return simulationSettings; }
		DebugData getDebugData() const noexcept { return debugDataSnaphot; }

		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(objectManager.bodies); }
		ColliderSoAViewer getColliders() const noexcept { return ColliderSoAViewer(objectManager.colliders); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(objectManager.circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(objectManager.boxes); }
		PolygonSoAViewer getPolygons() const noexcept { return PolygonSoAViewer(objectManager.polygons); }
		CapsuleSoAViewer getCapsules() const noexcept { return CapsuleSoAViewer(objectManager.capsules); }

		SpringSoAViewer getSprings() const noexcept { return springConstraintSystem.getViewer(); }
		RevoluteJointSoAViewer getJoints() const noexcept { return jointConstraintSystem.getViewer(); }
		RodSoAViewer getRods() const noexcept { return rodConstraintSystem.getViewer(); }
		PrismaticJointSoAViewer getPrismaticJoints() const noexcept { return prismaticJointConstraintSystem.getViewer(); }
		SliderJointSoAViewer getSliderJoints() const noexcept { return sliderJointConstraintSystem.getViewer(); }
		MotorSoAViewer getMotors() const noexcept { return motorConstraintSystem.getViewer(); }

		std::optional<BodyData> getBodyData(ObjectDenseIndex index) const noexcept
		{
			return objectManager.bodies.getBodyData(index);
		}

		IConstraintSystem& getConstraintSystem(ConstraintType type) noexcept { return *constraintSystems[size_t(type)]; }
		const IConstraintSystem& getConstraintSystem(ConstraintType type) const noexcept { return *constraintSystems[size_t(type)]; }

		Interactivity::BodyHolder& getMainBodyHolder() noexcept { return mainBodyHolder; }

		Real getRenderAlpha() const noexcept { return renderAlpha; }
	private:
		void physicsStep(Real deltaTime);

		void dispatchCollisionEvents();

		void preUpdate();

		void postUpdate();

		// Rebuilds BVH trees for collider changes made outside physics steps (paused, or no step this frame).
		void refreshSpatialData();
		void rebuildSpatialData();

		void buildColliderAABBs();

		void computeBodyWorldCenters();

		void computeColliderWorldTransforms();
		void computeColliderWorldPositions();
		void computeColliderWorldRotations();

		void applyBodyHolderConstraint(Real deltaTime);

		Real computeBodyTotalKineticEnergy();

		void collectMemoryUsage(DebugData& data) const;

		std::optional<Handle> toBodyHandle(std::optional<ObjectDenseIndex> index) const noexcept
		{
			return index ? std::optional<Handle>(objectManager.bodies.handleAt(*index)) : std::nullopt;
		}
		std::optional<Handle> toColliderHandle(std::optional<ColliderDenseIndex> index) const noexcept
		{
			return index ? std::optional<Handle>(objectManager.colliders.handleAt(*index)) : std::nullopt;
		}
	};
}