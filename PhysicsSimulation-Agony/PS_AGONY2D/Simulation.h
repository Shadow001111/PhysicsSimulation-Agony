#pragma once
#include "IslandManager.h"
#include "Constants.h"
#include "MemoryUsageTracking.h"

#include "Types/Material.h"
#include "Types/Types.h"

#include "CollisionDetection/Broad/BroadPhaseCollisionDetector.h"
#include "CollisionDetection/Narrow/NarrowPhaseCollisionDetector.h"
#include "CollisionDetection/CCD/ContinuousCollisionDetector.h"

#include "Solving/ConstraintScheduler.h"
#include "Solving/BodyCollisionSolver.h"

#include "Interactivity/BodyHolder.h"

#include "Integration/Integrator.h"

#include "SimulationImpl/MaterialManager.h"
#include "SimulationImpl/CollisionCallbackManager.h"
#include "SimulationImpl/ObjectManager.h"
#include "SimulationImpl/FixedClock.h"
#include "SimulationImpl/AllConstraintSystem.h"

#include "SimulationAPIs/ObjectAPI.h"
#include "SimulationAPIs/ConstraintAPI.h"

#include <vector>
#include <optional>
#include <array>
#include <span>

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

			// Solving errors.
			Real bodyCollisionSolverVelocityError = 0;
			Real bodyCollisionSolverPositionError = 0;
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
			Integrator::Settings integratorSettings;

			// Continuous collision (fast dynamic vs static). Zero cost when disabled.
			bool continuousCollisionEnabled = true;
			ContinuousCollisionDetector::Settings ccdSettings;

			// Sleep.
			Real linearSpeedSleepThreshold = 0.1f;
			Real angularSpeedSleepThreshold = 0.1f;
			Real sleepTimeThreshold = 0.5f;

			// Debug.
			bool trackBodyCollisionSolverConstraintErrors = false;
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
		PrismaticJointConstraintSystem prismaticJointConstraintSystem;
		SliderJointConstraintSystem sliderJointConstraintSystem;
		MotorConstraintSystem motorConstraintSystem;

		ConstraintScheduler constraintScheduler;
		ConstraintSystemArray constraintSystems{}; // Indexed by ConstraintType, which is also the solving order.

		// Non-owning facades. Must be declared after everything they reference.
		ObjectAPI objectAPI{ objectManager, constraintSystems };
		ConstraintAPI constraintAPI{ objectManager, constraintSystems.data() };

		// Settings.
		SimulationSettings simulationSettings;

		// Phases.
		BroadPhaseCollisionDetector broadPhaseCollisionDetector;
		NarrowPhaseCollisionDetector narrowPhaseCollisionDetector;
		ContinuousCollisionDetector continuousCollisionDetector;

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
		bool stepRequested = false;

		// Render.
		Real renderAlpha = 0;

		// Query colliders.
		std::vector<ObjectDenseIndex> broadPhaseColliders;
		std::vector<std::pair<ObjectDenseIndex, Real>> narrowPhaseColliders;

		// Islands.
		std::span<const ObjectPair> islandBodyCollisionEdges{};
		IslandManager islandManager;

		// Static edit wake scratch.
		std::vector<uint32_t> staticEditIslandIds;
		std::vector<Handle> staticEditPartners;
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

		void requestStep() noexcept { stepRequested = true; }

		MaterialIndex createMaterial(const Material& material);

		// Colliders using this material fall back to the default material (0). Returns false if not destroyable.
		// Wakes the islands of all colliders using it.
		bool destroyMaterial(MaterialIndex materialIndex);

		// Values are clamped (elasticity [0, 1], frictions >= 0). Wakes the islands of all colliders using it.
		// Returns false for dead or out-of-range slots.
		bool setMaterial(MaterialIndex materialIndex, const Material& material);

		size_t getMaterialCount() const noexcept { return materialManager.getMaterials().size(); }
		bool isMaterialAlive(MaterialIndex materialIndex) const noexcept { return materialManager.isAlive(materialIndex); }
		const Material& getMaterial(MaterialIndex materialIndex) const noexcept { return materialManager.get(materialIndex); }

		// Collision callbacks. Fired at the end of a physics step, per collider side, when the impact impulse
		// (approach speed * effective mass) reaches the collider's threshold.
		// nullopt if 'function' is null or the table is full.
		std::optional<CollisionCallbackIndex> createCollisionCallback(CollisionCallback function, void* userData = nullptr);

		// Colliders using it are reset to no callback. Returns false for id 0 and dead ids.
		bool destroyCollisionCallback(CollisionCallbackIndex callbackIndex);

		// Id 0 clears the collider's callback (threshold ignored).
		// Returns false for an invalid collider, a dead callback id, or a non-positive threshold.
		bool setColliderCollisionCallback(ObjectDenseIndex colliderIndex, CollisionCallbackIndex callbackIndex, Real impulseThreshold);

		void mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp);

		const BroadPhaseCollisionDetector& getBroadPhaseCollisionDetector() const noexcept { return broadPhaseCollisionDetector; };
		const NarrowPhaseCollisionDetector& getNarrowPhaseCollisionDetector() const noexcept { return narrowPhaseCollisionDetector; };

		auto getBodyCollisionData() const noexcept { return narrowPhaseCollisionDetector.getBodyCollisionData(); }

		SimulationSettings& getSimulationSettings() noexcept { return simulationSettings; }
		DebugData getDebugData() const noexcept { return debugDataSnaphot; }
		void collectMemoryUsage(MemoryUsageTracking& memoryTracking, uint32_t parentNodeIndex) const;

		IConstraintSystem& getConstraintSystem(ConstraintType type) noexcept { return *constraintSystems[size_t(type)]; }
		const IConstraintSystem& getConstraintSystem(ConstraintType type) const noexcept { return *constraintSystems[size_t(type)]; }

		Interactivity::BodyHolder& getMainBodyHolder() noexcept { return mainBodyHolder; }

		Real getRenderAlpha() const noexcept { return renderAlpha; }

		ObjectAPI& objects() noexcept { return objectAPI; }
		const ObjectAPI& objects() const noexcept { return objectAPI; }
		ConstraintAPI& constraints() noexcept { return constraintAPI; }
		const ConstraintAPI& constraints() const noexcept { return constraintAPI; }
		IslandManager& islands() noexcept { return islandManager; }
		const IslandManager& islands() const noexcept { return islandManager; }
	private:
		void physicsStep(Real deltaTime);

		void setDataViewers();

		void integrateAndApplyConstraints(Real deltaTime);

		void collisionStep();

		void flushCommands();

		// Static bodies are not part of islands: islands resting on an edited static collider are woken through
		// its saved contacts, and awake bodies touching it are kept from falling asleep this step.
		void wakeIslandsOfEditedStatics();

		// Wakes sleeping islands of bodies with a collider using the material. Static colliders using it are marked
		// edited, so islands resting on them wake at the next flush.
		void wakeCollidersUsingMaterial(MaterialIndex materialIndex);

		void updateDebugData(Real deltaTime, uint32_t stepCount);

		void dispatchCollisionEvents();

		void preUpdate();
		void postUpdate();

		// Rebuilds BVH trees for collider changes made outside physics steps (paused, or no step this frame).
		void refreshSpatialData();
		// Full pass over all body groups and trees. Needed after structural changes (spatialDirty).
		void rebuildSpatialData();
		// Per-step pass: dynamic bodies and the dynamic tree only.
		void updateDynamicSpatialData();

		// Puts dynamic bodies below the sleep thresholds to sleep. Held bodies are skipped.
		void updateSleeping(float deltaTimeF);

		// Held bodies are modified by the holder every step, so they must be awake before integration.
		void wakeHeldBodies();

		// Consume and clear objectManager.wokenBodies / sleptBodies.
		// Saved contact data is restored on wake (warm starting) and stored on sleep.
		void wakeSleepingContactData();
		void sleepContactData();

		void buildColliderAABBs(bool includeStaticAndSleeping);

		// range.begin must be 0 (aligned SIMD loads).
		void computeBodyWorldCenters(IndexRange range);

		void computeColliderWorldPositions(IndexRange colliderRange);
		void computeColliderWorldRotations(IndexRange colliderRange);

		void applyBodyHolderConstraint(Real deltaTime, const Interactivity::BodyHolder& holder);
	};
}