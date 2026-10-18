#pragma once
#include "SolverBase.h"

#include "Physics/SoA/ColliderSoA.h"
#include "Physics/Material.h"
#include "Physics/CollisionDetection/NarrowPhaseCollisionDetector.h"

#include <array>
#include <span>
#include <vector>

namespace PS_AGONY
{
	class BodyCollisionSolver : public SolverBase
	{
		// Structures / classes.

		struct VelocityConstraintData
		{
			struct ContactData
			{
				Vec2 rAPerp, rBPerp;
				Real normalMass;
				Real tangentMass;
				Real velocityBias;
			};

			std::array<ContactData, 2> points;
		};

		struct FrictionData
		{
			Real staticFriction;
			Real dynamicFriction;
		};

		// Lean per-constraint streams: the solver loops never touch the full BodyCollisionData
		// (colliders, contact points, ids, penetration). Built once in computeConstraintData.
		// TODO: also cache invMassA/B and invInertiaA/B here to avoid per-iteration body gathers (not measured yet).
		struct VelocityHotData
		{
			ObjectIndex bodyA, bodyB;
			Vec2 normal;
			uint32_t contactCount;
			std::array<PersistentContactData, 2> impulses; // Working copy; scattered back to the detector's data at the end.
		};

		struct PositionHotData
		{
			ObjectIndex bodyA, bodyB;
			Vec2 normal;
			// penetration + dot(posB0 - posA0, normal). Current penetration = this - dot(posB - posA, normal).
			Real penetrationBase;
		};

		struct SimulationSettings
		{
			enum class SolverType
			{
				Iterative,
				Block
			};

			static constexpr Real RESTITUTION_VELOCITY_THRESHOLD = 0.0;
			static constexpr Real MAX_CONDITION_NUMBER = 1000.0f;
			static constexpr SolverType NORMAL_SOLVER_TYPE = SolverType::Block;
			static constexpr SolverType FRICTION_SOLVER_TYPE = SolverType::Iterative;

			// Baumgarte stabilization.
			const Real positionCorrectionPercent = 0.5;
			const Real positionCorrectionSlop = 0.001;
		};

		// Member fields.

		const std::vector<Material>* materials = nullptr;
		ColliderSoAViewer colliders;

		std::vector<ObjectPair> collidingBodyPairs;

		// All containers below are indexed in solving order.
		std::vector<VelocityHotData> velocityHotContainer;
		std::vector<PositionHotData> positionHotContainer;
		std::vector<VelocityConstraintData> velocityConstraintContainer;
		std::vector<FrictionData> frictionDataContainer;

		size_t activeCollisionCount = 0;

		SimulationSettings simulationSettings;
	public:
		BodyCollisionSolver() = default;
		~BodyCollisionSolver() override = default;
		BodyCollisionSolver(const BodyCollisionSolver&) = delete;
		BodyCollisionSolver& operator=(const BodyCollisionSolver&) = delete;
		BodyCollisionSolver(BodyCollisionSolver&&) = delete;
		BodyCollisionSolver& operator=(BodyCollisionSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const ColliderSoAViewer& colliders,
			const std::vector<Material>& materials
		);

		void solveCollisions(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);

		void reportNoCollisions();

		// (Velocity, Position).
		Vec2 computeConstraintErrors() const;

		size_t getMemoryUsage() const override;
	private:
		// Collision constraint solving.

		size_t planWorkerCount(size_t collisionCount) const;

		// Builds all constraint streams. If 'order' is not null, constraint c is built from source[order[c]].
		void computeConstraintData(const std::vector<BodyCollisionData>& source, const size_t* order, size_t collisionCount);

		// Writes the solved impulses back to the detector's (mutable) persistent contact data.
		void scatterImpulses(const std::vector<BodyCollisionData>& target, const size_t* order, size_t collisionCount) const;

		void applyWarmStarting(SolvingPlan::Pass pass);

		void solveVelocityConstraints(SolvingPlan::Pass pass);

		void solvePositionConstraints(SolvingPlan::Pass pass);

		void solveConstraintsThreaded(
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	};
}