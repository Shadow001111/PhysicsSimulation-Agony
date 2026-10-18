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

		struct PositionConstraintData
		{
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
		std::vector<BodyCollisionData> orderedCollisionData;

		std::vector<PositionConstraintData> positionConstraintContainer;
		std::vector<VelocityConstraintData> velocityConstraintContainer;
		std::vector<FrictionData> frictionDataContainer;

		std::span<const BodyCollisionData> activeCollisionData;

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

		void computeConstraintData(const std::vector<BodyCollisionData>& collisionDataContainer);

		void applyWarmStarting(SolvingPlan::Pass pass);

		void solveVelocityConstraints(SolvingPlan::Pass pass);

		void solvePositionConstraints(SolvingPlan::Pass pass);

		void solveConstraintsThreaded(
			const std::vector<BodyCollisionData>& collisionDataContainer,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	};
}