#pragma once
#include "SolverBase.h"
#include "../NarrowPhaseCollisionDetector.h"
#include "../Material.h"

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
				Real normalMassXElasticityFactor;
				Real tangentMass;
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
			Vec2 localAnchorA;
			Vec2 localAnchorB;
		};

		struct SimulationSettings
		{
			// Baumgarte stabilization.
			const Real positionCorrectionPercent = 0.5;
			const Real positionCorrectionSlop = 0.001;
		};

		enum class VelocitySolverType
		{
			// Perfomance on two contact constraints:
			Sequential, // First contact has error, second contact error is zero.
			Block // Both contacts have no error.
		};

		// Static memeber fields.

		static constexpr VelocitySolverType VELOCITY_SOLVER_TYPE = VelocitySolverType::Block;

		// Memeber fields.

		const std::vector<Material>* materials = nullptr;

		std::vector<ObjectPair> collidingBodyPairs;
		std::vector<BodyCollisionData> orderedCollisionData;

		std::vector<PositionConstraintData> positionConstraintContainer;
		std::vector<VelocityConstraintData> velocityConstraintContainer;
		std::vector<FrictionData> frictionDataContainer;

		std::span<const BodyCollisionData> debugCollisionDataContainer;

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
			const std::vector<Material>& materials,
			SolvingPlanner& solvingPlanner
		);

		void solveCollisions(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);

		void reportNoCollisions();

		Real computeBodyPenetrationSum() const;

		size_t getMemoryUsage() const override;
	private:
		// Collision constraint solving.

		size_t planWorkerCount(size_t collisionCount) const;

		void computeConstraintData(const std::vector<BodyCollisionData>& collisionDataContainer);

		void applyWarmStartingForCollisions(
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const VelocityConstraintData> constraintDataContainer
		);

		void solveVelocityConstraints(
			bool firstIteration,
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const VelocityConstraintData> constraintDataContainer,
			std::span<const FrictionData> frictionDataContainer
		);

		void solveVelocityConstraintsSequential(
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const VelocityConstraintData> constraintDataContainer,
			std::span<const FrictionData> frictionDataContainer
		);

		void solveVelocityConstraintsBlock(
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const VelocityConstraintData> constraintDataContainer,
			std::span<const FrictionData> frictionDataContainer
		);

		void solvePositionConstraints(
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const PositionConstraintData> constraintDataContainer
		);

		void solveConstraintsThreaded(
			const std::vector<BodyCollisionData>& collisionDataContainer,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	};
}