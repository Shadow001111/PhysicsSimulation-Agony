#pragma once
#include "NarrowPhaseCollisionDetector.h"
#include "Material.h"

#include "SolvingPlanners/LLSolvingPlanner.h"

#include <atomic>
#include <vector>
#include <span>

namespace PS_AGONY
{
	class Solver
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

		struct WorkerResources
		{
			struct alignas(64) WorkerData
			{
				std::atomic<bool> isDestroyed{ false };

				WorkerData() = default;
				~WorkerData() = default;

				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					isDestroyed.store(true, std::memory_order_release);
				}

				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						isDestroyed.store(true, std::memory_order_release);
					}
					return *this;
				}
			};

			static constexpr uint32_t STOP_WAVE = -1;

			std::vector<WorkerData> workerData;
			alignas(64) std::atomic<uint32_t> workNotDone{ 0 };
			alignas(64) std::atomic<uint32_t> currentWaveTicket{ 0 };
		};

		enum class VelocitySolverType
		{
			ApplyImpulsesSequentially,
			ApplySumOfImpulses,
			//Block
		};

		// Static memeber fields.

		static constexpr VelocitySolverType VELOCITY_SOLVER_TYPE = VelocitySolverType::ApplyImpulsesSequentially;

		// Memeber fields.

		BodySoA* bodies;
		const std::vector<Material>* materials;

		WorkerResources workerResources;
		LLSolvingPlanner solvingPlanner;

		std::vector<BodyPair> collidingBodyPairs;
		std::vector<BodyCollisionData> orderedCollisionData;

		std::vector<PositionConstraintData> positionConstraintContainer;
		std::vector<VelocityConstraintData> velocityConstraintContainer;
		std::vector<FrictionData> frictionDataContainer;

		SimulationSettings simulationSettings;
	public:
		Solver() = default;
		~Solver() = default;
		Solver(const Solver&) = delete;
		Solver& operator=(const Solver&) = delete;
		Solver(Solver&&) = delete;
		Solver& operator=(Solver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const std::vector<Material>& materials
		);

		size_t getMemoryUsage() const;

		void solve(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);

		void solveThreaded(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	private:
		size_t planWorkerCount(size_t collisionCount);

		void computeConstantData(const std::vector<BodyCollisionData>& collisionDataContainer);

		void applyWarmStarting(
			std::span<const BodyCollisionData> collisionDataContainer,
			std::span<const VelocityConstraintData> constraintDataContainer
		);

		void solveVelocityConstraints(
			bool firstIteration,
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
