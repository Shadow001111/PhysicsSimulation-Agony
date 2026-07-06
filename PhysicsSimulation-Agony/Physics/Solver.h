#pragma once
#include "NarrowPhaseCollisionDetector.h"
#include "Material.h"

#include <atomic>
#include <vector>

namespace PS_AGONY
{
	class Solver
	{
		struct PositionAnchor
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

		struct ThreadedConstraintSolvingPlan
		{
			static constexpr size_t COLLISION_COUNT_PER_WORKER = 830;
			static constexpr size_t MIN_COLLISION_COUNT_FOR_THREADING = COLLISION_COUNT_PER_WORKER * 3;
			static constexpr size_t MAX_VALID_INDICES_PER_WORKER = 256;
			static constexpr bool DO_NOT_MARK_STATIC_BODIES_AS_USED = false;

			static_assert(MIN_COLLISION_COUNT_FOR_THREADING >= MAX_VALID_INDICES_PER_WORKER);

			struct Wave
			{
				std::vector<std::vector<size_t>> passes;

				Wave(size_t workerCount)
				{
					passes.resize(workerCount);
				}

				// TODO: Add moving.
			};

			using UsedSlot = uint8_t;

			size_t workerCount = 0;

			std::vector<size_t> remainingIndices;
			std::vector<size_t> nextRemainingIndices;
			std::vector<UsedSlot> usedBodies;
			std::vector<Wave> waves; // TODO: Reduce horrible reallocation issue. Maybe object pool?
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
			std::atomic<uint32_t> workNotDone{ 0 };
			std::atomic<uint32_t> currentWaveTicket{ 0 }; // Monotonically increasing: wave = ticket % waveCount, iteration = ticket / waveCount.
		};
		
		using SolveIndirectFunc = void(Solver::*)(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<size_t>& collisionIndices
			);

		BodySoA* bodies;
		const std::vector<Material>* materials;

		WorkerResources workerResources;
		ThreadedConstraintSolvingPlan threadedPlan;

		std::vector<PositionAnchor> positionAnchors;

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
		void planWorkerCount(size_t collisionCount);
		void planExecutionWithGraphColoring(const std::vector<BodyCollisionData>& narrowPhaseCollisions);

		void computeAnchorPoints(
			std::vector<PositionAnchor>& outPositionAnchors,
			const std::vector<BodyCollisionData>& narrowPhaseCollisions
		);

		void solveVelocityConstraints(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t solverIterations
		);

		void solvePositionConstraints(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<PositionAnchor>& positionAnchors,
			uint32_t solverIterations
		);

		void solveConstraintsThreaded(
			SolveIndirectFunc solveFunc,
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t solverIterations
		);

		void solveVelocityConstraintsIndirect
		(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<size_t>& collisionIndices
		);

		void solvePositionConstraintsIndirect
		(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<size_t>& collisionIndices
		);
	};
}
