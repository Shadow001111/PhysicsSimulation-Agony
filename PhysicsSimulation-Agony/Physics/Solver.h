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

		struct ResolveCollisionsThreadedResources
		{
			struct alignas(64) WorkerData
			{
				std::vector<size_t> indices;
				std::atomic<bool> isDestroyed{ false };
				std::atomic<uint32_t> workWave{ 0 };

				WorkerData() = default;
				~WorkerData() = default;

				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					indices = std::move(other.indices);
					isDestroyed.store(true, std::memory_order_release);
				}

				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						indices = std::move(other.indices);
						isDestroyed.store(true, std::memory_order_release);
					}
					return *this;
				}
			};

			using UsedSlot = uint8_t;

			static constexpr uint32_t STOP_WAVE = -1;

			std::vector<WorkerData> workerData;

			std::vector<size_t> remainingIndices;
			std::vector<size_t> nextRemainingIndices;
			std::vector<std::vector<size_t>> stagingPasses;
			std::vector<UsedSlot> usedBodies;

			alignas(64) std::atomic<uint32_t> workNotDone{ 0 };
		};


		BodySoA* bodies;
		const std::vector<Material>* materials;

		ResolveCollisionsThreadedResources solverResources;
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

		void solveVelocityConstraints(const std::vector<BodyCollisionData>& narrowPhaseCollisions);
		void solveVelocityConstraintsThreaded(const std::vector<BodyCollisionData>& narrowPhaseCollisions);

		void solvePositionConstraints(const std::vector<BodyCollisionData>& narrowPhaseCollisions, uint32_t solverIterations);
	private:
		void solveVelocityConstraintsIndirect(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<size_t>& collisionIndices
		);
	};
}
