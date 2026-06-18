#pragma once
#include "NarrowPhaseCollisionDetector.h"
#include "Material.h"

namespace PS_AGONY
{
	class Solver
	{
		struct SimulationSettings
		{
			static constexpr bool ENABLE_VELOCITY_CORRECTION = true;

			// Baumgarte stabilization. No slop.
			const Real positionCorrectionPercent = 1.0;
			const Real velocityCorrectionStrength = 8.0;
		};

		struct ResolveCollisionsThreadedResources
		{
			struct alignas(64) WorkerData
			{
				std::vector<size_t> indices;
				std::atomic<bool> isProcessing{ false };
				std::atomic<bool> isDestroyed{ true };
			};

			using UsedSlot = uint8_t;

			static constexpr size_t MAX_VALID_INDICES_PER_PASS = 256; // Idk which value to pick.
			static constexpr size_t WORKER_COUNT = std::min(12ull, size_t(Threading::MAX_THREADS_ALLOWED));

			std::array<WorkerData, WORKER_COUNT> workerData;

			std::vector<size_t> remainingIndices;
			std::array<std::vector<size_t>, WORKER_COUNT> stagingPasses;
			std::vector<UsedSlot> usedBodies;
		};


		BodySoA* bodies;
		const std::vector<Material>* materials;

		ResolveCollisionsThreadedResources solverResources;

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

		void resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions);
		void resolveCollisionsThreadedGraphColoring(const std::vector<BodyCollisionData>& narrowPhaseCollisions);
	private:
		void resolveCollisionsIndirect(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			const std::vector<size_t>& collisionIndices
		);
	};
}
