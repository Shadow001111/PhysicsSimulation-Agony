#pragma once
#include "SolvingPlanner.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

namespace PS_AGONY
{
	struct BodySoA;

	// Common base for the constraint solvers (BodyCollisionsSolver, SpringSolver).
	//
	// Everything that is identical between them lives here:
	//   - the atomic worker bookkeeping for the threaded wave scheduler
	//   - a generic wave-based threaded dispatcher (runThreadedWaves), used by both
	//     solvers whenever they plan their work through SolvingPlanner's graph coloring
	//   - the shared BodySoA viewer and the shared SolvingPlanner instance
	//
	// The SolvingPlanner is *shared* between sibling solvers (set once via
	// setSharedResources by whoever owns it, e.g. a PhysicsWorld) so its memory
	// footprint isn't duplicated per-solver. Since collision solving and spring
	// solving never run concurrently, this is safe.
	class SolverBase
	{
	protected:
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

		// Callback invoked once per (wave, worker) slot for every tick of runThreadedWaves.
		//   waveIndex  - index of the wave within a single pass over the graph coloring.
		//   workerIndex- which worker slot is executing this callback.
		//   passTicket - monotonically increasing tick (0, 1, 2, ...) counted across every
		//                wave of every iteration; lets callers distinguish e.g. velocity-
		//                solving ticks from position-solving ticks, or detect the very first
		//                tick (for warm starting) without maintaining their own counters.
		using WaveTaskFunc = std::function<void(size_t waveIndex, size_t workerIndex, uint32_t passTicket)>;

		BodySoA* bodies = nullptr;
		SolvingPlanner* solvingPlanner = nullptr; // Shared with sibling solver(s); not owned by this class.

		WorkerResources workerResources;

		void setSharedResources(BodySoA& bodiesIn, SolvingPlanner& solvingPlannerIn);

		// Drives `workerCount` workers through `waveCount` waves, `totalTicks` times in total
		// (wave index wraps naturally via passTicket % waveCount). Blocks until every worker
		// has finished all its ticks. Shared by every solver that plans its work through
		// SolvingPlanner's graph-coloring passes, so the wave-scheduling machinery only
		// needs to be written and tuned once.
		void runThreadedWaves(
			size_t workerCount,
			size_t waveCount,
			uint32_t totalTicks,
			const WaveTaskFunc& taskFunc
		);

	public:
		SolverBase() = default;
		virtual ~SolverBase() = default;

		SolverBase(const SolverBase&) = delete;
		SolverBase& operator=(const SolverBase&) = delete;
		SolverBase(SolverBase&&) = delete;
		SolverBase& operator=(SolverBase&&) = delete;

		// Note: intentionally does NOT include solvingPlanner->getMemoryUsage() since the
		// planner is shared between sibling solvers - whoever owns/creates the shared
		// SolvingPlanner should account for it exactly once.
		virtual size_t getMemoryUsage() const = 0;
	};
}