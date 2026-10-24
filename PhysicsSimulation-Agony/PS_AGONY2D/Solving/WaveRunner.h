#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace PS_AGONY2D
{
	// Runs `tickCount` ticks on `workerCount` workers with one barrier per tick. The caller's
	// thread is worker 0; the others come from the global thread pool. Blocks until all ticks are done.
	class WaveRunner
	{
		struct alignas(64) WorkerData
		{
			std::atomic<bool> isDestroyed{ false };

			WorkerData() = default;
			WorkerData(const WorkerData&) = delete;
			WorkerData& operator=(const WorkerData&) = delete;
			WorkerData(WorkerData&&) noexcept { isDestroyed.store(true, std::memory_order_release); }
			WorkerData& operator=(WorkerData&&) noexcept { isDestroyed.store(true, std::memory_order_release); return *this; }
		};

		using TaskFn = void(*)(void* context, size_t tick, size_t worker);

		std::vector<WorkerData> workerData;
		alignas(64) std::atomic<uint32_t> workNotDone{ 0 };
		alignas(64) std::atomic<uint32_t> currentTicket{ 0 };

		void runImpl(size_t workerCount, uint32_t tickCount, TaskFn fn, void* context);
	public:
		template<class F>
		void run(size_t workerCount, uint32_t tickCount, F&& task)
		{
			runImpl(workerCount, tickCount,
				[](void* c, size_t tick, size_t worker) { (*static_cast<std::remove_reference_t<F>*>(c))(tick, worker); },
				const_cast<void*>(static_cast<const void*>(&task)));
		}

		size_t getMemoryUsage() const noexcept { return workerData.capacity() * sizeof(WorkerData); }
	};
}