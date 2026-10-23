#include "WaveRunner.h"

#include "Physics/Threading.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
	void WaveRunner::runImpl(size_t workerCount, uint32_t tickCount, TaskFn fn, void* context)
	{
		if (tickCount == 0 || workerCount == 0) [[unlikely]]
		{
			return;
		}

		auto& threadingBackend = Threading::getThreadingBackend();

		workerData.resize(workerCount);
		for (auto& w : workerData)
		{
			w.isDestroyed.store(false, std::memory_order_release);
		}

		currentTicket.store(0, std::memory_order_release);
		workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);

		auto workerFunc = [&](size_t workerIndex)
			{
				uint32_t localTicket = 0;
				while (localTicket < tickCount)
				{
					fn(context, localTicket, workerIndex);

					// Last worker to finish the tick advances it and wakes everyone.
					const uint32_t remaining = workNotDone.fetch_sub(1, std::memory_order_acq_rel) - 1;
					if (remaining == 0)
					{
						workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);
						currentTicket.fetch_add(1, std::memory_order_release);
						currentTicket.notify_all();
						localTicket++;
					}
					else
					{
						for (int i = 0; i < 2'000; i++)
						{
							ECSTASY_SPIN_PAUSE();
							if (currentTicket.load(std::memory_order_acquire) != localTicket)
								break;
						}

						if (currentTicket.load(std::memory_order_acquire) == localTicket)
						{
							currentTicket.wait(localTicket, std::memory_order_acquire);
						}

						localTicket = currentTicket.load(std::memory_order_acquire);
					}
				}

				// Main (slot 0) never signals itself.
				if (workerIndex != 0)
				{
					workerData[workerIndex].isDestroyed.store(true, std::memory_order_release);
					workerData[workerIndex].isDestroyed.notify_one();
				}
			};

		// workerCount = participants (pool + main). Main takes slot 0.
		const size_t launchCount = workerCount - 1;
		if (launchCount > 0)
		{
			// Backend index i is 0-based -> pool slots 1..launchCount.
			threadingBackend.launch(
				launchCount,
				[](void* ctx, size_t i) { (*static_cast<decltype(workerFunc)*>(ctx))(i + 1); },
				&workerFunc
			);
		}
		workerFunc(0);

		{
			TRACY_SCOPE_NC("Wait for workers to finish", PS_AGONY2D::Profiling::Color::Brown);
			for (size_t i = 1; i < workerCount; i++)
			{
				workerData[i].isDestroyed.wait(false, std::memory_order_acquire);
			}
		}
	}
}