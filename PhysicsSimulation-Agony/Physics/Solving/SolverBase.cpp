#include "SolverBase.h"

#include "Physics/Threading.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
	void SolverBase::setResources(BodySoA& bodiesIn)
	{
		bodies = &bodiesIn;
	}

	void SolverBase::runThreadedWaves(
		size_t workerCount,
		size_t waveCount,
		uint32_t totalTicks,
		const WaveTaskFunc& taskFunc
	)
	{
		if (waveCount == 0 || totalTicks == 0) [[unlikely]]
		{
			return;
		}

		auto& threadingBackend = Threading::getThreadingBackend();

		// workerCount is the requested participant count (pool + main). Clamp to what the pool can serve.
		const size_t poolWorkers = threadingBackend.getThreadCount();
		const size_t launchCount = workerCount > 1 ? std::min(poolWorkers, workerCount - 1) : 0;
		const size_t participants = launchCount + 1;

		// Never shrink. Reset only active slots (slot 0 included).
		if (workerResources.workerData.size() < participants)
		{
			workerResources.workerData.resize(participants);
		}
		for (size_t i = 0; i < participants; i++)
		{
			workerResources.workerData[i].isDestroyed.store(false, std::memory_order_release);
		}

		workerResources.currentWaveTicket.store(0, std::memory_order_release);
		workerResources.workNotDone.store(static_cast<uint32_t>(participants), std::memory_order_release);

		auto workerFunc = [&](size_t workerIndex)
			{
				uint32_t localTicket = 0;
				while (localTicket < totalTicks)
				{
					const size_t waveIndex = localTicket % waveCount;

					taskFunc(waveIndex, workerIndex, localTicket);

					const uint32_t remaining = workerResources.workNotDone.fetch_sub(1, std::memory_order_acq_rel) - 1;
					if (remaining == 0)
					{
						workerResources.workNotDone.store(static_cast<uint32_t>(participants), std::memory_order_release);
						workerResources.currentWaveTicket.fetch_add(1, std::memory_order_release);
						workerResources.currentWaveTicket.notify_all();
						localTicket++;
					}
					else
					{
						for (int i = 0; i < 2'000; i++)
						{
							ECSTASY_SPIN_PAUSE();
							if (workerResources.currentWaveTicket.load(std::memory_order_acquire) != localTicket)
								break;
						}

						if (workerResources.currentWaveTicket.load(std::memory_order_acquire) == localTicket)
						{
							workerResources.currentWaveTicket.wait(localTicket, std::memory_order_acquire);
						}

						localTicket = workerResources.currentWaveTicket.load(std::memory_order_acquire);
					}
				}

				// Main (slot 0) never signals itself.
				if (workerIndex != 0)
				{
					auto& wData = workerResources.workerData[workerIndex];
					wData.isDestroyed.store(true, std::memory_order_release);
					wData.isDestroyed.notify_one();
				}
			};

		if (launchCount > 0)
		{
			std::vector<uint32_t> threadIndices(launchCount);
			for (uint32_t i = 0; i < launchCount; i++)
			{
				threadIndices[i] = i;
			}

			threadingBackend.launch(
				launchCount,
				[](void* ctx, size_t i) { (*static_cast<decltype(workerFunc)*>(ctx))(i + 1); },
				&workerFunc,
				threadIndices
			);
		}

		workerFunc(0);

		{
			TRACY_SCOPE_NC("Wait for workers to finish", PS_AGONY2D::Profiling::Color::Brown);
			for (size_t i = 1; i < participants; i++)
			{
				workerResources.workerData[i].isDestroyed.wait(false, std::memory_order_acquire);
			}
		}
	}
}