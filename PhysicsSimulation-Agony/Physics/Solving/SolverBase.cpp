#include "SolverBase.h"
#include "../Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

namespace PS_AGONY
{
	void SolverBase::setResources(BodySoA& bodiesIn, SolvingPlanner& solvingPlannerIn)
	{
		bodies = &bodiesIn;
		solvingPlanner = &solvingPlannerIn;
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

		auto& threadPool = Threading::getGlobalThreadPool();

		// Worker data.
		workerResources.workerData.resize(workerCount);
		for (auto& w : workerResources.workerData)
		{
			w.isDestroyed.store(false, std::memory_order_release);
		}

		workerResources.currentWaveTicket.store(0, std::memory_order_release);
		workerResources.workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);

		auto workerFunc = [&](size_t workerIndex)
			{
				auto& wData = workerResources.workerData[workerIndex];

				uint32_t localTicket = 0;
				while (true)
				{
					// All waves x all iterations done -> stop.
					if (localTicket >= totalTicks) break;

					const size_t waveIndex = localTicket % waveCount;
					
					taskFunc(waveIndex, workerIndex, localTicket);

					// Decrease atomic counter; last one to finish advances the wave and wakes everyone.
					const uint32_t remaining = workerResources.workNotDone.fetch_sub(1, std::memory_order_acq_rel) - 1;
					if (remaining == 0)
					{
						workerResources.workNotDone.store(static_cast<uint32_t>(workerCount), std::memory_order_release);
						workerResources.currentWaveTicket.fetch_add(1, std::memory_order_release);
						workerResources.currentWaveTicket.notify_all();
						localTicket++;
					}
					else
					{
						// Wait for new wave.
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

					// Loop naturally wraps back to wave 0 via (localTicket % waveCount) until totalTicks is hit.
				}

				wData.isDestroyed.store(true, std::memory_order_release);
				wData.isDestroyed.notify_one();
			};

		for (size_t i = 1; i < workerCount; i++)
		{
			threadPool.enqueue(workerFunc, i);
		}
		workerFunc(0);

		// Wait for all workers to finish every iteration and terminate.
		{
			TRACY_SCOPE_NC("Wait for workers to finish", Ecstasy::Color::Brown);
			for (size_t i = 1; i < workerResources.workerData.size(); i++)
			{
				workerResources.workerData[i].isDestroyed.wait(false, std::memory_order_acquire);
			}
		}
	}
}