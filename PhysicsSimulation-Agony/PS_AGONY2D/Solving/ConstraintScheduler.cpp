#include "ConstraintScheduler.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <algorithm>

namespace PS_AGONY2D
{
	void ConstraintScheduler::solve(const ConstraintSystemArray& systems, Real deltaTime, BodySoA& bodies)
	{
		TRACY_SCOPE_NC("Solve constraints", PS_AGONY2D::Profiling::Color::OliveDrab);

		active.clear();
		ticks.clear();

		uint32_t maxIterations = 0;
		size_t runWorkers = 1;

		for (IConstraintSystem* system : systems)
		{
			if (system->getCount() == 0) continue;

			system->prepare(bodies);

			IConstraintSolver& solver = system->getSolver();
			if (solver.getWaveCount() == 0) continue;

			active.push_back({ &solver, system->iterations });
			maxIterations = std::max(maxIterations, system->iterations);
			runWorkers = std::max(runWorkers, solver.getWorkerCount());
		}

		if (active.empty()) return;

		ticks.push_back({ nullptr, 0, Tick::Kind::Compute });

		for (const Active& a : active)
		{
			if (!a.solver->hasWarmStart()) continue;
			for (uint32_t w = 0; w < a.solver->getWaveCount(); w++)
			{
				ticks.push_back({ a.solver, w, Tick::Kind::WarmStart });
			}
		}

		for (uint32_t k = 0; k < maxIterations; k++)
		{
			for (const Active& a : active)
			{
				const uint64_t n = a.iterations;
				if ((uint64_t(k + 1) * n) / maxIterations == (uint64_t(k) * n) / maxIterations) continue;

				for (uint32_t w = 0; w < a.solver->getWaveCount(); w++)
				{
					ticks.push_back({ a.solver, w, Tick::Kind::Solve });
				}
			}
		}

		const auto runTick = [this, deltaTime](size_t tickIndex, size_t worker)
			{
				const Tick& tick = ticks[tickIndex];
				switch (tick.kind)
				{
				case Tick::Kind::Compute:
					for (const Active& a : active) a.solver->computeData(deltaTime, worker);
					break;
				case Tick::Kind::WarmStart:
					tick.solver->warmStartWave(tick.wave, worker);
					break;
				case Tick::Kind::Solve:
					tick.solver->solveWave(tick.wave, worker);
					break;
				}
			};

		if (runWorkers <= 1)
		{
			for (size_t t = 0; t < ticks.size(); t++) runTick(t, 0);
			return;
		}

		runner.run(runWorkers, static_cast<uint32_t>(ticks.size()), runTick);
	}

	size_t ConstraintScheduler::getMemoryUsage() const noexcept
	{
		return ticks.capacity() * sizeof(Tick) + active.capacity() * sizeof(Active) + runner.getMemoryUsage();
	}
}
