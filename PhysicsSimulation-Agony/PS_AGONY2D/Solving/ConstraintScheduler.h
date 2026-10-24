#pragma once
#include "PS_AGONY2D/ConstraintSystem.h"
#include "WaveRunner.h"

#include <vector>

namespace PS_AGONY2D
{
	// Solves all constraint types together. Each type keeps its own solving plan (replanned only when
	// that type changes); the scheduler only interleaves their iterations:
	//
	//   tick 0                : parallel constraint-data computation for every type (read-only)
	//   warm start ticks      : one per wave, for types that warm start (uses the type's plan)
	//   iteration k = 0..max-1: for each type active at k, its waves in order (type order)
	//
	// Type t with n_t iterations is active at k when floor((k+1)*n_t/max) > floor(k*n_t/max),
	// which spreads its iterations evenly between the others. The whole step is one
	// WaveRunner run: one fork/join, one barrier per tick, and one solver call per worker per tick.
	class ConstraintScheduler
	{
		struct Tick
		{
			enum class Kind : uint8_t { Compute, WarmStart, Solve };

			IConstraintSolver* solver;
			uint32_t wave;
			Kind kind;
		};

		struct Active
		{
			IConstraintSolver* solver;
			uint32_t iterations;
		};

		std::vector<Tick> ticks;
		std::vector<Active> active;
		WaveRunner runner;
	public:
		void solve(const ConstraintSystemArray& systems, Real deltaTime, BodySoA& bodies);

		size_t getMemoryUsage() const noexcept;
	};
}
