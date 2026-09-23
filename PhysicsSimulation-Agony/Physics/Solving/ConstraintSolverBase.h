#pragma once
#include "SolverBase.h"

#include "Physics/SoA/BodySoA.h"
#include "Physics/ContainerUtilities.h"
#include "Physics/Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <algorithm>
#include <new>
#include <numeric>
#include <span>
#include <vector>

namespace PS_AGONY
{
	// Threading thresholds of a constraint solver. Passed by the concrete solver to the base.
	struct ConstraintSolverTuning
	{
		size_t constraintsPerWorker;        // Constraints one worker thread handles.
		size_t minWorkersForThreading = 3;  // Threading starts at constraintsPerWorker * minWorkersForThreading constraints.
	};

	// Shared solving pipeline of all constraint solvers (CRTP, no virtual calls):
	// planning, single/multi-threaded dispatch, wave execution and memory stats.
	//
	// Derived must provide (private is fine if it befriends the base):
	//   void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices);
	//     - fills `constraintContainer` (one ConstraintData per index, in the given order).
	//   void solveVelocityConstraints(SolvingPlan::Pass pass);
	//     - solves constraintContainer[pass.start, pass.start + pass.size).
	// ConstraintData must be defined at namespace scope before the derived class.
	// Viewer must expose getCount(), bodyIndexA, bodyIndexB.
	template<class Derived, class Viewer, class ConstraintData>
	class ConstraintSolverBase : public SolverBase
	{
		ConstraintSolverTuning tuning;

		std::vector<ObjectPair> bodyPairs;
		std::vector<size_t> orderedIndices;
	protected:
		Viewer viewer;
		std::vector<ConstraintData> constraintContainer;

		explicit ConstraintSolverBase(const ConstraintSolverTuning& tuningIn) noexcept : tuning(tuningIn) {}
	public:
		void setDataViewers(BodySoA& bodiesIn, const Viewer& viewerIn)
		{
			setResources(bodiesIn);
			viewer = viewerIn;
		}

		size_t getMemoryUsage() const override
		{
			return
				getVectorMemoryUsage(bodyPairs) +
				getVectorMemoryUsage(orderedIndices) +
				getVectorMemoryUsage(constraintContainer) +
				solvingPlan.getMemoryUsage();
		}

		void solve(Real deltaTime, uint32_t iterations, bool wasSoAChanged)
		{
			Derived& self = static_cast<Derived&>(*this);

			const size_t count = viewer.getCount();
			if (count == 0) return;

			const size_t workerCount = planWorkerCount(count);

			// Single-threaded path.
			if (workerCount <= 1)
			{
				TRACY_SCOPE_NC("Solve constraints (Single-threaded)", Ecstasy::Core::Color::OliveDrab);

				if (orderedIndices.size() != count)
				{
					orderedIndices.resize(count);
					std::iota(orderedIndices.begin(), orderedIndices.end(), size_t(0));
				}

				self.computeConstraintData(deltaTime, orderedIndices);

				const SolvingPlan::Pass fullPass{ 0, static_cast<uint32_t>(count) };
				for (uint32_t i = 0; i < iterations; i++)
				{
					self.solveVelocityConstraints(fullPass);
				}
				return;
			}

			// Multi-threading path.
			TRACY_SCOPE_NC("Solve constraints (Multi-threaded)", Ecstasy::Core::Color::OliveDrab);
			if (wasSoAChanged || solvingPlan.getFlatIndices().size() != count)
			{
				{
					TRACY_SCOPE_NC("Collect body pairs from constraints", Ecstasy::Core::Color::Red);
					bodyPairs.resize(count);
					for (size_t i = 0; i < count; i++)
					{
						bodyPairs[i] = { viewer.bodyIndexA[i], viewer.bodyIndexB[i] };
					}
				}
				{
					TRACY_SCOPE_NC("Plan execution", Ecstasy::Core::Color::Blue);

					SolvingPlanner::threadLocal().planCacheSharingAwareExecution(
						bodyPairs, bodies->getCount(), workerCount,
						sizeof(Real),
						std::hardware_destructive_interference_size,
						solvingPlan
					);
				}
			}
			{
				TRACY_SCOPE_N("Reorder constraint indices");

				const auto& flatIndices = solvingPlan.getFlatIndices();
				orderedIndices.assign(flatIndices.begin(), flatIndices.end());
			}
			self.computeConstraintData(deltaTime, orderedIndices);

			solveThreaded(iterations);
		}
	private:
		size_t planWorkerCount(size_t count) const
		{
			const size_t minCountForThreading = tuning.constraintsPerWorker * tuning.minWorkersForThreading;

			// Single-threaded path.
			if (count < minCountForThreading)
			{
				return 0;
			}

			// Multi-threading path.
			auto& threadPool = Threading::getGlobalThreadPool();

			const size_t availableWorkerCount = threadPool.getThreadCount();
			const size_t neededWorkerCount = count / tuning.constraintsPerWorker;

			return std::min(availableWorkerCount, neededWorkerCount);
		}

		void solveThreaded(uint32_t iterations)
		{
			Derived& self = static_cast<Derived&>(*this);

			const size_t workerCount = solvingPlan.getWorkerCount();

			const auto& passOffsets = solvingPlan.getPassOffsets();
			const size_t waveCount = passOffsets.size() / workerCount;
			const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

			if (waveCount == 0 || iterations == 0) [[unlikely]]
			{
				return;
			}

			const uint32_t totalTicks = static_cast<uint32_t>(waveCount) * iterations;

			runThreadedWaves(workerCount, waveCount, totalTicks,
				[&](size_t waveIndex, size_t workerIndex, uint32_t /*passTicket*/)
				{
					const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
					const auto& passOffset = passOffsetsPtr[passGlobalIndex];

					// Get work from current wave and execute it. If empty, skip.
					if (passOffset.size == 0) return;

					self.solveVelocityConstraints(passOffset);
				}
			);
		}
	};
}
