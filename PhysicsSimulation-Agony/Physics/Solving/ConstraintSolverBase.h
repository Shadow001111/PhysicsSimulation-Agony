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
	struct ConstraintSolverTuning
	{
		size_t constraintsPerWorker;
		size_t minWorkersForThreading = 3;
	};

	// Shared solving logic. Derived provides:
	//   BodyRef makeBodyRef(size_t i) const;
	//   void planExecution(size_t workerCount);
	//   void computeConstraintData(Real, std::span/vector indices);
	//   void solveVelocityConstraints(const SolvingPlan::Pass&);
	template<class Derived, class Viewer, class ConstraintData, class BodyRef>
	class ConstraintSolverBase : public SolverBase
	{
		ConstraintSolverTuning tuning;
		std::vector<size_t> orderedIndices;

	protected:
		Viewer viewer;
		std::vector<ConstraintData> constraintContainer;
		std::vector<BodyRef> bodyRefs;

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
				getVectorMemoryUsage(bodyRefs) +
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

			TRACY_SCOPE_NC("Solve constraints (Multi-threaded)", Ecstasy::Core::Color::OliveDrab);
			if (wasSoAChanged || solvingPlan.getFlatIndices().size() != count)
			{
				{
					TRACY_SCOPE_NC("Collect body refs from constraints", Ecstasy::Core::Color::Red);
					bodyRefs.resize(count);
					for (size_t i = 0; i < count; i++)
					{
						bodyRefs[i] = self.makeBodyRef(i);
					}
				}
				{
					TRACY_SCOPE_NC("Plan execution", Ecstasy::Core::Color::Blue);
					self.planExecution(workerCount);
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

			if (count < minCountForThreading)
			{
				return 0;
			}

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

					if (passOffset.size == 0) return;

					self.solveVelocityConstraints(passOffset);
				}
			);
		}
	};

	template<class Derived, class Viewer, class ConstraintData>
	class PairConstraintSolverBase : public ConstraintSolverBase<Derived, Viewer, ConstraintData, ObjectPair>
	{
	protected:
		using ConstraintSolverBase<Derived, Viewer, ConstraintData, ObjectPair>::ConstraintSolverBase;

	public:
		ObjectPair makeBodyRef(size_t i) const
		{
			return { this->viewer.bodyIndexA[i], this->viewer.bodyIndexB[i] };
		}

		void planExecution(size_t workerCount)
		{
			SolvingPlanner::threadLocal().planCacheSharingAwareExecution(
				this->bodyRefs, this->bodies->getCount(), workerCount,
				sizeof(Real),
				std::hardware_destructive_interference_size,
				this->solvingPlan
			);
		}
	};

	template<class Derived, class Viewer, class ConstraintData>
	class SingleConstraintSolverBase : public ConstraintSolverBase<Derived, Viewer, ConstraintData, ObjectIndex>
	{
	protected:
		using ConstraintSolverBase<Derived, Viewer, ConstraintData, ObjectIndex>::ConstraintSolverBase;

	public:
		ObjectIndex makeBodyRef(size_t i) const
		{
			return this->viewer.bodyIndex[i];
		}

		void planExecution(size_t workerCount)
		{
			SolvingPlanner::threadLocal().planSingleStandardExecution(
				this->bodyRefs, this->bodies->getCount(), workerCount,
				this->solvingPlan
			);
		}
	};
}