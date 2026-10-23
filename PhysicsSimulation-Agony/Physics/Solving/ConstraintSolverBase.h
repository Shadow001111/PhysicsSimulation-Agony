#pragma once
#include "SolverBase.h"

#include "Physics/SoA/BodySoA.h"
#include "Physics/ContainerUtilities.h"
#include "Physics/Threading.h"

#include "Ecstasy/Core/Portablity.h"

#include <algorithm>
#include <new>
#include <numeric>
#include <span>
#include <vector>

namespace PS_AGONY2D
{
	struct ConstraintSolverTuning
	{
		size_t constraintsPerWorker;
		size_t minWorkersForThreading = 3;
	};

	// What ConstraintScheduler needs from a solver. Every call is made by all workers of the
	// scheduler's current run; a solver with fewer workers than the run ignores the extra ones.
	class IConstraintSolver
	{
	public:
		virtual ~IConstraintSolver() = default;

		virtual size_t getWorkerCount() const noexcept = 0; // Workers this solver uses (1 = single-threaded).
		virtual size_t getWaveCount() const noexcept = 0;   // 0 = nothing to solve.
		virtual bool hasWarmStart() const noexcept = 0;

		virtual void computeData(Real deltaTime, size_t workerIndex) = 0;      // Read-only on bodies, so slices run in parallel.
		virtual void warmStartWave(size_t waveIndex, size_t workerIndex) = 0;  // Writes bodies, uses the solving plan.
		virtual void solveWave(size_t waveIndex, size_t workerIndex) = 0;      // One velocity iteration of one wave.
	};

	// Shared solving logic. Derived provides:
	//   BodyRef makeBodyRef(size_t i) const;
	//   void planExecution(size_t workerCount);
	//   void computeConstraintData(Real, std::span<const size_t> indices, size_t outOffset);
	//   void solveVelocityConstraints(const SolvingPlan::Pass&);
	//   optional: void warmStart(const SolvingPlan::Pass&);
	template<class Derived, class Viewer, class ConstraintData, class BodyRef>
	class ConstraintSolverBase : public SolverBase, public IConstraintSolver
	{
		ConstraintSolverTuning tuning;

		std::vector<size_t> identityOrder; // Single-threaded order.
		std::span<const size_t> order;     // Active order: identityOrder, or the plan's flat indices.

		size_t plannedCount = 0;
		size_t plannedWorkerCount = 0;
		bool planValid = false;

		bool threaded = false;
		size_t workerCount = 1;
		size_t waveCount = 0;

	protected:
		Viewer viewer;
		std::vector<ConstraintData> constraintContainer; // Indexed by position in `order`.
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
				getVectorMemoryUsage(identityOrder) +
				getVectorMemoryUsage(constraintContainer) +
				solvingPlan.getMemoryUsage();
		}

		// Chooses the execution mode and replans only when this type changed.
		void prepare(bool wasSoAChanged)
		{
			Derived& self = static_cast<Derived&>(*this);

			const size_t count = viewer.getCount();
			if (count == 0)
			{
				order = {};
				threaded = false;
				workerCount = 1;
				waveCount = 0;
				planValid = false;
				return;
			}

			const size_t plannedWorkers = planWorkerCount(count);

			if (plannedWorkers <= 1)
			{
				if (identityOrder.size() != count)
				{
					identityOrder.resize(count);
					std::iota(identityOrder.begin(), identityOrder.end(), size_t(0));
				}
				order = identityOrder;
				threaded = false;
				workerCount = 1;
				waveCount = 1;
				planValid = false; // Plan (if any) is stale once we skip changes in single-threaded mode.
				constraintContainer.resize(count);
				return;
			}

			if (!planValid || wasSoAChanged || plannedCount != count || plannedWorkerCount != plannedWorkers)
			{
				bodyRefs.resize(count);
				for (size_t i = 0; i < count; i++)
				{
					bodyRefs[i] = self.makeBodyRef(i);
				}
				self.planExecution(plannedWorkers);

				plannedCount = count;
				plannedWorkerCount = plannedWorkers;
				planValid = true;
			}

			threaded = true;
			workerCount = solvingPlan.getWorkerCount();
			order = solvingPlan.getFlatIndices(); // May be shorter than count: all-static constraints are not planned.
			waveCount = workerCount > 0 ? solvingPlan.getPassOffsets().size() / workerCount : 0;
			constraintContainer.resize(order.size());
		}

		size_t getWorkerCount() const noexcept override { return workerCount; }
		size_t getWaveCount() const noexcept override { return waveCount; }

		bool hasWarmStart() const noexcept override
		{
			return requires(Derived & d, SolvingPlan::Pass p) { d.warmStart(p); };
		}

		void computeData(Real deltaTime, size_t workerIndex) override
		{
			if (workerIndex >= workerCount) return;

			const size_t n = order.size();
			const size_t begin = n * workerIndex / workerCount;
			const size_t end = n * (workerIndex + 1) / workerCount;
			if (begin == end) return;

			static_cast<Derived&>(*this).computeConstraintData(deltaTime, order.subspan(begin, end - begin), begin);
		}

		void warmStartWave(size_t waveIndex, size_t workerIndex) override
		{
			if constexpr (requires(Derived & d, SolvingPlan::Pass p) { d.warmStart(p); })
			{
				SolvingPlan::Pass pass;
				if (!getPass(waveIndex, workerIndex, pass)) return;
				static_cast<Derived&>(*this).warmStart(pass);
			}
		}

		void solveWave(size_t waveIndex, size_t workerIndex) override
		{
			SolvingPlan::Pass pass;
			if (!getPass(waveIndex, workerIndex, pass)) return;
			static_cast<Derived&>(*this).solveVelocityConstraints(pass);
		}

	private:
		bool getPass(size_t waveIndex, size_t workerIndex, SolvingPlan::Pass& out) const
		{
			if (workerIndex >= workerCount) return false;

			if (!threaded)
			{
				out = { 0, static_cast<uint32_t>(order.size()) };
				return out.size != 0;
			}

			out = solvingPlan.getPassOffsets()[waveIndex * workerCount + workerIndex];
			return out.size != 0;
		}

		size_t planWorkerCount(size_t count) const
		{
			const size_t minCountForThreading = tuning.constraintsPerWorker * tuning.minWorkersForThreading;

			if (count < minCountForThreading)
			{
				return 0;
			}

			auto& threadingBackend = Threading::getThreadingBackend();

			const size_t availableWorkerCount = threadingBackend.getThreadCount() + 1; // Include main thread.
			const size_t neededWorkerCount = count / tuning.constraintsPerWorker;

			return std::min(availableWorkerCount, neededWorkerCount);
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
				this->solvingPlan,
				this->bodies->isStatic.data()
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
				this->solvingPlan,
				this->bodies->isStatic.data()
			);
		}
	};
}