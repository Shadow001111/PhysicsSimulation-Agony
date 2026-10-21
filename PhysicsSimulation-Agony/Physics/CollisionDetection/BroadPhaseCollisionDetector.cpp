#include "BroadPhaseCollisionDetector.h"

#include "BroadPhaseInternal/LeafPairTesting.h"
#include "BroadPhaseInternal/BvhPairTraversal.h"

#include "Physics/Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"

#include <algorithm>
#include <array>

namespace PS_AGONY
{
	namespace BPI = BroadPhaseInternal;

	namespace
	{
		constexpr size_t THREADING_NODE_THRESHOLD = 400;
		constexpr size_t CROSS_TASKS_PER_WORKER = 4;
		constexpr size_t TASKS_PER_WORKER = 16; // Per task category; more tasks = better balance.

		// Runs leaf tests for one thread and batches the resulting pairs.
		struct LeafTester
		{
			const BPI::BvhTreeView& dyn;
			const BPI::BvhTreeView& stat;
			const ObjectIndex* owner;
			std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;
			BPI::PairPushBuffer pushBuffer;

			LeafTester(
				const BPI::BvhTreeView& dynIn,
				const BPI::BvhTreeView& statIn,
				const ObjectIndex* ownerIn,
				std::vector<ObjectPair>& dest) noexcept :
				dyn(dynIn), stat(statIn), owner(ownerIn), pushBuffer(dest)
			{
			}

			void self(uint32_t leaf)
			{
				BPI::testLeafSelfCross(dyn.nodes[leaf], dyn, owner, masks.data(), pushBuffer);
			}

			void selfPair(const BvhNodePair& p)
			{
				BPI::testLeafPairCross<true>(dyn.nodes[p.a], dyn, dyn.nodes[p.b], dyn, owner, masks.data(), pushBuffer);
			}

			void crossPair(const BvhNodePair& p)
			{
				BPI::testLeafPairCross<false>(dyn.nodes[p.a], dyn, stat.nodes[p.b], stat, nullptr, masks.data(), pushBuffer);
			}

			void flush()
			{
				if (pushBuffer.size > 0) pushBuffer.flush();
			}
		};
	}

	void BroadPhaseCollisionDetector::setDataViewers(const AABBSoAViewer& aabbs, const ObjectIndex* colliderBodyIndexIn)
	{
		collidersAABB = aabbs;
		colliderBodyIndex = colliderBodyIndexIn;
	}

	void BroadPhaseCollisionDetector::clearData()
	{
		collisionData.clear();
		for (auto& tree : trees)
		{
			tree.clear();
		}
		staticRebuildRequested = true;
	}

	void BroadPhaseCollisionDetector::buildTrees(
		std::span<const ColliderIndex> dynamicIds,
		std::span<const ColliderIndex> staticIds,
		StaticTreeUpdate staticUpdate)
	{
		TRACY_SCOPE_N("Build broad-phase trees");

		trees[0].build(collidersAABB, dynamicIds);

		const bool rebuildStatic =
			staticUpdate == StaticTreeUpdate::Rebuild ||
			staticRebuildRequested ||
			trees[1].colliderCount() != staticIds.size();

		if (rebuildStatic)
		{
			trees[1].build(collidersAABB, staticIds);
			staticRebuildRequested = false;
		}
		else if (staticUpdate == StaticTreeUpdate::Refit)
		{
			trees[1].refit(collidersAABB);
		}
	}

	const std::vector<ObjectPair>& BroadPhaseCollisionDetector::findCollisions(ExecutionPolicy executionPolicy)
	{
		TRACY_SCOPE_N("Find broad-phase collisions");

		collisionData.clear();

		if (trees[0].empty()) return collisionData;

		collisionData.reserve(trees[0].colliderCount());

		const bool useThreading =
			executionPolicy == ExecutionPolicy::ForceMultiThreaded ||
			(executionPolicy != ExecutionPolicy::ForceSingleThreaded &&
				trees[0].nodeCount() + trees[1].nodeCount() > THREADING_NODE_THRESHOLD);

		if (useThreading)
		{
			queryPairsThreaded();
		}
		else
		{
			queryPairs();
		}

		return collisionData;
	}

	void BroadPhaseCollisionDetector::getLeafNodeAABBs(BroadPhaseInternal::BvhTreeTypeFlag flags, std::vector<AABB>& outAABBs) const
	{
		uint32_t mask = static_cast<uint32_t>(flags);
		while (mask)
		{
			const uint32_t treeIndex = std::countr_zero(mask);
			mask &= mask - 1;
			trees[treeIndex].getLeafNodeAABBs(outAABBs);
		}
	}

	size_t BroadPhaseCollisionDetector::getMemoryUsage() const
	{
		size_t total = sizeof(BroadPhaseCollisionDetector);

		total += trees[0].getMemoryUsage();
		total += trees[1].getMemoryUsage();

		total += getVectorMemoryUsage(queryResources.selfNodePairs);
		total += getVectorMemoryUsage(queryResources.selfLeafPairs);
		total += getVectorMemoryUsage(queryResources.selfLeaves);
		total += getVectorMemoryUsage(queryResources.crossNodePairs);
		total += getVectorMemoryUsage(queryResources.crossLeafPairs);

		total += getVectorMemoryUsage(queryPairsThreadedResources.workerData);
		total += getVectorMemoryUsage(queryPairsThreadedResources.tasks);
		total += getVectorMemoryUsage(queryPairsThreadedResources.taskOutputs);
		total += getVectorMemoryUsage(queryPairsThreadedResources.crossScratch);
		for (const auto& wData : queryPairsThreadedResources.workerData)
		{
			total += getVectorMemoryUsage(wData.nodePairsToTraverse);
			total += getVectorMemoryUsage(wData.selfLeafPairs);
			total += getVectorMemoryUsage(wData.crossLeafPairs);
			total += getVectorMemoryUsage(wData.collisionData);
		}

		total += getVectorMemoryUsage(collisionData);
		return total;
	}

	void BroadPhaseCollisionDetector::queryPairs()
	{
		TRACY_SCOPE_N("Query pairs");

		const BPI::BvhTreeView dyn = trees[0].view();
		const BPI::BvhTreeView stat = trees[1].view();
		const bool hasStatic = !trees[1].empty();

		auto& r = queryResources;
		r.selfNodePairs.clear();
		r.selfLeafPairs.clear();
		r.selfLeaves.clear();
		r.crossNodePairs.clear();
		r.crossLeafPairs.clear();

		{
			TRACY_SCOPE_N("Traverse");

			BPI::traverseRootForInitialPairs(dyn.nodes, r.selfNodePairs, r.selfLeafPairs, r.selfLeaves);
			BPI::traverseNodePairStack(dyn.nodes, dyn.nodes, r.selfNodePairs, r.selfLeafPairs);

			if (hasStatic && dyn.nodes[0].overlaps(stat.nodes[0]))
			{
				r.crossNodePairs.push_back({ 0, 0 });
				BPI::traverseNodePairStack(dyn.nodes, stat.nodes, r.crossNodePairs, r.crossLeafPairs);
			}
		}

		{
			TRACY_SCOPE_N("Test collisions in leaves");

			LeafTester tester(dyn, stat, colliderBodyIndex, collisionData);

			for (const uint32_t leaf : r.selfLeaves) tester.self(leaf);
			for (const BvhNodePair& p : r.selfLeafPairs) tester.selfPair(p);
			for (const BvhNodePair& p : r.crossLeafPairs) tester.crossPair(p);

			tester.flush();
		}
	}

	void BroadPhaseCollisionDetector::queryPairsThreaded()
	{
		TRACY_SCOPE_N("Query pairs (Threaded)");

		auto& threadingBackend = Threading::getThreadingBackend();
		const size_t workerCount = threadingBackend.getThreadCount() > 0 ? threadingBackend.getThreadCount() - 1 : 0;
		if (workerCount == 0)
		{
			queryPairs();
			return;
		}

		using TK = QueryPairsThreadedResources::TaskKind;
		auto& tr = queryPairsThreadedResources;

		const BPI::BvhTreeView dyn = trees[0].view();
		const BPI::BvhTreeView stat = trees[1].view();
		const bool hasStatic = !trees[1].empty();

		auto& r = queryResources;
		r.selfNodePairs.clear();
		r.selfLeafPairs.clear();
		r.selfLeaves.clear();
		r.crossNodePairs.clear();
		r.crossLeafPairs.clear();

		// Initial frontiers (same as single-threaded, so ordering is shared).
		{
			TRACY_SCOPE_N("Initial traverse");

			BPI::traverseRootForInitialPairs(dyn.nodes, r.selfNodePairs, r.selfLeafPairs, r.selfLeaves);

			if (hasStatic)
			{
				BPI::expandCrossRootPairsOrdered(
					dyn.nodes, stat.nodes, workerCount * CROSS_TASKS_PER_WORKER,
					r.crossNodePairs, tr.crossScratch);
			}
		}

		// Ordered task list: selfLeaves, selfLeafPairs, selfNodePairs (reverse), cross.
		tr.tasks.clear();
		auto addRanges = [&](TK kind, size_t n, bool reverse)
			{
				if (n == 0) return;
				const size_t divisor = workerCount * TASKS_PER_WORKER;
				const size_t chunk = std::max<size_t>(1, (n + divisor - 1) / divisor);
				const size_t first = tr.tasks.size();
				for (size_t b = 0; b < n; b += chunk)
				{
					tr.tasks.push_back({ kind, static_cast<uint32_t>(b), static_cast<uint32_t>(std::min(b + chunk, n)) });
				}
				if (reverse) std::reverse(tr.tasks.begin() + first, tr.tasks.end());
			};

		addRanges(TK::SelfLeaves, r.selfLeaves.size(), false);
		addRanges(TK::SelfLeafPairs, r.selfLeafPairs.size(), false);
		addRanges(TK::SelfNodes, r.selfNodePairs.size(), true);
		addRanges(TK::CrossNodes, r.crossNodePairs.size(), false);

		const uint32_t taskCount = static_cast<uint32_t>(tr.tasks.size());
		if (taskCount == 0) return;

		tr.taskOutputs.resize(taskCount);

		const size_t launchCount = std::min<size_t>(workerCount, taskCount);
		if (tr.workerData.size() < launchCount) tr.workerData.resize(launchCount); // Never shrink: keeps buffers.

		alignas(64) std::atomic<uint32_t> globalWorkerIndex{ 0 };
		alignas(64) std::atomic<uint32_t> nextTask{ 0 };
		{
			TRACY_SCOPE_N("Launch workers");

			auto workerFunc = [&]()
				{
					TRACY_SCOPE_N("Query pairs worker thread");

					const auto workerIndex = globalWorkerIndex.fetch_add(1, std::memory_order_relaxed);
					auto& wData = tr.workerData[workerIndex];

					LeafTester tester(dyn, stat, colliderBodyIndex, wData.collisionData);

					for (;;)
					{
						const uint32_t t = nextTask.fetch_add(1, std::memory_order_relaxed);
						if (t >= taskCount) break;

						const auto& task = tr.tasks[t];
						const uint32_t outBegin = static_cast<uint32_t>(wData.collisionData.size());

						switch (task.kind)
						{
						case TK::SelfLeaves:
							for (uint32_t i = task.begin; i < task.end; i++)
								tester.self(r.selfLeaves[i]);
							break;

						case TK::SelfLeafPairs:
							for (uint32_t i = task.begin; i < task.end; i++)
								tester.selfPair(r.selfLeafPairs[i]);
							break;

						case TK::SelfNodes:
							// Descending: matches the single-threaded stack pop order.
							for (uint32_t i = task.end; i-- > task.begin; )
							{
								wData.nodePairsToTraverse.clear();
								wData.selfLeafPairs.clear();
								wData.nodePairsToTraverse.push_back(r.selfNodePairs[i]);
								BPI::traverseNodePairStack(dyn.nodes, dyn.nodes, wData.nodePairsToTraverse, wData.selfLeafPairs);
								for (const BvhNodePair& p : wData.selfLeafPairs) tester.selfPair(p);
							}
							break;

						case TK::CrossNodes:
							for (uint32_t i = task.begin; i < task.end; i++)
							{
								wData.nodePairsToTraverse.clear();
								wData.crossLeafPairs.clear();
								wData.nodePairsToTraverse.push_back(r.crossNodePairs[i]);
								BPI::traverseNodePairStack(dyn.nodes, stat.nodes, wData.nodePairsToTraverse, wData.crossLeafPairs);
								for (const BvhNodePair& p : wData.crossLeafPairs) tester.crossPair(p);
							}
							break;
						}

						tester.flush(); // Task output must be contiguous and complete in collisionData.
						tr.taskOutputs[t] = { workerIndex, outBegin, static_cast<uint32_t>(wData.collisionData.size()) };
					}

					wData.isDone.store(true, std::memory_order_release);
					wData.isDone.notify_one();
				};

			for (size_t i = 0; i < launchCount; i++)
			{
				auto& wData = tr.workerData[i];
				wData.nodePairsToTraverse.clear();
				wData.selfLeafPairs.clear();
				wData.crossLeafPairs.clear();
				wData.collisionData.clear();
				wData.isDone = false;
			}

			threadingBackend.launch(
				launchCount,
				[](void* ctx, size_t) { (*static_cast<decltype(workerFunc)*>(ctx))(); },
				&workerFunc
			);
		}

		// Wait for all workers, then concatenate task outputs in canonical task order.
		{
			TRACY_SCOPE_N("Wait for workers to finish and combine data");

			for (size_t i = 0; i < launchCount; i++)
			{
				tr.workerData[i].isDone.wait(false, std::memory_order_acquire);
			}

		}
		{
			TRACY_SCOPE_N("Combine");

			size_t total = 0;
			for (const auto& o : tr.taskOutputs) total += o.end - o.begin;
			collisionData.reserve(total);

			for (const auto& o : tr.taskOutputs)
			{
				if (o.end == o.begin) continue;
				const auto& src = tr.workerData[o.worker].collisionData;
				collisionData.insert(collisionData.end(), src.begin() + o.begin, src.begin() + o.end);
			}
		}
	}
}