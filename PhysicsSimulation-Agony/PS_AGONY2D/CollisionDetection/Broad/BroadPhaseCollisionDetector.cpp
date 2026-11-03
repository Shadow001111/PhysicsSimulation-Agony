#include "BroadPhaseCollisionDetector.h"
#include "LeafPairTesting.h"
#include "BvhPairTraversal.h"

#include "PS_AGONY2D/Threading.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <algorithm>
#include <array>
#include <utility>

namespace PS_AGONY2D
{
	namespace BPI = BroadPhaseInternal;

	namespace
	{
		constexpr size_t THREADING_NODE_THRESHOLD = 400;
		constexpr size_t CROSS_TASKS_PER_WORKER = 4;
		constexpr size_t TASKS_PER_WORKER = 16; // Per task category; more tasks = better balance.

		template <size_t N, size_t... I>
		std::array<BPI::BvhTreeView, N> makeViewsImpl(const std::array<BPI::BvhTree, N>& t, std::index_sequence<I...>)
		{
			return { t[I].view()... };
		}

		template <size_t N>
		std::array<BPI::BvhTreeView, N> makeViews(const std::array<BPI::BvhTree, N>& t)
		{
			return makeViewsImpl(t, std::make_index_sequence<N>{});
		}

		// Runs leaf tests for one thread and batches the resulting pairs.
		struct LeafTester
		{
			const BPI::BvhTreeView& dyn;
			const ObjectDenseIndex* owner;
			std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;
			BPI::PairPushBuffer pushBuffer;

			LeafTester(
				const BPI::BvhTreeView& dynIn,
				const ObjectDenseIndex* ownerIn,
				CacheLineAlignedVector<ObjectPair>& dest) noexcept :
				dyn(dynIn), owner(ownerIn), pushBuffer(dest)
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

			void crossPair(const BPI::BvhTreeView& a, const BPI::BvhTreeView& b, const BvhNodePair& p)
			{
				BPI::testLeafPairCross<false>(a.nodes[p.a], a, b.nodes[p.b], b, nullptr, masks.data(), pushBuffer);
			}

			void flush()
			{
				if (pushBuffer.size > 0) pushBuffer.flush();
			}
		};
	}

	void BroadPhaseCollisionDetector::setDataViewers(const AABBSoAViewer& aabbs, const ObjectDenseIndex* colliderBodyIndexIn)
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
	}

	void BroadPhaseCollisionDetector::updateTrees(const TreesUpdateParams& params)
	{
		TRACY_SCOPE_N("Update broad-phase trees");

		for (size_t i = 0; i < params.ids.size(); i++)
		{
			updateTree(trees[i], params.types[i], params.ids[i]);
		}
	}

	void BroadPhaseCollisionDetector::refitTrees(BroadPhaseInternal::BvhTreeTypeFlag flags)
	{
		uint32_t mask = static_cast<uint32_t>(flags);
		while (mask)
		{
			const uint32_t treeIndex = std::countr_zero(mask);
			mask &= mask - 1;
			trees[treeIndex].refit(collidersAABB);
		}
	}

	std::span<const ObjectPair> BroadPhaseCollisionDetector::findCollisions(ExecutionPolicy executionPolicy)
	{
		TRACY_SCOPE_N("Find broad-phase collisions");

		collisionData.clear();

		constexpr size_t dynIdx = size_t(BPI::BvhTreeType::Dynamic);
		if (trees[dynIdx].empty()) return collisionData;

		collisionData.reserve(trees[dynIdx].colliderCount());

		size_t nodeTotal = 0;
		for (const auto& t : trees) nodeTotal += t.nodeCount();

		const bool useThreading =
			executionPolicy == ExecutionPolicy::ForceMultiThreaded ||
			(executionPolicy != ExecutionPolicy::ForceSingleThreaded &&
				nodeTotal > THREADING_NODE_THRESHOLD);

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

		for (const auto& t : trees) total += t.getMemoryUsage();

		total += getVectorMemoryUsage(queryResources.selfNodePairs);
		total += getVectorMemoryUsage(queryResources.selfLeafPairs);
		total += getVectorMemoryUsage(queryResources.selfLeaves);
		for (const auto& v : queryResources.crossNodePairs) total += getVectorMemoryUsage(v);
		for (const auto& v : queryResources.crossLeafPairs) total += getVectorMemoryUsage(v);

		total += getVectorMemoryUsage(queryPairsThreadedResources.workerData);
		total += getVectorMemoryUsage(queryPairsThreadedResources.tasks);
		total += getVectorMemoryUsage(queryPairsThreadedResources.taskOutputs);
		for (const auto& v : queryPairsThreadedResources.crossScratch) total += getVectorMemoryUsage(v);
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

	void BroadPhaseCollisionDetector::updateTree(BroadPhaseInternal::BvhTree& tree, TreeUpdate updateType, std::span<const ObjectDenseIndex> ids)
	{
		if (updateType == TreeUpdate::None) return;

		const bool rebuild =
			updateType == TreeUpdate::Rebuild ||
			tree.colliderCount() != ids.size();

		if (rebuild)
		{
			tree.build(collidersAABB, ids);
		}
		else
		{
			tree.refit(collidersAABB);
		}
	}

	void BroadPhaseCollisionDetector::queryPairs()
	{
		TRACY_SCOPE_N("Query pairs");

		const auto views = makeViews(trees);
		const BPI::BvhTreeView& dyn = views[size_t(BPI::BvhTreeType::Dynamic)];

		auto& r = queryResources;
		r.selfNodePairs.clear();
		r.selfLeafPairs.clear();
		r.selfLeaves.clear();
		for (auto& v : r.crossNodePairs) v.clear();
		for (auto& v : r.crossLeafPairs) v.clear();

		{
			TRACY_SCOPE_N("Traverse");

			BPI::traverseRootForInitialPairs(dyn.nodes, r.selfNodePairs, r.selfLeafPairs, r.selfLeaves);
			BPI::traverseNodePairStack(dyn.nodes, dyn.nodes, r.selfNodePairs, r.selfLeafPairs);

			for (size_t c = 0; c < CROSS_PAIR_COUNT; c++)
			{
				const size_t ia = size_t(CROSS_TREE_PAIRS[c].first);
				const size_t ib = size_t(CROSS_TREE_PAIRS[c].second);
				if (trees[ia].empty() || trees[ib].empty() || !views[ia].nodes[0].overlaps(views[ib].nodes[0])) continue;

				r.crossNodePairs[c].push_back({ 0, 0 });
				BPI::traverseNodePairStack(views[ia].nodes, views[ib].nodes, r.crossNodePairs[c], r.crossLeafPairs[c]);
			}
		}

		{
			TRACY_SCOPE_N("Test collisions in leaves");

			LeafTester tester(dyn, colliderBodyIndex, collisionData);

			for (const uint32_t leaf : r.selfLeaves) tester.self(leaf);
			for (const BvhNodePair& p : r.selfLeafPairs) tester.selfPair(p);
			for (size_t c = 0; c < CROSS_PAIR_COUNT; c++)
			{
				const auto& va = views[size_t(CROSS_TREE_PAIRS[c].first)];
				const auto& vb = views[size_t(CROSS_TREE_PAIRS[c].second)];
				for (const BvhNodePair& p : r.crossLeafPairs[c]) tester.crossPair(va, vb, p);
			}

			tester.flush();
		}
	}

	void BroadPhaseCollisionDetector::queryPairsThreaded()
	{
		TRACY_SCOPE_N("Query pairs (Threaded)");

		auto& threadingBackend = Threading::getThreadingBackend();
		const size_t poolWorkers = threadingBackend.getThreadCount();
		if (poolWorkers == 0)
		{
			queryPairs();
			return;
		}
		const size_t participants = poolWorkers + 1; // Pool workers + main thread.

		using TK = QueryPairsThreadedResources::TaskKind;
		auto& tr = queryPairsThreadedResources;

		const auto views = makeViews(trees);
		const BPI::BvhTreeView& dyn = views[size_t(BPI::BvhTreeType::Dynamic)];

		auto& r = queryResources;
		r.selfNodePairs.clear();
		r.selfLeafPairs.clear();
		r.selfLeaves.clear();
		for (auto& v : r.crossNodePairs) v.clear();
		for (auto& v : r.crossLeafPairs) v.clear();

		// Initial frontiers (same as single-threaded, so ordering is shared).
		{
			TRACY_SCOPE_N("Initial traverse");

			BPI::traverseRootForInitialPairs(dyn.nodes, r.selfNodePairs, r.selfLeafPairs, r.selfLeaves);

			for (size_t c = 0; c < CROSS_PAIR_COUNT; c++)
			{
				const size_t ia = size_t(CROSS_TREE_PAIRS[c].first);
				const size_t ib = size_t(CROSS_TREE_PAIRS[c].second);
				if (trees[ia].empty() || trees[ib].empty()) continue;

				BPI::expandCrossRootPairsOrdered(
					views[ia].nodes, views[ib].nodes, participants * CROSS_TASKS_PER_WORKER,
					r.crossNodePairs[c], tr.crossScratch[c]);
			}
		}

		// Ordered task list: selfLeaves, selfLeafPairs, selfNodePairs (reverse), cross.
		tr.tasks.clear();
		auto addRanges = [&](TK kind, uint32_t pair, size_t n, bool reverse)
			{
				if (n == 0) return;
				const size_t divisor = participants * TASKS_PER_WORKER;
				const size_t chunk = std::max<size_t>(1, (n + divisor - 1) / divisor);
				const size_t first = tr.tasks.size();
				for (size_t b = 0; b < n; b += chunk)
				{
					tr.tasks.push_back({ kind, pair, static_cast<uint32_t>(b), static_cast<uint32_t>(std::min(b + chunk, n)) });
				}
				if (reverse) std::reverse(tr.tasks.begin() + first, tr.tasks.end());
			};

		addRanges(TK::SelfLeaves, 0, r.selfLeaves.size(), false);
		addRanges(TK::SelfLeafPairs, 0, r.selfLeafPairs.size(), false);
		addRanges(TK::SelfNodes, 0, r.selfNodePairs.size(), true);
		for (uint32_t c = 0; c < CROSS_PAIR_COUNT; c++)
			addRanges(TK::CrossNodes, c, r.crossNodePairs[c].size(), false);

		const uint32_t taskCount = static_cast<uint32_t>(tr.tasks.size());
		if (taskCount == 0) return;

		tr.taskOutputs.resize(taskCount);

		// Main thread always works too (slot 0), so the pool needs at most taskCount - 1 workers.
		const size_t launchCount = std::min<size_t>(poolWorkers, taskCount - 1);
		if (tr.workerData.size() < launchCount + 1) tr.workerData.resize(launchCount + 1); // Never shrink: keeps buffers.

		alignas(64) std::atomic<uint32_t> globalWorkerIndex{ 1 }; // Slot 0 is reserved for main.
		alignas(64) std::atomic<uint32_t> nextTask{ 0 };

		auto runWorker = [&](uint32_t workerIndex)
			{
				TRACY_SCOPE_N("Query pairs worker thread");

				auto& wData = tr.workerData[workerIndex];

				LeafTester tester(dyn, colliderBodyIndex, wData.collisionData);

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
					{
						const auto& va = views[size_t(CROSS_TREE_PAIRS[task.pair].first)];
						const auto& vb = views[size_t(CROSS_TREE_PAIRS[task.pair].second)];
						const auto& nodePairs = r.crossNodePairs[task.pair];
						for (uint32_t i = task.begin; i < task.end; i++)
						{
							wData.nodePairsToTraverse.clear();
							wData.crossLeafPairs.clear();
							wData.nodePairsToTraverse.push_back(nodePairs[i]);
							BPI::traverseNodePairStack(va.nodes, vb.nodes, wData.nodePairsToTraverse, wData.crossLeafPairs);
							for (const BvhNodePair& p : wData.crossLeafPairs) tester.crossPair(va, vb, p);
						}
						break;
					}
					}

					tester.flush(); // Task output must be contiguous and complete in collisionData.
					tr.taskOutputs[t] = { workerIndex, outBegin, static_cast<uint32_t>(wData.collisionData.size()) };
				}
			};

		auto poolFunc = [&]()
			{
				const uint32_t workerIndex = globalWorkerIndex.fetch_add(1, std::memory_order_relaxed);
				runWorker(workerIndex);

				auto& wData = tr.workerData[workerIndex];
				wData.isDone.store(true, std::memory_order_release);
				wData.isDone.notify_one();
			};

		{
			TRACY_SCOPE_N("Launch workers");

			for (size_t i = 0; i <= launchCount; i++) // Includes main slot 0.
			{
				auto& wData = tr.workerData[i];
				wData.nodePairsToTraverse.clear();
				wData.selfLeafPairs.clear();
				wData.crossLeafPairs.clear();
				wData.collisionData.clear();
				wData.isDone = false;
			}

			if (launchCount > 0)
			{
				std::vector<uint32_t> threadIndices(launchCount);
				for (uint32_t i = 0; i < launchCount; i++)
				{
					threadIndices[i] = i;
				}

				threadingBackend.launch(
					launchCount,
					[](void* ctx, size_t) { (*static_cast<decltype(poolFunc)*>(ctx))(); },
					&poolFunc,
					threadIndices
				);
			}
		}

		runWorker(0); // Main thread participates.

		// Wait for pool workers only (slots 1..launchCount), then concatenate in canonical task order.
		{
			TRACY_SCOPE_N("Wait for workers to finish and combine data");

			for (size_t i = 1; i <= launchCount; i++)
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