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
				BPI::testLeafSelfCross(dyn.nodes[leaf], dyn, owner, masks, pushBuffer);
			}

			void selfPair(const BvhNodePair& p)
			{
				BPI::testLeafPairCross<true>(dyn.nodes[p.a], dyn, dyn.nodes[p.b], dyn, owner, masks, pushBuffer);
			}

			void crossPair(const BvhNodePair& p)
			{
				BPI::testLeafPairCross<false>(dyn.nodes[p.a], dyn, stat.nodes[p.b], stat, nullptr, masks, pushBuffer);
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
		dynamicTree.clear();
		staticTree.clear();
		staticRebuildRequested = true;
	}

	void BroadPhaseCollisionDetector::buildTrees(
		std::span<const ColliderIndex> dynamicIds,
		std::span<const ColliderIndex> staticIds,
		StaticTreeUpdate staticUpdate)
	{
		TRACY_SCOPE_N("Build broad-phase trees");

		dynamicTree.build(collidersAABB, dynamicIds);

		const bool rebuildStatic =
			staticUpdate == StaticTreeUpdate::Rebuild ||
			staticRebuildRequested ||
			staticTree.colliderCount() != staticIds.size();

		if (rebuildStatic)
		{
			staticTree.build(collidersAABB, staticIds);
			staticRebuildRequested = false;
		}
		else if (staticUpdate == StaticTreeUpdate::Refit)
		{
			staticTree.refit(collidersAABB);
		}
	}

	const std::vector<ObjectPair>& BroadPhaseCollisionDetector::findCollisions(ExecutionPolicy executionPolicy)
	{
		TRACY_SCOPE_N("Find broad-phase collisions");

		collisionData.clear();

		if (dynamicTree.empty()) return collisionData;

		collisionData.reserve(dynamicTree.colliderCount());

		const bool useThreading =
			executionPolicy == ExecutionPolicy::ForceMultiThreaded ||
			(executionPolicy != ExecutionPolicy::ForceSingleThreaded &&
				dynamicTree.nodeCount() + staticTree.nodeCount() > THREADING_NODE_THRESHOLD);

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

	void BroadPhaseCollisionDetector::getDynamicLeafNodeAABBs(std::vector<AABB>& outAABBs) const
	{
		dynamicTree.getLeafNodeAABBs(outAABBs);
	}

	void BroadPhaseCollisionDetector::getStaticLeafNodeAABBs(std::vector<AABB>& outAABBs) const
	{
		staticTree.getLeafNodeAABBs(outAABBs);
	}

	void BroadPhaseCollisionDetector::getAllLeafNodeAABBs(std::vector<AABB>& outAABBs) const
	{
		dynamicTree.getLeafNodeAABBs(outAABBs);
		staticTree.getLeafNodeAABBs(outAABBs);
	}

	size_t BroadPhaseCollisionDetector::getMemoryUsage() const
	{
		size_t total = sizeof(BroadPhaseCollisionDetector);

		total += dynamicTree.getMemoryUsage();
		total += staticTree.getMemoryUsage();

		total += getVectorMemoryUsage(queryResources.selfNodePairs);
		total += getVectorMemoryUsage(queryResources.selfLeafPairs);
		total += getVectorMemoryUsage(queryResources.selfLeaves);
		total += getVectorMemoryUsage(queryResources.crossNodePairs);
		total += getVectorMemoryUsage(queryResources.crossLeafPairs);

		total += getVectorMemoryUsage(queryPairsThreadedResources.workerData);
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

		const BPI::BvhTreeView dyn = dynamicTree.view();
		const BPI::BvhTreeView stat = staticTree.view();
		const bool hasStatic = !staticTree.empty();

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

			if (hasStatic && BPI::overlaps(dyn.nodes[0], stat.nodes[0]))
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

		auto& threadPool = Threading::getGlobalThreadPool();
		const size_t workerCount = threadPool.getThreadCount() - 1;

		const BPI::BvhTreeView dyn = dynamicTree.view();
		const BPI::BvhTreeView stat = staticTree.view();
		const bool hasStatic = !staticTree.empty();

		auto& r = queryResources;
		r.selfNodePairs.clear();
		r.selfLeafPairs.clear();
		r.selfLeaves.clear();
		r.crossNodePairs.clear();
		r.crossLeafPairs.clear();

		// Initial frontiers: independent sub-traversals handed out to workers.
		{
			TRACY_SCOPE_N("Initial traverse");

			BPI::traverseRootForInitialPairs(dyn.nodes, r.selfNodePairs, r.selfLeafPairs, r.selfLeaves);

			if (hasStatic)
			{
				BPI::expandCrossRootPairs(
					dyn.nodes, stat.nodes, workerCount * CROSS_TASKS_PER_WORKER,
					r.crossNodePairs, r.crossLeafPairs);
			}
		}

		queryPairsThreadedResources.workerData.resize(workerCount);

		alignas(64) std::atomic<uint32_t> globalWorkerIndex{ 0 };
		{
			TRACY_SCOPE_N("Launch workers");

			auto workerFunc = [&]()
				{
					TRACY_SCOPE_N("Query pairs worker thread");

					const auto workerIndex = globalWorkerIndex.fetch_add(1, std::memory_order_relaxed);
					auto& wData = queryPairsThreadedResources.workerData[workerIndex];

					LeafTester tester(dyn, stat, colliderBodyIndex, wData.collisionData);

					// Drain this worker's share of the frontiers down to leaf pairs.
					{
						TRACY_SCOPE_N("Traverse");

						for (size_t t = workerIndex; t < r.selfNodePairs.size(); t += workerCount)
						{
							wData.nodePairsToTraverse.clear();
							wData.nodePairsToTraverse.push_back(r.selfNodePairs[t]);
							BPI::traverseNodePairStack(dyn.nodes, dyn.nodes, wData.nodePairsToTraverse, wData.selfLeafPairs);
						}

						for (size_t t = workerIndex; t < r.crossNodePairs.size(); t += workerCount)
						{
							wData.nodePairsToTraverse.clear();
							wData.nodePairsToTraverse.push_back(r.crossNodePairs[t]);
							BPI::traverseNodePairStack(dyn.nodes, stat.nodes, wData.nodePairsToTraverse, wData.crossLeafPairs);
						}
					}

					// Leaf tests: initial frontier share, then own traversal results.
					{
						TRACY_SCOPE_N("Test collisions");

						for (size_t t = workerIndex; t < r.selfLeaves.size(); t += workerCount)
							tester.self(r.selfLeaves[t]);

						for (size_t t = workerIndex; t < r.selfLeafPairs.size(); t += workerCount)
							tester.selfPair(r.selfLeafPairs[t]);

						for (size_t t = workerIndex; t < r.crossLeafPairs.size(); t += workerCount)
							tester.crossPair(r.crossLeafPairs[t]);

						for (const BvhNodePair& p : wData.selfLeafPairs) tester.selfPair(p);
						for (const BvhNodePair& p : wData.crossLeafPairs) tester.crossPair(p);

						wData.selfLeafPairs.clear();
						wData.crossLeafPairs.clear();
					}

					tester.flush();

					wData.isDone.store(true, std::memory_order_release);
					wData.isDone.notify_one();
				};

			for (auto& wData : queryPairsThreadedResources.workerData)
			{
				wData.nodePairsToTraverse.clear();
				wData.selfLeafPairs.clear();
				wData.crossLeafPairs.clear();
				wData.collisionData.clear();
				wData.isDone = false;
			}

			std::vector<Ecstasy::Core::Threading::Task> tasks;
			tasks.reserve(workerCount);

			for (size_t i = 0; i < workerCount; i++)
			{
				tasks.emplace_back(workerFunc);
			}
			threadPool.enqueueBulk(tasks);
		}

		// Wait for workers and combine their pairs.
		{
			TRACY_SCOPE_N("Wait for workers to finish and combine data");
			for (auto& wData : queryPairsThreadedResources.workerData)
			{
				wData.isDone.wait(false, std::memory_order_acquire);
				{
					TRACY_SCOPE_N("Combine");
					collisionData.insert(
						collisionData.end(),
						wData.collisionData.begin(),
						wData.collisionData.end()
					);
				}
			}
		}
	}
}
