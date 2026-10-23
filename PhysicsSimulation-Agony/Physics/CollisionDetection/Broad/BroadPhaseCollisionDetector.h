#pragma once
#include "BvhNode.h"
#include "BvhNodePair.h"
#include "BvhTree.h"
#include "QueryShapes.h"
#include "Math.h"

#include "Physics/ObjectSoA.h"
#include "Physics/Types/ExecutionPolicy.h"

#include <atomic>
#include <cstdint>
#include <span>
#include <vector>
#include <array>

namespace PS_AGONY2D
{
	using BvhNode = BroadPhaseInternal::BvhNode;
	using BvhNodePair = BroadPhaseInternal::BvhNodePair;

	// How the static tree is updated by buildTrees.
	enum class StaticTreeUpdate : uint8_t
	{
		None,    // Static colliders did not change.
		Refit,   // Same membership, some AABBs moved (e.g. kinematic bodies).
		Rebuild  // Membership changed or a full rebuild is wanted.
	};

	// NOTE: This detector operates over COLLIDER AABBs, not body AABBs.
	// Every ObjectIndex produced or consumed here is a ColliderIndex. Callers that
	// need the owning body must resolve it via ColliderSoAViewer::bodyIndex.
	//
	// Dynamic and static colliders live in separate trees. Static-static pairs are
	// never tested.
	class BroadPhaseCollisionDetector
	{
		struct QueryResources
		{
			CacheLineAlignedVector<BvhNodePair> selfNodePairs;
			CacheLineAlignedVector<BvhNodePair> selfLeafPairs;
			CacheLineAlignedVector<uint32_t> selfLeaves;
			CacheLineAlignedVector<BvhNodePair> crossNodePairs;
			CacheLineAlignedVector<BvhNodePair> crossLeafPairs;
		};

		struct QueryPairsThreadedResources
		{
			struct alignas(64) WorkerData
			{
				CacheLineAlignedVector<BvhNodePair> nodePairsToTraverse;
				CacheLineAlignedVector<BvhNodePair> selfLeafPairs;
				CacheLineAlignedVector<BvhNodePair> crossLeafPairs;
				CacheLineAlignedVector<ObjectPair> collisionData;

				std::atomic<bool> isDone{ false };

				WorkerData() = default;
				~WorkerData() = default;
				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					nodePairsToTraverse = std::move(other.nodePairsToTraverse);
					selfLeafPairs = std::move(other.selfLeafPairs);
					crossLeafPairs = std::move(other.crossLeafPairs);
					collisionData = std::move(other.collisionData);
				}
				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						nodePairsToTraverse = std::move(other.nodePairsToTraverse);
						selfLeafPairs = std::move(other.selfLeafPairs);
						crossLeafPairs = std::move(other.crossLeafPairs);
						collisionData = std::move(other.collisionData);
					}
					return *this;
				}
			};

			// Ordered work list. Concatenating task outputs in this order reproduces
			// the single-threaded pair order exactly, for any worker count.
			enum class TaskKind : uint8_t
			{
				SelfLeaves,    // Range of queryResources.selfLeaves, ascending.
				SelfLeafPairs, // Range of queryResources.selfLeafPairs, ascending.
				SelfNodes,     // Range of queryResources.selfNodePairs, processed DESCENDING (stack order).
				CrossNodes     // Range of queryResources.crossNodePairs, ascending (already DFS-ordered).
			};

			struct Task
			{
				TaskKind kind;
				uint32_t begin, end;
			};

			// Where a task's pairs live: workerData[worker].collisionData[begin, end).
			struct TaskOutput
			{
				uint32_t worker, begin, end;
			};

			std::vector<WorkerData> workerData;
			std::vector<Task> tasks;
			std::vector<TaskOutput> taskOutputs;
			CacheLineAlignedVector<BvhNodePair> crossScratch;
		};

		AABBSoAViewer collidersAABB;
		const ObjectIndex* colliderBodyIndex = nullptr; // Collider -> owning body; rejects same-body pairs.

		std::array<BroadPhaseInternal::BvhTree, 2> trees;

		bool staticRebuildRequested = true;

		QueryResources queryResources;
		QueryPairsThreadedResources queryPairsThreadedResources;

		CacheLineAlignedVector<ObjectPair> collisionData; // Pairs of COLLIDER indices.
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = delete;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = delete;

		// 'aabbs' must be COLLIDER AABBs. 'colliderBodyIndexIn' maps collider index -> owning
		// body index and is used to reject pairs of colliders that belong to the same body.
		void setDataViewers(
			const AABBSoAViewer& aabbs,
			const ObjectIndex* colliderBodyIndexIn
		);

		// Clears both trees and cached pairs. Call when there are zero colliders so stale
		// tree data isn't served to queries.
		void clearData();

		// Rebuilds the dynamic tree every call. The static tree follows 'staticUpdate', and is
		// also rebuilt if requestStaticRebuild() was called or its collider count differs from
		// 'staticIds'. Spans hold COLLIDER indices; either may be empty.
		void buildTrees(
			std::span<const ColliderIndex> dynamicIds,
			std::span<const ColliderIndex> staticIds,
			StaticTreeUpdate staticUpdate
		);

		// Forces a static tree rebuild on the next buildTrees.
		void requestStaticRebuild() noexcept { staticRebuildRequested = true; }

		// Finds overlapping collider pairs: dynamic-dynamic and dynamic-static. Dynamic-static
		// pairs are ordered (dynamic, static). Returns empty if there are no dynamic colliders.
		std::span<const ObjectPair> findCollisions(ExecutionPolicy executionPolicy = ExecutionPolicy::Auto);

		void getLeafNodeAABBs(BroadPhaseInternal::BvhTreeTypeFlag flags, std::vector<AABB>& outAABBs) const;

		// Returns COLLIDER indices (dynamic first, then static) whose AABB overlaps the query shape.
		template <typename ShapeQuery>
		void queryCollidersInShape(const ShapeQuery& shape, BroadPhaseInternal::BvhTreeTypeFlag flags, std::vector<ColliderIndex>& outColliders) const;

		// Returns the AABBs of leaf nodes (of both trees) overlapping the query shape.
		template <typename ShapeQuery>
		void queryLeafNodeAABBsInShape(const ShapeQuery& shape, BroadPhaseInternal::BvhTreeTypeFlag flags, std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;
	private:
		void queryPairs();
		void queryPairsThreaded();
	};

	template <typename ShapeQuery>
	inline void BroadPhaseCollisionDetector::queryCollidersInShape(
		const ShapeQuery& shape,
		BroadPhaseInternal::BvhTreeTypeFlag flags,
		std::vector<ColliderIndex>& outColliders
	) const
	{
		uint32_t mask = static_cast<uint32_t>(flags);
		while (mask)
		{
			const uint32_t treeIndex = std::countr_zero(mask);
			mask &= mask - 1;
			trees[treeIndex].queryCollidersInShape(shape, outColliders);
		}
	}

	template <typename ShapeQuery>
	inline void BroadPhaseCollisionDetector::queryLeafNodeAABBsInShape(
		const ShapeQuery& shape,
		BroadPhaseInternal::BvhTreeTypeFlag flags,
		std::vector<AABB>& outAABBs
	) const
	{
		uint32_t mask = static_cast<uint32_t>(flags);
		while (mask)
		{
			const uint32_t treeIndex = std::countr_zero(mask);
			mask &= mask - 1;
			trees[treeIndex].queryLeafNodeAABBsInShape(shape, outAABBs);
		}
	}
}