#pragma once
#include "ObjectSoA.h"

#include "BroadPhaseInternal/BvhNode.h"
#include "BroadPhaseInternal/QueryShapes.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <vector>
#include <atomic>
#include <array>
#include <bit>

namespace PS_AGONY
{
	using BvhNode = BroadPhaseInternal::BvhNode;

	// NOTE: This detector operates over COLLIDER AABBs, not body AABBs.
	// Every ObjectIndex produced or consumed here (in ObjectPair results, or in
	// fetchBodiesInCircle's output) is therefore a ColliderIndex. Callers that
	// need the owning body must resolve it via ColliderSoAViewer::bodyIndex.
	class BroadPhaseCollisionDetector
	{
	public:
		enum class ExecutionPolicy
		{
			Standard,
			ForceSingleThreaded,
			ForceMultiThreaded
		};
	private:
		struct BvhNodePair
		{
			uint32_t a, b;

			BvhNodePair swap() const noexcept { return { b, a }; }
		};

		struct alignas(64) PairVector
		{
			std::vector<ObjectPair> pairs;
		};

		struct PackedBodyIndex
		{
			uint32_t key;
			ObjectIndex index; // Collider index.
		};

		struct BvhFunctionResources
		{
			// Must keep their state between frames:

			std::vector<BvhNode> nodes;
			std::vector<ObjectIndex> mainColliderIndices; // Collider indices, sorted by Morton code.

			// The rest (Build):

			SimdAlignedVector<Real> transformedCentroidX;
			SimdAlignedVector<Real> transformedCentroidY;
			SimdAlignedVector<uint32_t> mortonCodes;
			std::vector<PackedBodyIndex> tempPackedBodyIndicesToSort;
			std::vector<PackedBodyIndex> tempPackedBodyIndicesToSort2;

			// The rest (Query):

			std::vector<BvhNodePair> nodePairsToTraverse;
			std::vector<BvhNodePair> leafPairsToTestCollisions;
			std::vector<uint32_t> leavesToTestCollisions;
		};

		struct QueryPairsThreadedResources
		{
			struct alignas(64) WorkerData
			{
				std::vector<BvhNodePair> nodePairsToTraverse;
				std::vector<BvhNodePair> leafPairsToTestCollisions;
				std::vector<ObjectPair> collisionData;

				std::atomic<bool> isDone{ false };

				WorkerData() = default;
				~WorkerData() = default;
				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					nodePairsToTraverse = std::move(other.nodePairsToTraverse);
					leafPairsToTestCollisions = std::move(other.leafPairsToTestCollisions);
					collisionData = std::move(other.collisionData);
				}
				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						nodePairsToTraverse = std::move(other.nodePairsToTraverse);
						leafPairsToTestCollisions = std::move(other.leafPairsToTestCollisions);
						collisionData = std::move(other.collisionData);
					}
					return *this;
				}
			};

			std::vector<WorkerData> workerData;
		};

		struct LeafBodyAABBSoA
		{
			struct LeafData
			{
				std::array<Real, BvhNode::KD_LEAF_SIZE> data;
			};

			RealSimdAlignedVector<LeafData> minX;
			RealSimdAlignedVector<LeafData> maxX;
			RealSimdAlignedVector<LeafData> minY;
			RealSimdAlignedVector<LeafData> maxY;
		};

		AABBSoAViewer collidersAABB;
		const ObjectIndex* colliderBodyIndex = nullptr; // Collider -> owning body; used to reject same-body pairs.

		BvhFunctionResources bvhFunctionResources;
		QueryPairsThreadedResources queryPairsThreadedResources;

		LeafBodyAABBSoA leafBodyAABBs;

		std::vector<ObjectPair> collisionData; // Pairs of COLLIDER indices.
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

		// Clears all broad-phase state (BVH tree, cached indices, pairs). Call when there are
		// zero colliders so stale tree data isn't served to fetchAABBs/fetchCollidersInCircle/
		// fetchCollidersInAABB.
		void clearData();

		// Builds (rebuild=true) or refits (rebuild=false) the BVH tree from the current collider
		// AABBs. Safe to call with 0 or 1 colliders: 0 clears the tree, 1 produces a single-leaf
		// tree. Needed so downstream queries reflect the current scene even when there are too
		// few colliders to collide.
		void buildTree(bool rebuild);

		// Queries the already-built tree for overlapping collider pairs. Caller must ensure at
		// least 2 colliders exist (checked defensively); returns empty otherwise.
		// Returned pairs are COLLIDER index pairs.
		const std::vector<ObjectPair>& findCollisions(ExecutionPolicy executionPolicy = ExecutionPolicy::Standard);

		// Returns all node aabbs.
		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		// Returns COLLIDER indices whose AABB overlaps the query shape.
		template <typename ShapeQuery>
		void queryCollidersInShape(const ShapeQuery& shape, std::vector<ColliderIndex>& outColliders) const;

		size_t getMemoryUsage() const;
	private:
		static constexpr uint64_t bvhDepth(uint64_t n, uint64_t leafSize)
		{
			uint64_t d = 0ull;
			while (n > leafSize) { n = (n + 1ull) >> 1ull; d++; }
			return d;
		}

		void computeCentroidsWithTransformations(uint32_t colliderCount, Vec2 globalMin, Vec2 scale, Real clampMax);

		template<std::floating_point TReal>
		void computeMortonCodes(uint32_t colliderCount);

		void sortBodyIndicesByMortonCodes(uint32_t colliderCount);

		void buildBvhTree(const uint32_t colliderCount);

		void fitBvhNodeAABBs(bool isRebuild);

		void queryBvhPairs();
		void queryBvhPairsThreaded();

		void traverseNodesToGetOverlappingLeafPairs();
		void testCollisionsInLeaves();
	};

	template <typename ShapeQuery>
	inline void BroadPhaseCollisionDetector::queryCollidersInShape(const ShapeQuery& shape, std::vector<ColliderIndex>& outColliders) const
	{
		using RealSimd = Ecstasy::Core::Simd<Real>;
		constexpr uint32_t LANES = RealSimd::lanes;

		if (bvhFunctionResources.nodes.empty()) return;

		// Quick escape if root node does not overlap shape.
		if (!shape.overlaps(bvhFunctionResources.nodes[0])) return;

		// Fetch base pointers.
		const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<const Real*>(leafBodyAABBs.minX.data());
		const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<const Real*>(leafBodyAABBs.maxX.data());
		const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<const Real*>(leafBodyAABBs.minY.data());
		const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<const Real*>(leafBodyAABBs.maxY.data());
		const BvhNode* ECSTASY_RESTRICT nodePtr = bvhFunctionResources.nodes.data();

		// Local stack traversal setup.
		constexpr uint64_t MAX_STACK_CAPACITY = 2ull * (32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;

		uint32_t stack[MAX_STACK_CAPACITY];
		uint32_t stackSize = 0;

		stack[stackSize++] = 0;

		while (stackSize > 0)
		{
			const uint32_t nodeIdx = stack[--stackSize];
			const BvhNode& node = nodePtr[nodeIdx];

			// Fast path: fully contained node allows bulk-inserting all leaf elements.
			if (shape.contains(node))
			{
				outColliders.insert(
					outColliders.end(),
					&bvhFunctionResources.mainColliderIndices[node.start],
					&bvhFunctionResources.mainColliderIndices[node.end]
				);
				continue;
			}

			if (node.leftChildIndex == BvhNode::INVALID_INDEX) // Leaf Node.
			{
				const uint32_t count = node.end - node.start;

				const size_t srcIndex = node.leafIndex * BvhNode::KD_LEAF_SIZE;
				const Real* leafMinX = leafMinXPtr + srcIndex;
				const Real* leafMaxX = leafMaxXPtr + srcIndex;
				const Real* leafMinY = leafMinYPtr + srcIndex;
				const Real* leafMaxY = leafMaxYPtr + srcIndex;

				uint32_t mask = 0;
				for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LANES)
				{
					const RealSimd minXV = RealSimd::load(leafMinX + j);
					const RealSimd maxXV = RealSimd::load(leafMaxX + j);
					const RealSimd minYV = RealSimd::load(leafMinY + j);
					const RealSimd maxYV = RealSimd::load(leafMaxY + j);

					const auto overlap = shape.simdOverlap(minXV, maxXV, minYV, maxYV);

					mask |= overlap.movemask() << j;
				}

				// Mask out invalid padded elements past count boundary.
				mask &= static_cast<uint32_t>((1ULL << count) - 1ULL);

				while (mask)
				{
					const uint32_t lane = std::countr_zero(mask);
					mask &= mask - 1;
					outColliders.push_back(bvhFunctionResources.mainColliderIndices[node.start + lane]);
				}
			}
			else // Internal Node.
			{
				const BvhNode& left = nodePtr[node.leftChildIndex];
				const BvhNode& right = nodePtr[node.leftChildIndex + 1];

				if (shape.overlaps(right))
				{
					stack[stackSize++] = node.leftChildIndex + 1;
				}
				if (shape.overlaps(left))
				{
					stack[stackSize++] = node.leftChildIndex;
				}
			}
		}
	}
}