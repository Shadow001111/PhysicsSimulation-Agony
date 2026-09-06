#pragma once
#include "ObjectSoA.h"

#include <vector>
#include <atomic>
#include <array>

namespace PS_AGONY
{
	// NOTE: This detector operates over COLLIDER AABBs, not body AABBs.
	// Every ObjectIndex produced or consumed here (in ObjectPair results, or in
	// fetchBodiesInCircle's output) is therefore a ColliderIndex. Callers that
	// need the owning body must resolve it via ColliderSoAViewer::bodyIndex.
	class BroadPhaseCollisionDetector
	{
	public:
		// Note: Splitting on cold and hot didn't help.
		struct BvhNode
		{
			// Max KD_LEAF_SIZE is 32. Larger size will fuck up bitwise mask.
			// (We can change mask to me uint64_t to allow max KD_LEAF_SIZE to be 64, but increasing KD_LEAF_SIZE leads to perfomance decrease in narrow phase.)
			static constexpr uint32_t KD_LEAF_SIZE = 16;
			static_assert((KD_LEAF_SIZE% Ecstasy::Core::Simd<Real>::lanes) == 0, "KD_LEAF_SIZE must be multiple of Simd<Real>::lanes.");

			static constexpr uint32_t INVALID_INDEX = -1;

			// AABB data must stay first.
			Real minX, maxX, minY, maxY; // Merged AABB of all colliders in this subtree.
			uint32_t leftChildIndex = INVALID_INDEX; // INVALID_INDEX for leaves.
			// rightChildIndex = leftChildIndex + 1.
			uint32_t start, end; // Range in kdIndices: [start, end).
			uint32_t leafIndex; // If node is a leaf, it's its index.

			BvhNode() :
				start(0), end(0)
			{
			}

			BvhNode(uint32_t start, uint32_t end) :
				start(start), end(end)
			{
			}
		};

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

		// Returned pairs are COLLIDER index pairs.
		const std::vector<ObjectPair>& findCollisions(bool rebuild, ExecutionPolicy executionPolicy = ExecutionPolicy::Standard);

		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		// Returns COLLIDER indices whose AABB overlaps the query circle.
		void fetchBodiesInCircle(Vec2 pos, Real radius, std::vector<ObjectIndex>& outColliders) const;

		size_t getMemoryUsage() const;
	private:
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
}