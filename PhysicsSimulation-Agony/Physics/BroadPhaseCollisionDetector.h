#pragma once
#include "BodySoAViewer.h"

#include <vector>
#include <atomic>

namespace PS_AGONY
{
	class BroadPhaseCollisionDetector
	{
		// Note: Splitting on cold and hot didn't help.
	public:
		struct BvhNode
		{
			// Max KD_LEAF_SIZE is 32. Larger size will fuck up bitwise mask.
			// (We can change mask to me uint64_t to allow max KD_LEAF_SIZE to be 64, but increasing KD_LEAF_SIZE leads to perfomance decrease in narrow phase.)
			static constexpr uint32_t KD_LEAF_SIZE = 16;

			static constexpr uint32_t INVALID_INDEX = -1;

			Real minX, maxX, minY, maxY; // Merged AABB of all bodies in this subtree.
			uint32_t leftChildIndex = INVALID_INDEX; // INVALID_INDEX for leaves.
			// rightChildIndex = leftChildIndex + 1.
			uint32_t start, end; // Range in kdIndices: [start, end).
			uint32_t leafIndex; // If node is a leaf, it's its index.
		
			BvhNode() :
				start(0), end(0)
			{}

			BvhNode(uint32_t start, uint32_t end) :
				start(start), end(end)
			{}
		};
	private:
		struct BvhNodePair
		{
			uint32_t a, b;

			BvhNodePair swap() const noexcept { return { b, a }; }
		};

		struct alignas(64) PairVector
		{
			std::vector<BodyPair> pairs;
		};

		struct PackedBodyIndex
		{
			uint32_t key;
			BodyIndex index;
		};

		struct BvhFunctionResources
		{
			std::vector<BvhNode> nodes;

			SimdAlignedVector<Real> transformedCentroidX;
			SimdAlignedVector<Real> transformedCentroidY;

			SimdAlignedVector<uint32_t> mortonCodes;

			std::vector<BodyIndex> mainBodyIndices;
			std::vector<PackedBodyIndex> tempPackedBodyIndicesToSort;
			std::vector<PackedBodyIndex> tempPackedBodyIndicesToSort2;

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
				std::vector<BodyPair> collisionData;

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
				Real data[BvhNode::KD_LEAF_SIZE];
			};

			RealSimdAlignedVector<LeafData> minX;
			RealSimdAlignedVector<LeafData> maxX;
			RealSimdAlignedVector<LeafData> minY;
			RealSimdAlignedVector<LeafData> maxY;
		};

		AABBSoAViewer bodiesAABB;
		BvhFunctionResources bvhFunctionResources;
		QueryPairsThreadedResources queryPairsThreadedResources;

		LeafBodyAABBSoA leafBodyAABBs;

		std::vector<BodyPair> collisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = delete;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = delete;

		void setDataViewers(
			const AABBSoAViewer& aabbs
		);

		const std::vector<BodyPair>& findCollisions(bool rebuild);

		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;
	private:
		void computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax);

		template<std::floating_point TReal>
		void computeMortonCodes(uint32_t bodyCount);

		void sortBodyIndicesByMortonCodes(uint32_t bodyCount);

		void buildBvhTree(const uint32_t bodyCount);

		void fitBvhNodeAABBs();

		void queryBvhPairs();
		void queryBvhPairsThreaded();

		void traverseNodesToGetOverlappingLeafPairs();
		void testCollisionsInLeaves();
	};
}

