#pragma once
#include "BodySoAViewer.h"

#include <vector>
#include "robin_hood.h"

namespace PS_AGONY
{
	class BroadPhaseCollisionDetector
	{
		struct BvhNode
		{
			static constexpr uint32_t KD_LEAF_SIZE = 8;
			static constexpr uint32_t INVALID_INDEX = -1;

			Real minX, maxX, minY, maxY; // Merged AABB of all bodies in this subtree.
			uint32_t left, right; // Child node indices; INVALID_INDEX for leaves.
			uint32_t start, end; // Range in kdIndices: [start, end).
		};

		struct BvhNodePair { uint32_t a, b; };

		struct alignas(Simd<Real>::bytes) LeafAABB
		{
			// TODO: Ensure 'KD_LEAF_SIZE' is a multiple of SIMD lanes for efficient processing.
			//static_assert(BvhNode::KD_LEAF_SIZE % Simd<Real>::lanes == 0, "KD_LEAF_SIZE must be a multiple of SIMD lanes.");

			static_assert(BvhNode::KD_LEAF_SIZE == Simd<Real>::lanes, "KD_LEAF_SIZE must be equal to SIMD lanes.");

			Real minX[BvhNode::KD_LEAF_SIZE];
			Real maxX[BvhNode::KD_LEAF_SIZE];
			Real minY[BvhNode::KD_LEAF_SIZE];
			Real maxY[BvhNode::KD_LEAF_SIZE];
		};

		struct FunctionResources
		{
			std::vector<BodyIndex> bodyIndexVector1;
			std::vector<BodyIndex> bodyIndexVector2;

			std::vector<BvhNode> bvhNodeVector1;

			SimdAlignedVector<Real> centroidX;
			SimdAlignedVector<Real> centroidY;

			robin_hood::unordered_flat_map<uint64_t, std::vector<BodyIndex>> spaceGrid;

			robin_hood::unordered_flat_set<uint64_t> uint64Set;
		};

		AABBSoAViewer bodiesAABB;
		FunctionResources functionResources;

		std::vector<BodyPair> broadCollisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = default;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = default;

		const std::vector<BodyPair>& findCollisions(const AABBSoAViewer& bodiesAABBViewer);
	private:
		void sweepAndPruneXAxis(size_t bodyCount);
		void boundVolumeHierarchy(size_t bodyCount);
		//void uniformSpaceGrid(size_t bodyCount);
	private:
		void buildBvhNode(
			std::vector<BvhNode>& nodes,
			std::vector<BodyIndex>& indices,
			uint32_t bodyCount);

		void queryBvhPairs(
			const std::vector<BvhNode>& nodes,
			const std::vector<BodyIndex>& indices);
	};
}

