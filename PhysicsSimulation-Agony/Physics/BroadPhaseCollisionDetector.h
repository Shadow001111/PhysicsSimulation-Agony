#pragma once
#include "BodySoAViewer.h"

#include <vector>

namespace PS_AGONY
{
	class BroadPhaseCollisionDetector
	{
		using MortonCode = uint32_t;

		struct BvhNode
		{
			static constexpr uint32_t KD_LEAF_SIZE = 8;
			static constexpr uint32_t INVALID_INDEX = -1;

			Real minX, maxX, minY, maxY; // Merged AABB of all bodies in this subtree.
			uint32_t left, right; // Child node indices; INVALID_INDEX for leaves.
			uint32_t start, end; // Range in kdIndices: [start, end).
		
			BvhNode() :
				left(INVALID_INDEX), right(INVALID_INDEX),
				start(0), end(0)
			{}

			BvhNode(uint32_t start, uint32_t end) :
				left(INVALID_INDEX), right(INVALID_INDEX),
				start(start), end(end)
			{}
		};

		struct BvhNodePair { uint32_t a, b; };

		struct alignas(Simd<Real>::bytes) LeafAABB
		{
			static_assert(BvhNode::KD_LEAF_SIZE % Simd<Real>::lanes == 0, "KD_LEAF_SIZE must be a multiple of SIMD lanes.");

			Real minX[BvhNode::KD_LEAF_SIZE];
			Real maxX[BvhNode::KD_LEAF_SIZE];
			Real minY[BvhNode::KD_LEAF_SIZE];
			Real maxY[BvhNode::KD_LEAF_SIZE];
		};

		struct BvhFunctionResources
		{
			std::vector<BvhNode> nodeVector;

			SimdAlignedVector<Real> transformedCentroidX;
			SimdAlignedVector<Real> transformedCentroidY;

			SimdAlignedVector<MortonCode> mortonCodes;

			SimdAlignedVector<Real> leafMinX;
			SimdAlignedVector<Real> leafMaxX;
			SimdAlignedVector<Real> leafMinY;
			SimdAlignedVector<Real> leafMaxY;

			std::vector<BodyIndex> bodyIndexVector1;
		};

		AABBSoAViewer bodiesAABB;
		BvhFunctionResources bvhFunctionResources;

		std::vector<BodyPair> broadCollisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = default;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = default;

		const std::vector<BodyPair>& findCollisions(const AABBSoAViewer& bodiesAABBViewer);

		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;
	private:
		void findCollisionsBVH(size_t bodyCount);

		void computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax);

		void computeMortonCodes(uint32_t bodyCount);

		void buildBvhTree(
			std::vector<BvhNode>& nodes,
			std::vector<BodyIndex>& indices,
			const uint32_t bodyCount
		);

		void reorderAABBByIndices(const std::vector<BodyIndex>& indices);

		void queryBvhPairs(
			const std::vector<BvhNode>& nodes,
			const std::vector<BodyIndex>& indices);
	};
}

