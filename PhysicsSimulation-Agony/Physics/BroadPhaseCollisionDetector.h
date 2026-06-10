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

		struct BvhNodePair { uint32_t a, b; };

		struct alignas(Simd<Real>::bytes) LeafBodyAABBs
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

			std::vector<BodyIndex> bodyIndexVector1;
			std::vector<BodyIndex> bodyIndexVector2;
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

		LeafBodyAABBSoA leafBodyAABBs;

		std::vector<BodyPair> collisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = default;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = default;

		const std::vector<BodyPair>& findCollisions(const AABBSoAViewer& bodiesAABBViewer, bool rebuild);

		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;
	private:
		void computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax);

		template<std::floating_point TReal>
		void computeMortonCodes(uint32_t bodyCount);

		void sortBodyIndicesByMortonCodes(uint32_t bodyCount);

		void buildBvhTree(
			std::vector<BvhNode>& nodes,
			std::vector<BodyIndex>& indices,
			const uint32_t bodyCount
		);

		void refitBvhNodeAABBS();

		void queryBvhPairs(
			const std::vector<BvhNode>& nodes,
			const std::vector<BodyIndex>& indices);
	};
}

