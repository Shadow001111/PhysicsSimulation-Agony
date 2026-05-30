#pragma once
//#include "BodySoA.h"
#include "BodySoAViewer.h"

#include <vector>

namespace PS_AGONY
{
	struct BodyPair
	{
		BodyIndex a, b;
	};

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

		BodySoAViewer bodies;

		std::vector<BodyPair> collidingBodyPairs;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = default;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = default;

		const std::vector<BodyPair>& findCollisions(const BodySoAViewer& bodiesViewer);
	private:
		void sweepAndPruneXAxis(size_t bodyCount);
		void boundVolumeHierarchy(size_t bodyCount);
		void uniformSpaceGrid(size_t bodyCount);
	private:
		uint32_t buildBvhNode(
			std::vector<BvhNode>& nodes,
			std::vector<BodyIndex>& indices,
			uint32_t start, uint32_t end);

		void queryBvhPairs(
			const std::vector<BvhNode>& nodes,
			const std::vector<BodyIndex>& indices,
			uint32_t nodeA, uint32_t nodeB);
	};
}

