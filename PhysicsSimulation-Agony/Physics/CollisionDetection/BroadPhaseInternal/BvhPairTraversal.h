#pragma once
#include "BvhNode.h"
#include "BvhNodePair.h"

#include "Physics/ObjectSoA.h"
#include "Ecstasy/Core/Portablity.h"

#include <cstdint>
#include <vector>

namespace PS_AGONY::BroadPhaseInternal
{
	inline bool overlaps(const BvhNode& a, const BvhNode& b) noexcept
	{
		return
			a.minX < b.maxX && a.maxX > b.minX &&
			a.minY < b.maxY && a.maxY > b.minY;
	}

	// Walks down from the root to build the initial frontier of work: leaves
	// ready for self-cross testing, leaf/leaf pairs ready for AxB testing, and
	// node/node pairs still needing further splitting. Each 'outNodePairsToTraverse'
	// entry roots an independent sub-traversal, which is what lets the threaded
	// path hand them out to workers as separate tasks.
	inline void traverseRootForInitialPairs(
		const BvhNode* ECSTASY_RESTRICT nodesPtr,
		std::vector<BvhNodePair>& outNodePairsToTraverse,
		std::vector<BvhNodePair>& outLeafPairsToTestCollisions,
		std::vector<uint32_t>& outLeavesToTestCollisions)
	{
		if (nodesPtr[0].leftChildIndex == BvhNode::INVALID_INDEX) [[unlikely]] // Root is leaf.
		{
			outLeavesToTestCollisions.push_back(0);
			return;
		}

		uint32_t stack[MAX_STACK_CAPACITY];
		uint32_t stackSize = 0;
		stack[stackSize++] = 0; // Root index.

		while (stackSize > 0)
		{
			const uint32_t nodeIndex = stack[--stackSize];
			const BvhNode& node = nodesPtr[nodeIndex];

			const uint32_t L = node.leftChildIndex;
			const uint32_t R = L + 1;

			const BvhNode& nodeL = nodesPtr[L];
			const BvhNode& nodeR = nodesPtr[R];

			const bool lLeaf = nodeL.leftChildIndex == BvhNode::INVALID_INDEX;
			const bool rLeaf = nodeR.leftChildIndex == BvhNode::INVALID_INDEX;

			if (rLeaf)
			{
				outLeavesToTestCollisions.push_back(R);
			}
			else
			{
				stack[stackSize++] = R;
			}
			if (lLeaf)
			{
				outLeavesToTestCollisions.push_back(L);
			}
			else
			{
				stack[stackSize++] = L;
			}

			// Prune L and R nodes.
			if (overlaps(nodeL, nodeR))
			{
				if (lLeaf && rLeaf)
				{
					outLeafPairsToTestCollisions.emplace_back(L, R);
				}
				else
				{
					outNodePairsToTraverse.emplace_back(L, R);
				}
			}
		}
	}

	// Drains 'stack' (a LIFO of node-pairs still needing splitting), recursively
	// splitting the larger of the two nodes and pushing surviving sub-pairs back
	// onto 'stack', until every pair has either been discarded (no overlap) or
	// resolved to a leaf/leaf pair (appended to 'outLeafPairsToTestCollisions').
	// Used identically by the single-threaded path (stack pre-seeded with the
	// whole initial frontier, drained once) and by each worker thread (stack
	// re-seeded with one stolen task at a time and drained per task).
	inline void traverseNodePairStack(
		const BvhNode* ECSTASY_RESTRICT nodesPtr,
		std::vector<BvhNodePair>& stack,
		std::vector<BvhNodePair>& outLeafPairsToTestCollisions)
	{
		while (stack.size() > 0)
		{
			const BvhNodePair nodePair = stack.back();
			stack.pop_back();

			const BvhNode& nodeA = nodesPtr[nodePair.a];
			const BvhNode& nodeB = nodesPtr[nodePair.b];

			const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
			const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;
			if (aLeaf && bLeaf)
			{
				outLeafPairsToTestCollisions.emplace_back(nodePair);
				continue;
			}

			const Real nodeAreaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
			const Real nodeAreaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

			const bool splitB = aLeaf || (!bLeaf && (nodeAreaB > nodeAreaA));
			if (splitB)
			{
				// Split node B: check overlap with each child before pushing.
				const uint32_t leftChildB = nodeB.leftChildIndex;
				const uint32_t rightChildB = leftChildB + 1;

				const BvhNode& leftNodeB = nodesPtr[leftChildB];
				const BvhNode& rightNodeB = nodesPtr[rightChildB];

				if (overlaps(nodeA, leftNodeB))
				{
					stack.emplace_back(nodePair.a, leftChildB);
				}
				if (overlaps(nodeA, rightNodeB))
				{
					stack.emplace_back(nodePair.a, rightChildB);
				}
			}
			else
			{
				// Split node A: check overlap with each child before pushing.
				const uint32_t leftChildA = nodeA.leftChildIndex;
				const uint32_t rightChildA = leftChildA + 1;

				const BvhNode& leftNodeA = nodesPtr[leftChildA];
				const BvhNode& rightNodeA = nodesPtr[rightChildA];

				if (overlaps(leftNodeA, nodeB))
				{
					stack.emplace_back(leftChildA, nodePair.b);
				}
				if (overlaps(rightNodeA, nodeB))
				{
					stack.emplace_back(rightChildA, nodePair.b);
				}
			}
		}
	}
}