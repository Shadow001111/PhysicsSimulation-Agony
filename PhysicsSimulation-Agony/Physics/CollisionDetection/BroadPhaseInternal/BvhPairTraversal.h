#pragma once
#include "BvhNode.h"
#include "BvhNodePair.h"

#include "Physics/ObjectSoA.h"
#include "Ecstasy/Core/Portablity.h"

#include <algorithm>
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

	// Self-tree only. Walks down from the root to build the initial frontier:
	// leaves for self-cross testing, leaf/leaf pairs for AxB testing, and
	// node/node pairs that still need splitting (independent sub-traversals).
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

	// Splits the larger node of 'pair' (a in nodesA, b in nodesB) and appends the
	// overlapping child pairs to 'out'. The pair must not be leaf/leaf.
	inline void splitLargerNode(
		const BvhNode* nodesA,
		const BvhNode* nodesB,
		const BvhNodePair pair,
		std::vector<BvhNodePair>& out)
	{
		const BvhNode& nodeA = nodesA[pair.a];
		const BvhNode& nodeB = nodesB[pair.b];

		const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
		const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;

		const Real areaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
		const Real areaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

		const bool splitB = aLeaf || (!bLeaf && (areaB > areaA));
		if (splitB)
		{
			const uint32_t leftChildB = nodeB.leftChildIndex;
			const uint32_t rightChildB = leftChildB + 1;

			if (overlaps(nodeA, nodesB[leftChildB]))
			{
				out.push_back({ pair.a, leftChildB });
			}
			if (overlaps(nodeA, nodesB[rightChildB]))
			{
				out.push_back({ pair.a, rightChildB });
			}
		}
		else
		{
			const uint32_t leftChildA = nodeA.leftChildIndex;
			const uint32_t rightChildA = leftChildA + 1;

			if (overlaps(nodesA[leftChildA], nodeB))
			{
				out.push_back({ leftChildA, pair.b });
			}
			if (overlaps(nodesA[rightChildA], nodeB))
			{
				out.push_back({ rightChildA, pair.b });
			}
		}
	}

	// Drains 'stack' down to leaf/leaf pairs (appended to 'outLeafPairsToTestCollisions').
	// Pairs are (node in nodesA, node in nodesB); pass the same array twice for self-tree traversal.
	inline void traverseNodePairStack(
		const BvhNode* nodesA,
		const BvhNode* nodesB,
		std::vector<BvhNodePair>& stack,
		std::vector<BvhNodePair>& outLeafPairsToTestCollisions)
	{
		while (!stack.empty())
		{
			const BvhNodePair nodePair = stack.back();
			stack.pop_back();

			const bool aLeaf = nodesA[nodePair.a].leftChildIndex == BvhNode::INVALID_INDEX;
			const bool bLeaf = nodesB[nodePair.b].leftChildIndex == BvhNode::INVALID_INDEX;
			if (aLeaf && bLeaf)
			{
				outLeafPairsToTestCollisions.push_back(nodePair);
				continue;
			}

			splitLargerNode(nodesA, nodesB, nodePair, stack);
		}
	}

	// Cross-tree only. Expands (rootA, rootB) level by level until at least 'minTasks'
	// pairs are pending (or none can be split further), PRESERVING the single-threaded
	// DFS output order: 'outPairs' is ordered exactly as traverseNodePairStack would
	// visit them (right child before left). Leaf/leaf pairs stay in the list as ordered
	// entries; draining each entry in sequence reproduces the single-threaded leaf-pair order.
	// 'outPairs' must be empty on entry; 'scratch' is a reusable buffer.
	inline void expandCrossRootPairsOrdered(
		const BvhNode* nodesA,
		const BvhNode* nodesB,
		size_t minTasks,
		std::vector<BvhNodePair>& outPairs,
		std::vector<BvhNodePair>& scratch)
	{
		if (!overlaps(nodesA[0], nodesB[0])) return;

		outPairs.push_back({ 0, 0 });

		while (outPairs.size() < minTasks)
		{
			scratch.clear();
			bool split = false;
			for (const BvhNodePair pair : outPairs)
			{
				const bool aLeaf = nodesA[pair.a].leftChildIndex == BvhNode::INVALID_INDEX;
				const bool bLeaf = nodesB[pair.b].leftChildIndex == BvhNode::INVALID_INDEX;
				if (aLeaf && bLeaf)
				{
					scratch.push_back(pair);
					continue;
				}

				const size_t before = scratch.size();
				splitLargerNode(nodesA, nodesB, pair, scratch); // Appends [left, right] (stack order).
				std::reverse(scratch.begin() + before, scratch.end()); // Output order: [right, left].
				split = true;
			}
			outPairs.swap(scratch);
			if (!split) break;
		}
	}
}