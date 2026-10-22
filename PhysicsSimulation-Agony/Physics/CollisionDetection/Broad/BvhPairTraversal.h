#pragma once
#include "BvhNode.h"
#include "BvhNodePair.h"

#include "Ecstasy/Core/Portablity.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace PS_AGONY::BroadPhaseInternal
{
	// Self-tree only. Walks down from the root to build the initial frontier:
	// leaves for self-cross testing, leaf/leaf pairs for AxB testing, and
	// node/node pairs that still need splitting (independent sub-traversals).
	inline void traverseRootForInitialPairs(
		const BvhNode* ECSTASY_RESTRICT nodesPtr,
		CacheLineAlignedVector<BvhNodePair>& outNodePairsToTraverse,
		CacheLineAlignedVector<BvhNodePair>& outLeafPairsToTestCollisions,
		CacheLineAlignedVector<uint32_t>& outLeavesToTestCollisions)
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
			if (nodeL.overlaps(nodeR))
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
		CacheLineAlignedVector<BvhNodePair>& out)
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

			if (nodeA.overlaps(nodesB[leftChildB]))
			{
				out.push_back({ pair.a, leftChildB });
			}
			if (nodeA.overlaps(nodesB[rightChildB]))
			{
				out.push_back({ pair.a, rightChildB });
			}
		}
		else
		{
			const uint32_t leftChildA = nodeA.leftChildIndex;
			const uint32_t rightChildA = leftChildA + 1;

			if (nodeB.overlaps(nodesA[leftChildA]))
			{
				out.push_back({ leftChildA, pair.b });
			}
			if (nodeB.overlaps(nodesA[rightChildA]))
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
		CacheLineAlignedVector<BvhNodePair>& stack,
		CacheLineAlignedVector<BvhNodePair>& outLeafPairs)
	{
		// Pushes the (left, right) child pairs the same way the old pop/leaf-check did, but
		// leaf/leaf pairs that would be popped immediately are written straight to the output.
		auto emitChildren = [&](
			const BvhNodePair left, const BvhNodePair right,
			const bool overlapL, const bool overlapR,
			const bool leafLeafL, const bool leafLeafR)
			{
				if (overlapR)
				{
					if (leafLeafR)
					{
						outLeafPairs.push_back(right);
						if (overlapL)
						{
							if (leafLeafL) outLeafPairs.push_back(left);
							else stack.push_back(left);
						}
					}
					else
					{
						if (overlapL) stack.push_back(left); // Must wait for right's subtree.
						stack.push_back(right);
					}
				}
				else if (overlapL)
				{
					if (leafLeafL) outLeafPairs.push_back(left);
					else stack.push_back(left);
				}
			};

		while (!stack.empty())
		{
			const BvhNodePair nodePair = stack.back();
			stack.pop_back();

			const BvhNode& nodeA = nodesA[nodePair.a];
			const BvhNode& nodeB = nodesB[nodePair.b];

			const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
			const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;
			if (aLeaf && bLeaf)
			{
				outLeafPairs.push_back(nodePair);
				continue;
			}

			const Real areaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
			const Real areaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

			const bool splitB = aLeaf || (!bLeaf && (areaB > areaA));
			if (splitB)
			{
				const uint32_t l = nodeB.leftChildIndex;
				const uint32_t r = l + 1;
				const BvhNode& cl = nodesB[l];
				const BvhNode& cr = nodesB[r];

				emitChildren(
					{ nodePair.a, l }, { nodePair.a, r },
					nodeA.overlaps(cl), nodeA.overlaps(cr),
					aLeaf && cl.leftChildIndex == BvhNode::INVALID_INDEX,
					aLeaf && cr.leftChildIndex == BvhNode::INVALID_INDEX);
			}
			else
			{
				const uint32_t l = nodeA.leftChildIndex;
				const uint32_t r = l + 1;
				const BvhNode& cl = nodesA[l];
				const BvhNode& cr = nodesA[r];

				emitChildren(
					{ l, nodePair.b }, { r, nodePair.b },
					nodeB.overlaps(cl), nodeB.overlaps(cr),
					bLeaf && cl.leftChildIndex == BvhNode::INVALID_INDEX,
					bLeaf && cr.leftChildIndex == BvhNode::INVALID_INDEX);
			}
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
		CacheLineAlignedVector<BvhNodePair>& outPairs,
		CacheLineAlignedVector<BvhNodePair>& scratch)
	{
		if (!nodesA[0].overlaps(nodesB[0])) return;

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