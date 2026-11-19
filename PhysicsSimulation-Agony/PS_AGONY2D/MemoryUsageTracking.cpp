#include "MemoryUsageTracking.h"

namespace PS_AGONY2D
{
	void MemoryUsageTracking::reset()
	{
		nodes.clear();
	}

	void MemoryUsageTracking::finalize()
	{
		// Iterate over all nodes from back to front.
		// Because all children will point left to their parent.
		const size_t nodeCount = nodes.size();
		for (size_t i = nodeCount; i-- > 0;)
		{
			Node& node = nodes[i];
			node.childrenSumBytes = 0;
			uint32_t childIndex = node.childIndex;
			while (childIndex != INVALID)
			{
				Node& child = nodes[childIndex];
				node.childrenSumBytes += child.ownBytes + child.childrenSumBytes;
				childIndex = child.neighborIndex;
			}
		}
	}

	uint32_t MemoryUsageTracking::createNode(const char* string, size_t bytes, uint32_t parentIndex)
	{
		const uint32_t newIndex = nodes.size();
		Node& newNode = nodes.emplace_back(string, INVALID, INVALID, bytes);

		if (parentIndex >= newIndex) [[unlikely]]
		{
			if (newIndex == 0) [[unlikely]]
			{
				return newIndex; // Skip linking root to itself.
			}
			parentIndex = 0; // Link to root node.
		}

		// Link node by replacing parent's child with new node, and linking old child to new node as neighbor.
		Node& parentNode = nodes[parentIndex];
		newNode.neighborIndex = parentNode.childIndex;
		parentNode.childIndex = newIndex;

		return newIndex;
	}

	void MemoryUsageTracking::setNodeByteCount(uint32_t nodeIndex, size_t bytes)
	{
		if (nodeIndex < nodes.size()) [[likely]]
		{
			nodes[nodeIndex].ownBytes = bytes;
		}
	}
}