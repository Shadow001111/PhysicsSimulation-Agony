#pragma once
#include "ContainerUtilities.h"

#include <cstdint>
#include <span>

namespace PS_AGONY2D
{
	class MemoryUsageTracking
	{
		constexpr static uint32_t INVALID = -1;
	public:
		struct Node
		{
			const char* string = nullptr;

			uint32_t childIndex = INVALID;
			uint32_t neighborIndex = INVALID;

			size_t ownBytes = 0;
			size_t childrenSumBytes = 0;

			Node(
				const char* string,
				uint32_t childIndex,
				uint32_t neighborIndex,
				size_t ownBytes
			) :
				string(string),
				childIndex(childIndex),
				neighborIndex(neighborIndex),
				ownBytes(ownBytes)
			{}
		};

		class Iterator
		{
		public:
			struct Item
			{
				const Node& node;
				uint32_t depth;
			};

			using value_type = const Node;
			using difference_type = std::ptrdiff_t;
			using iterator_concept = std::forward_iterator_tag;
		private:
			struct TreePosition
			{
				uint32_t nodeIndex = 0;
				uint32_t nodeDepth = 0;
			};

			std::span<const Node> nodes{};
			TreePosition currentPosition{};
			std::vector<TreePosition> stack;
		public:
			constexpr Iterator() = default;

			constexpr Iterator(std::span<const Node> nodes, size_t beginIndex) :
				nodes(nodes), currentPosition(beginIndex, 0)
			{}

			constexpr Item operator*() const
			{
				Item item
				{
					nodes[currentPosition.nodeIndex],
					currentPosition.nodeDepth
				};
				return item;
			}

			constexpr Iterator& operator++()
			{
				advance();
				return *this;
			}

			constexpr Iterator operator++(int)
			{
				auto t = *this;
				advance();
				return t;
			}

			friend constexpr bool operator==(const Iterator& a, const Iterator& b)
			{
				return a.currentPosition.nodeIndex == b.currentPosition.nodeIndex;
			}
		private:
			void advance()
			{
				const uint32_t currentIndex = currentPosition.nodeIndex;
				if (currentIndex >= nodes.size()) [[unlikely]]
				{
					return;
				}

				const Node& currentNode = nodes[currentIndex];

				// Traverse children.
				if (currentNode.childIndex != INVALID)
				{
					// Save neighbor to stack.
					if (currentNode.neighborIndex != INVALID)
					{
						stack.emplace_back(currentNode.neighborIndex, currentPosition.nodeDepth);
					}

					// Advance.
					currentPosition.nodeIndex = currentNode.childIndex;
					currentPosition.nodeDepth++;
					return;
				}
				
				// Traverse neighbors.
				if (currentNode.neighborIndex != INVALID)
				{
					currentPosition.nodeIndex = currentNode.neighborIndex;
					return;
				}

				// Return to position stored in stack.
				if (stack.empty()) [[unlikely]]
				{
					// End.
					currentPosition.nodeIndex = nodes.size();
					currentPosition.nodeDepth = -1;
					return;
				}
				currentPosition = stack.back();
				stack.pop_back();
			}
		};
	private:
		CacheLineAlignedVector<Node> nodes;
	public:
		MemoryUsageTracking()
		{
			reset();
		}

		// Use before new frame starts.
		void reset();

		// Sums up all node byte sizes and completes the tree.
		// Use it before using for results.
		void finalize();

		// Creates node and makes it child of pare node if index is valid.
		// Returns index of new node.
		uint32_t createNode(const char* string, size_t bytes, uint32_t parentIndex = INVALID);

		void setNodeByteCount(uint32_t nodeIndex, size_t bytes);

		Iterator begin() const 
		{
			return Iterator(nodes, 0);
		}

		Iterator end() const
		{
			return Iterator(nodes, nodes.size());
		}
	};
}

