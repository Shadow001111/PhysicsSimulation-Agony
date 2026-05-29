#pragma once
#include <optional>
#include <concepts>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>

template<std::unsigned_integral TIndex>
class BlockAllocator
{
public:
	struct Block
	{
		TIndex offset = 0;
		TIndex size = 0;

		bool operator==(const Block& other) const
		{
			return offset == other.offset && size == other.size;
		}
	};

private:
	using FreeByOffset = std::map<TIndex, TIndex>;      // offset -> size
	using FreeBySize = std::multimap<TIndex, TIndex>;   // size -> offset

	TIndex capacity = 0;
	FreeByOffset freeByOffset;
	FreeBySize freeBySize;

	static constexpr TIndex maxValue() noexcept
	{
		return std::numeric_limits<TIndex>::max();
	}

	static constexpr bool addWouldOverflow(TIndex a, TIndex b) noexcept
	{
		return b > maxValue() - a;
	}

	static bool tryAdd(TIndex a, TIndex b, TIndex& out) noexcept
	{
		if (addWouldOverflow(a, b))
		{
			return false;
		}
		out = a + b;
		return true;
	}

	void addToSizeIndex(TIndex offset, TIndex size)
	{
		freeBySize.emplace(size, offset);
	}

	void removeFromSizeIndex(TIndex offset, TIndex size)
	{
		auto range = freeBySize.equal_range(size);
		for (auto it = range.first; it != range.second; ++it)
		{
			if (it->second == offset)
			{
				freeBySize.erase(it);
				return;
			}
		}
	}

	void eraseFreeBlock(typename FreeByOffset::iterator it)
	{
		removeFromSizeIndex(it->first, it->second);
		freeByOffset.erase(it);
	}

	void insertFreeBlock(TIndex offset, TIndex size)
	{
		if (size == 0)
		{
			return;
		}

		// Merge with left neighbor if adjacent.
		auto it = freeByOffset.lower_bound(offset);
		if (it != freeByOffset.begin())
		{
			auto left = std::prev(it);
			TIndex leftEnd = 0;
			if (tryAdd(left->first, left->second, leftEnd) && leftEnd == offset)
			{
				offset = left->first;
				if (addWouldOverflow(size, left->second))
				{
					return;
				}
				size += left->second;
				eraseFreeBlock(left);
				it = freeByOffset.lower_bound(offset);
			}
		}

		// Merge with right neighbor if adjacent.
		if (it != freeByOffset.end())
		{
			TIndex end = 0;
			if (tryAdd(offset, size, end) && end == it->first)
			{
				if (addWouldOverflow(size, it->second))
				{
					return;
				}
				size += it->second;
				eraseFreeBlock(it);
			}
		}

		freeByOffset.emplace(offset, size);
		addToSizeIndex(offset, size);
	}

	bool rangeIsFullyFree(TIndex from, TIndex to) const noexcept
	{
		if (from >= to)
		{
			return true;
		}

		auto it = freeByOffset.lower_bound(from);

		// If the previous free block covers `from`, start from it.
		if (it != freeByOffset.begin())
		{
			auto prev = std::prev(it);
			TIndex prevEnd = 0;
			if (tryAdd(prev->first, prev->second, prevEnd) && prevEnd > from)
			{
				it = prev;
			}
		}

		TIndex cursor = from;
		while (cursor < to)
		{
			if (it == freeByOffset.end())
			{
				return false;
			}

			TIndex blockEnd = 0;
			if (!tryAdd(it->first, it->second, blockEnd))
			{
				return false;
			}

			if (blockEnd <= cursor)
			{
				++it;
				continue;
			}

			if (it->first > cursor)
			{
				return false; // gap
			}

			cursor = blockEnd;
			++it;
		}

		return true;
	}

	void trimCapacity(TIndex newCapacity)
	{
		// Remove all free blocks that start at or beyond newCapacity.
		auto it = freeByOffset.lower_bound(newCapacity);

		// If the previous block crosses the new capacity, truncate it.
		if (it != freeByOffset.begin())
		{
			auto prev = std::prev(it);
			TIndex prevEnd = 0;
			if (tryAdd(prev->first, prev->second, prevEnd) && prevEnd > newCapacity)
			{
				removeFromSizeIndex(prev->first, prev->second);
				prev->second = newCapacity - prev->first;
				addToSizeIndex(prev->first, prev->second);
			}
		}

		while (it != freeByOffset.end())
		{
			auto current = it++;
			removeFromSizeIndex(current->first, current->second);
			freeByOffset.erase(current);
		}
	}

public:
	explicit BlockAllocator(TIndex capacity = 0) :
		capacity(capacity)
	{
		if (capacity > 0)
		{
			insertFreeBlock(0, capacity);
		}
	}

	~BlockAllocator() = default;

	BlockAllocator(const BlockAllocator&) = delete;
	BlockAllocator& operator=(const BlockAllocator&) = delete;
	BlockAllocator(BlockAllocator&&) = delete;
	BlockAllocator& operator=(BlockAllocator&&) = delete;

	std::optional<Block> allocate(TIndex size)
	{
		if (size == 0)
		{
			return std::nullopt;
		}

		// Best-fit: smallest free block that can satisfy the request.
		auto it = freeBySize.lower_bound(size);
		if (it == freeBySize.end())
		{
			return std::nullopt;
		}

		const TIndex blockSize = it->first;
		const TIndex offset = it->second;

		auto offIt = freeByOffset.find(offset);
		if (offIt == freeByOffset.end() || offIt->second != blockSize)
		{
			return std::nullopt; // Internal invariant broken.
		}

		eraseFreeBlock(offIt);

		const TIndex remaining = blockSize - size;
		if (remaining > 0)
		{
			TIndex newOffset = 0;
			if (!tryAdd(offset, size, newOffset))
			{
				return std::nullopt;
			}
			insertFreeBlock(newOffset, remaining);
		}

		return Block{ offset, size };
	}

	bool free(const Block& blockToFree)
	{
		if (blockToFree.size == 0)
		{
			return false;
		}

		TIndex end = 0;
		if (!tryAdd(blockToFree.offset, blockToFree.size, end))
		{
			return false;
		}

		if (end > capacity)
		{
			return false;
		}

		// Reject overlaps / double frees by checking neighboring free blocks.
		auto it = freeByOffset.lower_bound(blockToFree.offset);

		if (it != freeByOffset.begin())
		{
			auto left = std::prev(it);
			TIndex leftEnd = 0;
			if (!tryAdd(left->first, left->second, leftEnd))
			{
				return false;
			}
			if (leftEnd > blockToFree.offset)
			{
				return false; // overlaps existing free block
			}
		}

		if (it != freeByOffset.end() && it->first < end)
		{
			return false; // overlaps existing free block
		}

		insertFreeBlock(blockToFree.offset, blockToFree.size);
		return true;
	}

	bool setCapacity(TIndex newCapacity)
	{
		if (newCapacity == capacity)
		{
			return true;
		}

		if (newCapacity > capacity)
		{
			// New tail becomes free.
			const TIndex added = newCapacity - capacity;
			insertFreeBlock(capacity, added);
			capacity = newCapacity;
			return true;
		}

		// Shrinking: only allowed if the truncated tail is fully free.
		if (!rangeIsFullyFree(newCapacity, capacity))
		{
			return false;
		}

		trimCapacity(newCapacity);
		capacity = newCapacity;
		return true;
	}

	// Debug / inspection
	TIndex getCapacity() const noexcept
	{
		return capacity;
	}

	TIndex getLastFreeBlockEnd() const noexcept
	{
		if (freeByOffset.empty())
		{
			return 0;
		}

		const auto& last = freeByOffset.rbegin();
		TIndex end = 0;
		if (!tryAdd(last->first, last->second, end))
		{
			return maxValue();
		}
		return end;
	}

	TIndex getLastAllocatedBlockEnd() const noexcept
	{
		TIndex cursor = 0;
		TIndex lastAllocatedEnd = 0;

		for (const auto& [freeOffset, freeSize] : freeByOffset)
		{
			if (freeOffset > cursor)
			{
				// Found allocated block
				lastAllocatedEnd = freeOffset;
			}

			TIndex freeEnd = 0;
			if (!tryAdd(freeOffset, freeSize, freeEnd))
			{
				return maxValue();
			}

			cursor = freeEnd;
		}

		// If there's allocation after last free block
		if (cursor < capacity)
		{
			lastAllocatedEnd = capacity;
		}

		return lastAllocatedEnd;
	}

	std::optional<std::pair<TIndex, TIndex>> getAllocatedRangeBounds() const noexcept
	{
		TIndex cursor = 0;

		std::optional<TIndex> firstStart;
		TIndex lastEnd = 0;

		for (const auto& [freeOffset, freeSize] : freeByOffset)
		{
			// If there is an allocated region before this free block
			if (freeOffset > cursor)
			{
				if (!firstStart.has_value())
				{
					firstStart = cursor;
				}

				lastEnd = freeOffset;
			}

			TIndex freeEnd = 0;
			if (!tryAdd(freeOffset, freeSize, freeEnd))
			{
				return std::nullopt; // overflow / invalid state
			}

			cursor = freeEnd;
		}

		// Tail allocation after last free block
		if (cursor < capacity)
		{
			if (!firstStart.has_value())
			{
				firstStart = cursor;
			}

			lastEnd = capacity;
		}

		if (!firstStart.has_value())
		{
			// No allocations at all
			return std::nullopt;
		}

		return std::pair<TIndex, TIndex>{ *firstStart, lastEnd };
	}

	TIndex getFreeSpaceSum() const noexcept
	{
		TIndex total = 0;
		for (const auto& [offset, size] : freeByOffset)
		{
			(void)offset;
			if (size > maxValue() - total)
			{
				return maxValue();
			}
			total += size;
		}
		return total;
	}

	std::vector<Block> getAllFreeBlocks() const
	{
		std::vector<Block> out;
		out.reserve(freeByOffset.size());

		for (const auto& [offset, size] : freeByOffset)
		{
			out.push_back(Block{ offset, size });
		}

		return out;
	}

	std::vector<Block> getAllAllocatedBlocks() const
	{
		std::vector<Block> result;

		TIndex cursor = 0;

		for (const auto& [freeOffset, freeSize] : freeByOffset)
		{
			if (freeOffset > cursor)
			{
				// Allocated gap before this free block
				result.push_back(Block{ cursor, freeOffset - cursor });
			}

			TIndex freeEnd = 0;
			if (!tryAdd(freeOffset, freeSize, freeEnd))
			{
				return {}; // overflow -> invalid state
			}

			cursor = freeEnd;
		}

		// Tail allocation after last free block
		if (cursor < capacity)
		{
			result.push_back(Block{ cursor, capacity - cursor });
		}

		return result;
	}
};