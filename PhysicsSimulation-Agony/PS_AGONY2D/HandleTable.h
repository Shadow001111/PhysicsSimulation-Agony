#pragma once
#include <cstdint>
#include <vector>

namespace PS_AGONY2D
{
	struct Handle
	{
		static constexpr uint32_t NULL_DENSE_INDEX = -1;

		uint32_t slot = NULL_DENSE_INDEX;
		uint32_t generation = 0;

		friend constexpr bool operator==(const Handle& a, const Handle& b) noexcept
		{
			return a.slot == b.slot && a.generation == b.generation;
		}
	};

	// Tracks stable handles for an external dense array that uses swap-and-pop.
	class HandleTable
	{
		struct Slot
		{
			uint32_t index;
			uint32_t generation;
		};
		std::vector<Slot> slots; // Sparse, indexed by Handle::slot.
		std::vector<uint32_t> denseToSlot;
		uint32_t freeHead = Handle::NULL_DENSE_INDEX;
	public:
		// Registers a new element at the back of the dense array.
		Handle push()
		{
			uint32_t s;
			if (freeHead != Handle::NULL_DENSE_INDEX)
			{
				s = freeHead;
				freeHead = slots[s].index;
			}
			else
			{
				s = static_cast<uint32_t>(slots.size());
				slots.push_back({ 0, 0 });
			}
			slots[s].index = static_cast<uint32_t>(denseToSlot.size());
			denseToSlot.push_back(s);
			return { s, slots[s].generation };
		}

		// Mirrors swapping dense[index] with dense[back].
		void swapWithBack(uint32_t index) noexcept
		{
			if (index + 1 >= denseToSlot.size()) return; // Out of range or already last.
			const size_t last = denseToSlot.size() - 1;
			const uint32_t a = denseToSlot[index];
			const uint32_t b = denseToSlot[last];
			denseToSlot[index] = b;
			denseToSlot[last] = a;
			slots[b].index = static_cast<uint32_t>(index);
			slots[a].index = static_cast<uint32_t>(last);
		}

		// Mirrors popping the dense array. Invalidates the back element's handle.
		void popBack() noexcept
		{
			if (denseToSlot.empty()) return;
			const uint32_t s = denseToSlot.back();
			denseToSlot.pop_back();
			slots[s].generation++;
			slots[s].index = freeHead;
			freeHead = s;
		}

		bool isValid(Handle h) const noexcept
		{
			return h.slot < slots.size() && slots[h.slot].generation == h.generation;
		}

		// Handle -> dense index (Handle::NULL_DENSE_INDEX if invalid).
		uint32_t indexOf(Handle h) const noexcept
		{
			return isValid(h) ? slots[h.slot].index : Handle::NULL_DENSE_INDEX;
		}

		// Handle -> dense index with no validation. Caller guarantees the handle is alive.
		uint32_t indexOfUnchecked(Handle h) const noexcept
		{
			return slots[h.slot].index;
		}

		// Dense index -> handle. Index must be in range.
		Handle handleAt(uint32_t dense) const noexcept
		{
			const uint32_t s = denseToSlot[dense];
			return { s, slots[s].generation };
		}

		size_t getMemoryUsage() const noexcept
		{
			return slots.capacity() * sizeof(Slot) + denseToSlot.capacity() * sizeof(uint32_t);
		}
	};
}