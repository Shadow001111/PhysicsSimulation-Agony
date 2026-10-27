#pragma once
#include <cstdint>
#include <vector>

namespace PS_AGONY2D
{
	struct Handle
	{
		static constexpr uint32_t kNull = 0xFFFFFFFFu;
		uint32_t slot = kNull;
		uint32_t generation = 0;
		friend constexpr bool operator==(const Handle& a, const Handle& b) noexcept
		{
			return a.slot == b.slot && a.generation == b.generation;
		}
	};

	// Tracks stable handles for an external dense array that uses swap-and-pop.
	// Mirror every push / swapWithBack / popBack of the array here.
	class HandleTable
	{
		struct Slot
		{
			uint32_t index; // alive: dense index. free: next free slot.
			uint32_t generation;	// bumped on destroy
		};
		std::vector<Slot> slots;		   // sparse, indexed by Handle::slot
		std::vector<uint32_t> denseToSlot; // dense -> slot
		uint32_t freeHead = Handle::kNull;
	public:
		// Registers a new element at the back of the dense array.
		Handle push()
		{
			uint32_t s;
			if (freeHead != Handle::kNull)
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
			if (index + 1 >= denseToSlot.size()) return; // out of range or already last
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

		// Handle -> dense index (Handle::kNull if invalid).
		uint32_t indexOf(Handle h) const noexcept
		{
			return isValid(h) ? slots[h.slot].index : Handle::kNull;
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