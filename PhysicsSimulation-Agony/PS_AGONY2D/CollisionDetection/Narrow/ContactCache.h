#pragma once
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/ContainerUtilities.h"

#include <bit>

namespace PS_AGONY2D
{
	struct ContactCache
	{
		// Open-addressing table: uint64 collider-pair key -> index into previousCollisionData.
		// Rebuilt once per frame, never erased from, load factor <= 0.5, linear probing.
		static constexpr uint64_t EMPTY = ~0ull;
		static constexpr uint32_t NONE = ~0u;

		struct Slot { uint64_t key; uint32_t val; uint32_t pad; };

		CacheLineAlignedVector<Slot> slots;
		size_t mask = 0;
		uint32_t shift = 63;
		size_t capacity = 0;

		static uint64_t makeKey(ColliderDenseIndex a, ColliderDenseIndex b) noexcept
		{
			static_assert(sizeof(ColliderDenseIndex) <= 4);
			return (uint64_t(a) << 32) | uint64_t(b);
		}

		size_t slotOf(uint64_t key) const noexcept
		{
			return size_t((key * 0x9E3779B97F4A7C15ull) >> shift);
		}

		bool empty() const noexcept { return capacity == 0; }

		void reset(size_t maxEntries)
		{
			if (maxEntries == 0) { capacity = 0; return; }
			const size_t c = std::bit_ceil(std::max<size_t>(maxEntries * 2, 16));
			if (slots.size() < c) slots.resize(c);
			std::memset(slots.data(), 0xFF, c * sizeof(Slot));
			capacity = c;
			mask = c - 1;
			shift = 64 - unsigned(std::countr_zero(c));
		}

		// Duplicate key: last write wins.
		void insert(uint64_t key, uint32_t val) noexcept
		{
			Slot* s = slots.data();
			size_t i = slotOf(key);
			while (true)
			{
				if (s[i].key == EMPTY || s[i].key == key) { s[i].key = key; s[i].val = val; return; }
				i = (i + 1) & mask;
			}
		}

		uint32_t find(uint64_t key) const noexcept
		{
			const Slot* s = slots.data();
			size_t i = slotOf(key);
			while (true)
			{
				if (s[i].key == key) return s[i].val;
				if (s[i].key == EMPTY) return NONE;
				i = (i + 1) & mask;
			}
		}

		void prefetch(uint64_t key) const noexcept
		{
			_mm_prefetch(reinterpret_cast<const char*>(slots.data() + slotOf(key)), _MM_HINT_T0);
		}

		size_t memoryUsage() const { return slots.capacity() * sizeof(Slot); }
	};
}

