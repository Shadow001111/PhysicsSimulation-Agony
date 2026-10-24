#pragma once
#include "PS_AGONY2D/Types/CollisionEvent.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <span>

namespace PS_AGONY2D
{
	// Owns collision callbacks. Indices are stable: destroyed slots are recycled, never erased.
	// Slot 0 is a permanent no-op and cannot be destroyed.
	class CollisionCallbackManager
	{
		struct Entry
		{
			CollisionCallback function = nullptr;
			void* userData = nullptr;
		};

		static constexpr size_t MAX_SLOTS = size_t(UINT16_MAX);

		std::vector<Entry> entries;
		std::vector<CollisionCallbackIndex> freeList; // LIFO.
		std::vector<uint8_t> alive;
		size_t aliveCount = 0; // Excluding slot 0.
	public:
		CollisionCallbackManager()
		{
			entries.reserve(16);
			alive.reserve(16);
			entries.emplace_back();
			alive.push_back(1);
		}

		// nullopt if 'function' is null or all 65535 slots are in use.
		std::optional<CollisionCallbackIndex> create(CollisionCallback function, void* userData)
		{
			if (function == nullptr)
				return std::nullopt;

			CollisionCallbackIndex index;
			if (!freeList.empty())
			{
				index = freeList.back();
				freeList.pop_back();
				entries[index] = { function, userData };
				alive[index] = 1;
			}
			else
			{
				if (entries.size() >= MAX_SLOTS)
					return std::nullopt;

				index = static_cast<CollisionCallbackIndex>(entries.size());
				entries.push_back({ function, userData });
				alive.push_back(1);
			}
			aliveCount++;
			return index;
		}

		// Returns false for slot 0, out-of-range and already dead slots.
		bool destroy(CollisionCallbackIndex index)
		{
			if (index == 0 || !isAlive(index))
				return false;

			entries[index] = {};
			alive[index] = 0;
			freeList.push_back(index);
			aliveCount--;
			return true;
		}

		bool isAlive(CollisionCallbackIndex index) const noexcept
		{
			return static_cast<size_t>(index) < alive.size() && alive[index] != 0;
		}

		// Skips slot 0 and dead slots. Entry is copied first, so a handler may safely create/destroy callbacks.
		void invoke(const CollisionEvent& event) const
		{
			const CollisionCallbackIndex index = event.callbackId;
			if (index == 0 || !isAlive(index))
				return;

			const Entry entry = entries[index];
			entry.function(event, entry.userData);
		}

		void invoke(std::span<const CollisionEvent> events) const
		{
			for (const CollisionEvent& event : events)
			{
				const CollisionCallbackIndex index = event.callbackId;
				if (index == 0 || !isAlive(index))
					continue;

				const Entry entry = entries[index];
				entry.function(event, entry.userData);
			}
		}

		// True if at least one user callback is registered (slot 0 doesn't count).
		bool hasCallbacks() const noexcept { return aliveCount != 0; }

		size_t getMemoryUsage() const noexcept
		{
			return entries.capacity() * sizeof(Entry)
				+ freeList.capacity() * sizeof(CollisionCallbackIndex)
				+ alive.capacity() * sizeof(uint8_t);
		}
	};
}
