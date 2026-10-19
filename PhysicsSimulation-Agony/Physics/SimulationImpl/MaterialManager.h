#pragma once
#include "Physics/Material.h"
#include "Physics/Types.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace PS_AGONY
{
	// Owns materials. Indices are stable: destroyed slots are recycled, never erased.
	// Slot 0 is the default material and cannot be destroyed.
	// A destroyed slot is reset to Material{}, so stale indices behave like the default material.
	class MaterialManager
	{
		std::vector<Material> materials;
		std::vector<MaterialIndex> freeList; // LIFO.
		std::vector<uint8_t> alive;
	public:
		MaterialManager()
		{
			materials.reserve(16);
			alive.reserve(16);
			materials.emplace_back();
			alive.push_back(1);
		}

		MaterialIndex create(const Material& material)
		{
			if (!freeList.empty())
			{
				const MaterialIndex index = freeList.back();
				freeList.pop_back();
				materials[index] = material;
				alive[index] = 1;
				return index;
			}

			const MaterialIndex index = static_cast<MaterialIndex>(materials.size());
			materials.push_back(material);
			alive.push_back(1);
			return index;
		}

		// Returns false for the default material, out-of-range and already dead slots.
		bool destroy(MaterialIndex index)
		{
			if (index == 0 || !isAlive(index))
				return false;

			materials[index] = Material{};
			alive[index] = 0;
			freeList.push_back(index);
			return true;
		}

		bool isAlive(MaterialIndex index) const noexcept
		{
			return static_cast<size_t>(index) < alive.size() && alive[index] != 0;
		}

		const std::vector<Material>& getMaterials() const noexcept { return materials; }
		const Material& get(MaterialIndex index) const noexcept { return materials[index]; }
		Material& get(MaterialIndex index) noexcept { return materials[index]; }

		size_t getMemoryUsage() const noexcept
		{
			return materials.capacity() * sizeof(Material)
				+ freeList.capacity() * sizeof(MaterialIndex)
				+ alive.capacity() * sizeof(uint8_t);
		}
	};
}
