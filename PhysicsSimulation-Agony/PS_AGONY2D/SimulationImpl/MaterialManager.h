#pragma once
#include "PS_AGONY2D/Types/Material.h"
#include "PS_AGONY2D/Types/Types.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace PS_AGONY2D
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

		// Elasticity in [0, 1], frictions >= 0. fmin/fmax: safe under fp:fast.
		static Material sanitize(Material m) noexcept
		{
			m.elasticity = std::fmin(std::fmax(m.elasticity, Real(0)), Real(1));
			m.staticFriction = std::fmax(m.staticFriction, Real(0));
			m.dynamicFriction = std::fmax(m.dynamicFriction, Real(0));
			return m;
		}

		MaterialIndex create(const Material& material)
		{
			if (!freeList.empty())
			{
				const MaterialIndex index = freeList.back();
				freeList.pop_back();
				materials[index] = sanitize(material);
				alive[index] = 1;
				return index;
			}

			const MaterialIndex index = static_cast<MaterialIndex>(materials.size());
			materials.push_back(sanitize(material));
			alive.push_back(1);
			return index;
		}

		// Returns false for out-of-range and dead slots.
		bool set(MaterialIndex index, const Material& material)
		{
			if (!isAlive(index))
				return false;

			materials[index] = sanitize(material);
			return true;
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