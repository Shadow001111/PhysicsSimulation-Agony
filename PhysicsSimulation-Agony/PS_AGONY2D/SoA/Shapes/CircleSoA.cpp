#include "CircleSoA.h"
#include <utility>

namespace PS_AGONY2D
{
	void CircleSoA::append(
		ColliderDenseIndex colliderIndex,
		Real radius
	)
	{
		this->colliderIndices.push_back(colliderIndex);
		this->radius.push_back(radius);
	}

	void CircleSoA::swapWithBack(size_t index)
	{
		if (index >= colliderIndices.size()) [[unlikely]]
		{
			return;
		}

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void CircleSoA::popBack()
	{
		if (colliderIndices.empty()) [[unlikely]]
		{
			return;
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t CircleSoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	CircleSoAViewer::CircleSoAViewer(const CircleSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
