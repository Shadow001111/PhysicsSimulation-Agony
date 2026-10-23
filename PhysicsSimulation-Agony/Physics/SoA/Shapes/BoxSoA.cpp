#include "BoxSoA.h"
#include <utility>

namespace PS_AGONY2D
{
	void BoxSoA::append(
		ColliderIndex colliderIndex,
		Real halfWidth,
		Real halfHeight
	)
	{
		this->colliderIndices.push_back(colliderIndex);
		this->halfWidth.push_back(halfWidth);
		this->halfHeight.push_back(halfHeight);
	}

	void BoxSoA::swapWithBack(size_t index)
	{
		if (index >= colliderIndices.size()) [[unlikely]]
		{
			return;
		}

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_BOX_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void BoxSoA::popBack()
	{
		if (colliderIndices.empty()) [[unlikely]]
		{
			return;
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_BOX_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t BoxSoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_BOX_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	BoxSoAViewer::BoxSoAViewer(const BoxSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_BOX_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
