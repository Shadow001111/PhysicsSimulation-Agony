#include "RodSoA.h"

#include <utility>


namespace PS_AGONY2D
{
	ObjectDenseIndex RodSoA::append(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Vec2 anchorA, Vec2 anchorB, Real length)
	{
		const ObjectDenseIndex newIndex = this->bodyIndexA.size();

		this->bodyIndexA.push_back(indexA);
		this->bodyIndexB.push_back(indexB);
		this->localAnchorA.push_back(anchorA);
		this->localAnchorB.push_back(anchorB);
		this->length.push_back(length);

		handles.push();

		return newIndex;
	}

	void RodSoA::swapWithBack(size_t index)
	{
		if (index >= bodyIndexA.size()) [[unlikely]]
		{
			return;
		}
		handles.swapWithBack(index);

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_ROD_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void RodSoA::popBack()
	{
		if (bodyIndexA.empty()) [[unlikely]]
		{
			return;
		}
		handles.popBack();

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_ROD_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t RodSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = bodyIndexA.size() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t RodSoA::getMemoryUsage() const noexcept
	{
		size_t total = handles.getMemoryUsage();

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_ROD_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	RodSoAViewer::RodSoAViewer(const RodSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_ROD_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
