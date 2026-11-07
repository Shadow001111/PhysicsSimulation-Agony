#include "SliderJointSoA.h"

namespace PS_AGONY2D
{
	ObjectDenseIndex SliderJointSoA::append(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Vec2 anchorA, Vec2 anchorB, Vec2 axisA, Real stiffness, Real damping, Real lowerLimit, Real upperLimit, bool enableLimit)
	{
		const ObjectDenseIndex newIndex = getCount();

		this->bodyIndexPair.emplace_back(indexA, indexB);
		this->localAnchorA.push_back(anchorA);
		this->localAnchorB.push_back(anchorB);
		this->localAxisA.push_back(axisA);
		this->stiffness.push_back(stiffness);
		this->damping.push_back(damping);
		this->lowerLimit.push_back(lowerLimit);
		this->upperLimit.push_back(upperLimit);
		this->enableLimit.push_back(enableLimit ? uint8_t(1) : uint8_t(0));

		handles.push();

		return newIndex;
	}

	void SliderJointSoA::swap(ObjectDenseIndex a, ObjectDenseIndex b)
	{
		if (a == b || a >= getCount() || b >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swap(a, b);

		#define SWAP_FIELD(name, type) std::swap(name[a], name[b]);
		PS_AGONY_SLIDER_JOINT_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void SliderJointSoA::swapWithBack(size_t index)
	{
		if (index >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swapWithBack(index);

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_SLIDER_JOINT_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void SliderJointSoA::popBack()
	{
		if (getCount() == 0) [[unlikely]]
		{
			return;
		}
		handles.popBack();

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_SLIDER_JOINT_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t SliderJointSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = getCount() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t SliderJointSoA::getMemoryUsage() const noexcept
	{
		size_t total = handles.getMemoryUsage();

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_SLIDER_JOINT_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	SliderJointSoAViewer::SliderJointSoAViewer(const SliderJointSoA& data) noexcept :
		SliderJointSoAViewer(data, IndexRange{ 0, static_cast<ObjectDenseIndex>(data.getCount()) })
	{
	}

	SliderJointSoAViewer::SliderJointSoAViewer(const SliderJointSoA& data, IndexRange range) noexcept :
		count(range.end - range.begin)
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data() + range.begin;
		PS_AGONY_SLIDER_JOINT_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
