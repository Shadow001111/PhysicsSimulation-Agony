#include "RevoluteJointSoA.h"

namespace PS_AGONY2D
{
	ObjectDenseIndex RevoluteJointSoA::append(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Vec2 anchorA, Vec2 anchorB, Real stiffness, Real damping)
	{
		const ObjectDenseIndex newIndex = getCount();

		this->bodyIndexPair.emplace_back(indexA, indexB);
		this->localAnchorA.push_back(anchorA);
		this->localAnchorB.push_back(anchorB);
		this->stiffness.push_back(stiffness);
		this->damping.push_back(damping);

		handles.push();

		return newIndex;
	}

	void RevoluteJointSoA::swap(ObjectDenseIndex a, ObjectDenseIndex b)
	{
		if (a == b || a >= getCount() || b >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swap(a, b);

		#define SWAP_FIELD(name, type) std::swap(name[a], name[b]);
		PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void RevoluteJointSoA::swapWithBack(size_t index)
	{
		if (index >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swapWithBack(index);

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void RevoluteJointSoA::popBack()
	{
		if (getCount() == 0) [[unlikely]]
		{
			return;
		}
		handles.popBack();

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t RevoluteJointSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = getCount() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t RevoluteJointSoA::getMemoryUsage() const noexcept
	{
		size_t total = handles.getMemoryUsage();

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	RevoluteJointSoAViewer::RevoluteJointSoAViewer(const RevoluteJointSoA& data) noexcept :
		RevoluteJointSoAViewer(data, IndexRange{ 0, static_cast<ObjectDenseIndex>(data.getCount()) })
	{
	}

	RevoluteJointSoAViewer::RevoluteJointSoAViewer(const RevoluteJointSoA& data, IndexRange range) noexcept :
		count(range.end - range.begin)
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data() + range.begin;
		PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
