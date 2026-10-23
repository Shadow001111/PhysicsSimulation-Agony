#include "PrismaticJointSoA.h"

#include <utility>


namespace PS_AGONY2D
{
	ObjectIndex PrismaticJointSoA::append(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Vec2 axisA, Real referenceAngle, Real stiffness, Real damping, Real lowerLimit, Real upperLimit, bool enableLimit)
	{
		const ObjectIndex newIndex = this->bodyIndexA.size();

		this->bodyIndexA.push_back(indexA);
		this->bodyIndexB.push_back(indexB);
		this->localAnchorA.push_back(anchorA);
		this->localAnchorB.push_back(anchorB);
		this->localAxisA.push_back(axisA);
		this->referenceAngle.push_back(referenceAngle);
		this->stiffness.push_back(stiffness);
		this->damping.push_back(damping);
		this->lowerLimit.push_back(lowerLimit);
		this->upperLimit.push_back(upperLimit);
		this->enableLimit.push_back(enableLimit ? uint8_t(1) : uint8_t(0));

		return newIndex;
	}

	size_t PrismaticJointSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = bodyIndexA.size() - 1;

		if (index != lastIndex)
		{
			#define SWAP_FIELD(name, type) std::swap(name[index], name[lastIndex]);
			PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(SWAP_FIELD)
			#undef SWAP_FIELD
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD

		return lastIndex;
	}

	size_t PrismaticJointSoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	PrismaticJointSoAViewer::PrismaticJointSoAViewer(const PrismaticJointSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
