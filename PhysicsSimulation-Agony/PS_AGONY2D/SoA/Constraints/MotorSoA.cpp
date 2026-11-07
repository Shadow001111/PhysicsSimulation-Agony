#include "MotorSoA.h"

namespace PS_AGONY2D
{
	ObjectDenseIndex MotorSoA::append(ObjectDenseIndex index, Real speed, Real torqueLimit)
	{
		const ObjectDenseIndex newIndex = getCount();

		this->bodyIndex.push_back(index);
		this->targetSpeed.push_back(speed);
		this->maxTorque.push_back(torqueLimit);
		this->accumulatedImpulse.push_back(Real(0));

		handles.push();

		return newIndex;
	}

	void MotorSoA::swap(ObjectDenseIndex a, ObjectDenseIndex b)
	{
		if (a == b || a >= getCount() || b >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swap(a, b);

		#define SWAP_FIELD(name, type) std::swap(name[a], name[b]);
		PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(SWAP_FIELD)
		PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void MotorSoA::swapWithBack(size_t index)
	{
		if (index >= getCount()) [[unlikely]]
		{
			return;
		}
		handles.swapWithBack(index);

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(SWAP_FIELD)
		PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void MotorSoA::popBack()
	{
		if (getCount() == 0) [[unlikely]]
		{
			return;
		}
		handles.popBack();

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(POP_FIELD)
		PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t MotorSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = getCount() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t MotorSoA::getMemoryUsage() const noexcept
	{
		size_t total = handles.getMemoryUsage();

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(MEM_FIELD)
		PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		return total;
	}

	MotorSoAViewer::MotorSoAViewer(const MotorSoA& data) noexcept :
		MotorSoAViewer(data, IndexRange{ 0, static_cast<ObjectDenseIndex>(data.getCount()) })
	{
	}

	MotorSoAViewer::MotorSoAViewer(const MotorSoA& data, IndexRange range) noexcept :
		count(range.end - range.begin)
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data() + range.begin;
		PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
