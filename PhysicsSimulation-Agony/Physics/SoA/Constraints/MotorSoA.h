#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/ContainerUtilities.h"

#include <algorithm>
#include <vector>

namespace PS_AGONY
{
	// Single-body angular motor: drives angular velocity toward targetSpeed, torque limited by maxTorque.
	struct MotorSoA
	{
		std::vector<ObjectIndex> bodyIndex;
		std::vector<Real> targetSpeed;
		std::vector<Real> maxTorque;
		std::vector<Real> accumulatedImpulse; // Persistent, used for warm-starting.

		ObjectIndex append(ObjectIndex index, Real speed, Real torqueLimit)
		{
			const ObjectIndex newIndex = bodyIndex.size();

			bodyIndex.push_back(index);
			targetSpeed.push_back(speed);
			maxTorque.push_back(torqueLimit);
			accumulatedImpulse.push_back(Real(0));

			return newIndex;
		}

		size_t getCount() const noexcept { return bodyIndex.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndex) +
				PS_AGONY::getVectorMemoryUsage(targetSpeed) +
				PS_AGONY::getVectorMemoryUsage(maxTorque) +
				PS_AGONY::getVectorMemoryUsage(accumulatedImpulse);
		}

		size_t swapRemove(size_t index)
		{
			const size_t lastIndex = bodyIndex.size() - 1;

			if (index != lastIndex)
			{
				std::swap(bodyIndex[index], bodyIndex[lastIndex]);
				std::swap(targetSpeed[index], targetSpeed[lastIndex]);
				std::swap(maxTorque[index], maxTorque[lastIndex]);
				std::swap(accumulatedImpulse[index], accumulatedImpulse[lastIndex]);
			}

			bodyIndex.pop_back();
			targetSpeed.pop_back();
			maxTorque.pop_back();
			accumulatedImpulse.pop_back();

			return lastIndex;
		}
	};

	class MotorSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndex = nullptr;
		const Real* targetSpeed = nullptr;
		const Real* maxTorque = nullptr;
		Real* accumulatedImpulse = nullptr; // Written by the solver.

		MotorSoAViewer() = default;

		explicit MotorSoAViewer(MotorSoA& data) noexcept :
			count(data.getCount()),
			bodyIndex(data.bodyIndex.data()),
			targetSpeed(data.targetSpeed.data()),
			maxTorque(data.maxTorque.data()),
			accumulatedImpulse(data.accumulatedImpulse.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}
