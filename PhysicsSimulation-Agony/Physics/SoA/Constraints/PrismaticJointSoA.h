#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/ContainerUtilities.h"

#include <algorithm>
#include <cstdint>

namespace PS_AGONY
{
	// A prismatic (slider / piston) joint. Locks relative translation between two
	// bodies to a single axis attached to body A (so the axis rotates with A), and
	// locks relative rotation, like a piston rod sliding through a cylinder.
	// Optionally limited to [lowerLimit, upperLimit] travel along the axis.
	struct PrismaticJointSoA
	{
		std::vector<ObjectIndex> bodyIndexA;
		std::vector<ObjectIndex> bodyIndexB;
		std::vector<Vec2> localAnchorA;
		std::vector<Vec2> localAnchorB;
		std::vector<Vec2> localAxisA;      // Unit axis in body A's local space; translation is locked to this axis.
		std::vector<Real> referenceAngle;  // (bodyB.rotation - bodyA.rotation) captured at creation, wrapped to (-pi, pi].
		std::vector<Real> stiffness;
		std::vector<Real> damping;
		std::vector<Real> lowerLimit;
		std::vector<Real> upperLimit;
		std::vector<uint8_t> enableLimit;

		ObjectIndex append(
			ObjectIndex indexA, ObjectIndex indexB,
			Vec2 anchorA, Vec2 anchorB,
			Vec2 axisA, Real refAngle,
			Real stiffnessIn, Real dampingIn,
			Real lowerLimitIn, Real upperLimitIn, bool enableLimitIn)
		{
			const ObjectIndex newIndex = this->bodyIndexA.size();

			this->bodyIndexA.push_back(indexA);
			this->bodyIndexB.push_back(indexB);
			this->localAnchorA.push_back(anchorA);
			this->localAnchorB.push_back(anchorB);
			this->localAxisA.push_back(axisA);
			this->referenceAngle.push_back(refAngle);
			this->stiffness.push_back(stiffnessIn);
			this->damping.push_back(dampingIn);
			this->lowerLimit.push_back(lowerLimitIn);
			this->upperLimit.push_back(upperLimitIn);
			this->enableLimit.push_back(enableLimitIn ? uint8_t(1) : uint8_t(0));

			return newIndex;
		}

		size_t getCount() const noexcept { return bodyIndexA.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndexA) +
				PS_AGONY::getVectorMemoryUsage(bodyIndexB) +
				PS_AGONY::getVectorMemoryUsage(localAnchorA) +
				PS_AGONY::getVectorMemoryUsage(localAnchorB) +
				PS_AGONY::getVectorMemoryUsage(localAxisA) +
				PS_AGONY::getVectorMemoryUsage(referenceAngle) +
				PS_AGONY::getVectorMemoryUsage(stiffness) +
				PS_AGONY::getVectorMemoryUsage(damping) +
				PS_AGONY::getVectorMemoryUsage(lowerLimit) +
				PS_AGONY::getVectorMemoryUsage(upperLimit) +
				PS_AGONY::getVectorMemoryUsage(enableLimit);
		}

		size_t swapRemove(size_t index)
		{
			const size_t lastIndex = bodyIndexA.size() - 1;

			if (index != lastIndex)
			{
				std::swap(bodyIndexA[index], bodyIndexA[lastIndex]);
				std::swap(bodyIndexB[index], bodyIndexB[lastIndex]);
				std::swap(localAnchorA[index], localAnchorA[lastIndex]);
				std::swap(localAnchorB[index], localAnchorB[lastIndex]);
				std::swap(localAxisA[index], localAxisA[lastIndex]);
				std::swap(referenceAngle[index], referenceAngle[lastIndex]);
				std::swap(stiffness[index], stiffness[lastIndex]);
				std::swap(damping[index], damping[lastIndex]);
				std::swap(lowerLimit[index], lowerLimit[lastIndex]);
				std::swap(upperLimit[index], upperLimit[lastIndex]);
				std::swap(enableLimit[index], enableLimit[lastIndex]);
			}

			bodyIndexA.pop_back();
			bodyIndexB.pop_back();
			localAnchorA.pop_back();
			localAnchorB.pop_back();
			localAxisA.pop_back();
			referenceAngle.pop_back();
			stiffness.pop_back();
			damping.pop_back();
			lowerLimit.pop_back();
			upperLimit.pop_back();
			enableLimit.pop_back();

			return lastIndex;
		}
	};

	class PrismaticJointSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndexA = nullptr;
		const ObjectIndex* bodyIndexB = nullptr;
		const Vec2* localAnchorA = nullptr;
		const Vec2* localAnchorB = nullptr;
		const Vec2* localAxisA = nullptr;
		const Real* referenceAngle = nullptr;
		const Real* stiffness = nullptr;
		const Real* damping = nullptr;
		const Real* lowerLimit = nullptr;
		const Real* upperLimit = nullptr;
		const uint8_t* enableLimit = nullptr;

		PrismaticJointSoAViewer() = default;

		explicit PrismaticJointSoAViewer(const PrismaticJointSoA& data) noexcept :
			count(data.getCount()),
			bodyIndexA(data.bodyIndexA.data()),
			bodyIndexB(data.bodyIndexB.data()),
			localAnchorA(data.localAnchorA.data()),
			localAnchorB(data.localAnchorB.data()),
			localAxisA(data.localAxisA.data()),
			referenceAngle(data.referenceAngle.data()),
			stiffness(data.stiffness.data()),
			damping(data.damping.data()),
			lowerLimit(data.lowerLimit.data()),
			upperLimit(data.upperLimit.data()),
			enableLimit(data.enableLimit.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}