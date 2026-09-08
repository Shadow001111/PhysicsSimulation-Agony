#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/ContainerUtilities.h"

#include <algorithm>

namespace PS_AGONY
{
	// Revolute joint: pins localAnchorA (on bodyA) to localAnchorB (on bodyB) together,
	// leaving rotation free. Unlike SpringSoA there's no rest length - the anchors are
	// meant to coincide.
	struct JointSoA
	{
		std::vector<ObjectIndex> bodyIndexA;
		std::vector<ObjectIndex> bodyIndexB;
		std::vector<Vec2> localAnchorA;
		std::vector<Vec2> localAnchorB;
		std::vector<Real> stiffness;
		std::vector<Real> damping;

		ObjectIndex append(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real stiffness, Real damping)
		{
			const ObjectIndex newIndex = this->bodyIndexA.size();

			this->bodyIndexA.push_back(indexA);
			this->bodyIndexB.push_back(indexB);
			this->localAnchorA.push_back(anchorA);
			this->localAnchorB.push_back(anchorB);
			this->stiffness.push_back(stiffness);
			this->damping.push_back(damping);

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
				PS_AGONY::getVectorMemoryUsage(stiffness) +
				PS_AGONY::getVectorMemoryUsage(damping);
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
				std::swap(stiffness[index], stiffness[lastIndex]);
				std::swap(damping[index], damping[lastIndex]);
			}

			bodyIndexA.pop_back();
			bodyIndexB.pop_back();
			localAnchorA.pop_back();
			localAnchorB.pop_back();
			stiffness.pop_back();
			damping.pop_back();

			return lastIndex;
		}
	};

	class JointSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndexA = nullptr;
		const ObjectIndex* bodyIndexB = nullptr;
		const Vec2* localAnchorA = nullptr;
		const Vec2* localAnchorB = nullptr;
		const Real* stiffness = nullptr;
		const Real* damping = nullptr;

		JointSoAViewer() = default;

		explicit JointSoAViewer(const JointSoA& data) noexcept :
			count(data.getCount()),
			bodyIndexA(data.bodyIndexA.data()),
			bodyIndexB(data.bodyIndexB.data()),
			localAnchorA(data.localAnchorA.data()),
			localAnchorB(data.localAnchorB.data()),
			stiffness(data.stiffness.data()),
			damping(data.damping.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}