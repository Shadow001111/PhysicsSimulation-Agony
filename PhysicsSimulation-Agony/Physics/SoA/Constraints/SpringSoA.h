#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

#include <algorithm>

namespace PS_AGONY
{
	struct SpringSoA
	{
		std::vector<ObjectIndex> bodyIndexA;
		std::vector<ObjectIndex> bodyIndexB;
		std::vector<Vec2> localAnchorA;
		std::vector<Vec2> localAnchorB;
		std::vector<Real> restLength;
		std::vector<Real> stiffness;
		std::vector<Real> damping;

		ObjectIndex append(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real restLen, Real stifness, Real damping)
		{
			const ObjectIndex newIndex = this->bodyIndexA.size();

			this->bodyIndexA.push_back(indexA);
			this->bodyIndexB.push_back(indexB);
			this->localAnchorA.push_back(anchorA);
			this->localAnchorB.push_back(anchorB);
			this->restLength.push_back(restLen);
			this->stiffness.push_back(stifness);
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
				PS_AGONY::getVectorMemoryUsage(restLength) +
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
				std::swap(restLength[index], restLength[lastIndex]);
				std::swap(stiffness[index], stiffness[lastIndex]);
				std::swap(damping[index], damping[lastIndex]);
			}

			bodyIndexA.pop_back();
			bodyIndexB.pop_back();
			localAnchorA.pop_back();
			localAnchorB.pop_back();
			restLength.pop_back();
			stiffness.pop_back();
			damping.pop_back();

			return lastIndex;
		}
	};

	class SpringSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndexA = nullptr;
		const ObjectIndex* bodyIndexB = nullptr;
		const Vec2* localAnchorA = nullptr;
		const Vec2* localAnchorB = nullptr;
		const Real* restLength = nullptr;
		const Real* stiffness = nullptr;
		const Real* damping = nullptr;

		SpringSoAViewer() = default;

		explicit SpringSoAViewer(const SpringSoA& data) noexcept :
			count(data.getCount()),
			bodyIndexA(data.bodyIndexA.data()),
			bodyIndexB(data.bodyIndexB.data()),
			localAnchorA(data.localAnchorA.data()),
			localAnchorB(data.localAnchorB.data()),
			restLength(data.restLength.data()),
			stiffness(data.stiffness.data()),
			damping(data.damping.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}