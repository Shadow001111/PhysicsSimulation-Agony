#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

#include <algorithm>

namespace PS_AGONY
{
	struct SpringSoA
	{
		std::vector<BodyIndex> bodyIndexA;
		std::vector<BodyIndex> bodyIndexB;
		std::vector<Vec2> localAnchorA;
		std::vector<Vec2> localAnchorB;
		std::vector<Real> restLength;
		std::vector<Real> stiffness;
		std::vector<Real> damping;

		void append(BodyIndex indexA, BodyIndex indexB, Vec2 anchorA, Vec2 anchorB, Real restLen, Real stifness, Real damping)
		{
			this->bodyIndexA.push_back(indexA);
			this->bodyIndexB.push_back(indexB);
			this->localAnchorA.push_back(anchorA);
			this->localAnchorB.push_back(anchorB);
			this->restLength.push_back(restLen);
			this->stiffness.push_back(stifness);
			this->damping.push_back(damping);
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

		void remapAfterDeletions(const std::vector<BodyDeletion>& deletedBodies)
		{
			for (const auto& deletion : deletedBodies)
			{
				auto [deletedIdx, swappedIdx] = deletion;

				size_t i = 0;
				while (i < bodyIndexA.size())
				{
					// If the spring is attached to the deleted body, destroy the constraint.
					if (bodyIndexA[i] == deletedIdx || bodyIndexB[i] == deletedIdx)
					{
						if (i != bodyIndexA.size() - 1)
						{
							std::swap(bodyIndexA[i], bodyIndexA.back());
							std::swap(bodyIndexB[i], bodyIndexB.back());
							std::swap(localAnchorA[i], localAnchorA.back());
							std::swap(localAnchorB[i], localAnchorB.back());
							std::swap(restLength[i], restLength.back());
							std::swap(stiffness[i], stiffness.back());
							std::swap(damping[i], damping.back());
						}
						bodyIndexA.pop_back();
						bodyIndexB.pop_back();
						localAnchorA.pop_back();
						localAnchorB.pop_back();
						restLength.pop_back();
						stiffness.pop_back();
						damping.pop_back();
						continue;
					}

					// If the spring is attached to the body that took the deleted body's place, update it.
					if (bodyIndexA[i] == swappedIdx) bodyIndexA[i] = deletedIdx;
					if (bodyIndexB[i] == swappedIdx) bodyIndexB[i] = deletedIdx;
					i++;
				}
			}
		}
	};

	class SpringSoAViewer
	{
		size_t count = 0;
	public:
		const BodyIndex* bodyIndexA = nullptr;
		const BodyIndex* bodyIndexB = nullptr;
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