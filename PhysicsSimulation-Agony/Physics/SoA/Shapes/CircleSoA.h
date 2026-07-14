#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

namespace PS_AGONY
{
	struct CircleSoA
	{
		SimdAlignedVector<BodyIndex> bodyIndices;

		SimdAlignedVector<Real> radius;

		void append(
			BodyIndex bodyIndex,
			Real radius
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->radius.push_back(radius);
		}

		size_t getCount() const noexcept { return bodyIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndices) +
				PS_AGONY::getVectorMemoryUsage(radius);
		}
	};

	class CircleSoAViewer
	{
		size_t count = 0;
	public:
		const BodyIndex* bodyIndices = nullptr;
		const Real* radius = nullptr;

		CircleSoAViewer() = default;

		explicit CircleSoAViewer(const CircleSoA& data) :
			count(data.getCount()),
			bodyIndices(data.bodyIndices.data()),
			radius(data.radius.data())
		{}

		size_t getCount() const noexcept { return count; }
	};
}