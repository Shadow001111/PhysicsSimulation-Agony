#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

namespace PS_AGONY
{
	struct CircleSoA
	{
		SimdAlignedVector<ColliderIndex> colliderIndices;

		SimdAlignedVector<Real> radius;

		void append(
			ColliderIndex colliderIndex,
			Real radius
		)
		{
			this->colliderIndices.push_back(colliderIndex);
			this->radius.push_back(radius);
		}

		size_t getCount() const noexcept { return colliderIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(colliderIndices) +
				PS_AGONY::getVectorMemoryUsage(radius);
		}
	};

	class CircleSoAViewer
	{
		size_t count = 0;
	public:
		const ColliderIndex* colliderIndices = nullptr;
		const Real* radius = nullptr;

		CircleSoAViewer() = default;

		explicit CircleSoAViewer(const CircleSoA& data) :
			count(data.getCount()),
			colliderIndices(data.colliderIndices.data()),
			radius(data.radius.data())
		{}

		size_t getCount() const noexcept { return count; }
	};
}