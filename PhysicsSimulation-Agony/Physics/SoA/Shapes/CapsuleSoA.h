#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

namespace PS_AGONY
{
	struct CapsuleSoA
	{
		SimdAlignedVector<ColliderIndex> colliderIndices;

		SimdAlignedVector<Real> halfLength;
		SimdAlignedVector<Real> radius;

		void append(
			ColliderIndex colliderIndex,
			Real halfLength,
			Real radius
		)
		{
			this->colliderIndices.push_back(colliderIndex);
			this->halfLength.push_back(halfLength);
			this->radius.push_back(radius);
		}

		size_t getCount() const noexcept { return colliderIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(colliderIndices) +
				PS_AGONY::getVectorMemoryUsage(halfLength) +
				PS_AGONY::getVectorMemoryUsage(radius);
		}
	};

	class CapsuleSoAViewer
	{
		size_t count = 0;
	public:
		const ColliderIndex* colliderIndices = nullptr;
		const Real* halfLength = nullptr;
		const Real* radius = nullptr;

		CapsuleSoAViewer() = default;

		explicit CapsuleSoAViewer(const CapsuleSoA& data) :
			count(data.getCount()),
			colliderIndices(data.colliderIndices.data()),
			halfLength(data.halfLength.data()),
			radius(data.radius.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}