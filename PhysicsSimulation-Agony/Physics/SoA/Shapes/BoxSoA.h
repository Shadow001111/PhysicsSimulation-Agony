#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

namespace PS_AGONY
{
	struct BoxSoA
	{
		SimdAlignedVector<ColliderIndex> colliderIndices;

		std::vector<Real> halfWidth;
		std::vector<Real> halfHeight;

		void append(
			ColliderIndex colliderIndex,
			Real halfWidth,
			Real halfHeight
		)
		{
			this->colliderIndices.push_back(colliderIndex);
			this->halfWidth.push_back(halfWidth);
			this->halfHeight.push_back(halfHeight);
		}

		size_t getCount() const noexcept { return colliderIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(colliderIndices) +
				PS_AGONY::getVectorMemoryUsage(halfWidth) +
				PS_AGONY::getVectorMemoryUsage(halfHeight);
		}
	};

	class BoxSoAViewer
	{
		size_t count = 0;
	public:
		const ColliderIndex* colliderIndices = nullptr;
		const Real* halfWidth = nullptr;
		const Real* halfHeight = nullptr;

		BoxSoAViewer() = default;

		explicit BoxSoAViewer(const BoxSoA& data) :
			count(data.getCount()),
			colliderIndices(data.colliderIndices.data()),
			halfWidth(data.halfWidth.data()),
			halfHeight(data.halfHeight.data())
		{}

		size_t getCount() const noexcept { return count; }
	};
}