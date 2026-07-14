#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

namespace PS_AGONY
{
	struct BoxSoA
	{
		SimdAlignedVector<ObjectIndex> bodyIndices;

		std::vector<Real> halfWidth;
		std::vector<Real> halfHeight;

		void append(
			ObjectIndex bodyIndex,
			Real halfWidth,
			Real halfHeight
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->halfWidth.push_back(halfWidth);
			this->halfHeight.push_back(halfHeight);
		}

		size_t getCount() const noexcept { return bodyIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndices) +
				PS_AGONY::getVectorMemoryUsage(halfWidth) +
				PS_AGONY::getVectorMemoryUsage(halfHeight);
		}
	};

	class BoxSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndices = nullptr;
		const Real* halfWidth = nullptr;
		const Real* halfHeight = nullptr;

		BoxSoAViewer() = default;

		explicit BoxSoAViewer(const BoxSoA& data) :
			count(data.getCount()),
			bodyIndices(data.bodyIndices.data()),
			halfWidth(data.halfWidth.data()),
			halfHeight(data.halfHeight.data())
		{}

		size_t getCount() const noexcept { return count; }
	};
}