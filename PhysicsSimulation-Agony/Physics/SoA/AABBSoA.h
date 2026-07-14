#pragma once
#include "../GlmTypes.h"
#include "../ContainerUtilities.h"

namespace PS_AGONY
{
	struct AABBSoA
	{
		SimdAlignedVector<Real> minX;
		SimdAlignedVector<Real> minY;
		SimdAlignedVector<Real> maxX;
		SimdAlignedVector<Real> maxY;

		size_t getCount() const noexcept { return minX.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(minX) +
				PS_AGONY::getVectorMemoryUsage(minY) +
				PS_AGONY::getVectorMemoryUsage(maxX) +
				PS_AGONY::getVectorMemoryUsage(maxY);
		}
	};

    class AABBSoAViewer
    {
        size_t count = 0;
    public:
        const Real* minX = nullptr;
        const Real* minY = nullptr;
        const Real* maxX = nullptr;
        const Real* maxY = nullptr;

        AABBSoAViewer() = default;

        explicit AABBSoAViewer(const AABBSoA& data) :
            count(data.minX.size()),
            minX(data.minX.data()),
            minY(data.minY.data()),
            maxX(data.maxX.data()),
            maxY(data.maxY.data())
        {}

        size_t getCount() const noexcept { return count; }
    };
}