#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/Types/Types.h"

// Define all scalar SoA fields.
#define PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(F) \
	F(colliderIndices, ColliderIndex) \
	F(radius, Real)

namespace PS_AGONY2D
{
	struct CircleSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(DECLARE_FIELD)
		#undef DECLARE_FIELD

		void append(
		ColliderIndex colliderIndex,
		Real radius
		);

		void swapWithBack(size_t index);
		void popBack();

		size_t getCount() const noexcept { return colliderIndices.size(); }
		size_t getMemoryUsage() const noexcept;
	};

	class CircleSoAViewer
	{
		size_t count = 0;
	public:
		#define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
		#undef DECLARE_VIEWER_FIELD

		CircleSoAViewer() = default;
		explicit CircleSoAViewer(const CircleSoA& data) noexcept;

		size_t getCount() const noexcept { return count; }
	};
}
