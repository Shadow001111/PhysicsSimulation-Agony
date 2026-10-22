#pragma once
#include "Physics/ContainerUtilities.h"
#include "Physics/Types/Types.h"

// Define all scalar SoA fields.
#define PS_AGONY_CAPSULE_SOA_SCALAR_FIELDS(F) \
	F(colliderIndices, ColliderIndex) \
	F(halfLength, Real) \
	F(radius, Real)

namespace PS_AGONY
{
	struct CapsuleSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_CAPSULE_SOA_SCALAR_FIELDS(DECLARE_FIELD)
		#undef DECLARE_FIELD

		void append(
		ColliderIndex colliderIndex,
		Real halfLength,
		Real radius
		);

		void swapWithBack(size_t index);
		void popBack();

		size_t getCount() const noexcept { return colliderIndices.size(); }
		size_t getMemoryUsage() const noexcept;
	};

	class CapsuleSoAViewer
	{
		size_t count = 0;
	public:
		#define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
		PS_AGONY_CAPSULE_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
		#undef DECLARE_VIEWER_FIELD

		CapsuleSoAViewer() = default;
		explicit CapsuleSoAViewer(const CapsuleSoA& data) noexcept;

		size_t getCount() const noexcept { return count; }
	};
}
