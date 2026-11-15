#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/GroupRanges.h"
#include "PS_AGONY2D/Types/Types.h"
#include <utility>

// Define all scalar SoA fields.
#define PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(F) \
	F(colliderIndices, ObjectDenseIndex) \
	F(radius, Real)

namespace PS_AGONY2D
{
	struct CircleSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(DECLARE_FIELD)
		#undef DECLARE_FIELD

		void append(
			ObjectDenseIndex colliderIndex,
			Real radius
		);

		// Dense order follows BodyGroup of the owning collider. Changed by ObjectManager group moves only.
		GroupRanges groups;
		IndexRange range(BodyGroup group) const noexcept { return groups.range(group, getCount()); }
		IndexRange liveRange() const noexcept { return groups.liveRange(); }

		// Swaps two entries in every field. Does not touch groups or collider back-references.
		void swap(ObjectDenseIndex a, ObjectDenseIndex b)
		{
			#define SWAP_FIELD_AB(name, type) std::swap(name[a], name[b]);
			PS_AGONY_CIRCLE_SOA_SCALAR_FIELDS(SWAP_FIELD_AB)
			#undef SWAP_FIELD_AB
		}

		void swapWithBack(ObjectDenseIndex index);
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