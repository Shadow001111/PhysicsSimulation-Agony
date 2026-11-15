#pragma once
#include "PS_AGONY2D/Types/Types.h"

#include <cstddef>
#include <cstdint>

namespace PS_AGONY2D
{
    // Ordered in ascending order by change frequency to reduce swap count.
    // Keeping sleeping and dynamic bodies together because constraints still act on sleeping bodies, so solving planner can claim single big range.
    // TODO: Remove above comment if it will get fixed.
    // Used by bodies, colliders and shape SoAs (a collider and its shape always share the owner body's group).
    enum class BodyGroup : uint8_t { Static, Sleeping, Dynamic, Unused };

    static_assert(
        size_t(BodyGroup::Sleeping) + 1 == size_t(BodyGroup::Dynamic) ||
        size_t(BodyGroup::Dynamic) + 1 == size_t(BodyGroup::Sleeping),
        "Sleeping and Dynamic groups must be adjacent."
        );

    inline constexpr bool SLEEPING_AFTER_DYNAMIC = BodyGroup::Sleeping > BodyGroup::Dynamic;

    // Group boundaries of a dense array. starts[g] .. starts[g + 1] is group g; Unused is starts[Unused] .. count.
    // Changed by ObjectManager group moves only.
    struct GroupRanges
    {
        uint32_t starts[size_t(BodyGroup::Unused) + 1]{};

        IndexRange range(BodyGroup group, size_t count) const noexcept
        {
            const size_t index = static_cast<size_t>(group);
            if (index < size_t(BodyGroup::Unused)) [[likely]]
            {
                return { starts[index], starts[index + 1] };
            }
            return { starts[size_t(BodyGroup::Unused)], static_cast<ObjectDenseIndex>(count) };
        }

        IndexRange liveRange() const noexcept
        {
            return { 0, starts[size_t(BodyGroup::Unused)] };
        }

        BodyGroup groupOf(ObjectDenseIndex index) const noexcept
        {
            for (size_t i = 0; i < size_t(BodyGroup::Unused); i++)
            {
                if (index < starts[i + 1]) return static_cast<BodyGroup>(i);
            }
            return BodyGroup::Unused;
        }
    };
}
