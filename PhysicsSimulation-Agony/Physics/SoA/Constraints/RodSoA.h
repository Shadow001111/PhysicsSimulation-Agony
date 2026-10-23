#pragma once
#include "Physics/ContainerUtilities.h"
#include "Physics/Types/Types.h"

// Define all scalar SoA fields.
#define PS_AGONY_ROD_SOA_SCALAR_FIELDS(F) \
    F(bodyIndexA, ObjectIndex) \
    F(bodyIndexB, ObjectIndex) \
    F(localAnchorA, Vec2) \
    F(localAnchorB, Vec2) \
    F(length, Real)

namespace PS_AGONY2D
{
    struct RodSoA
    {
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_ROD_SOA_SCALAR_FIELDS(DECLARE_FIELD)
        #undef DECLARE_FIELD

        ObjectIndex append(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real length);
        size_t swapRemove(size_t index);

        size_t getCount() const noexcept { return bodyIndexA.size(); }
        size_t getMemoryUsage() const noexcept;
    };

    class RodSoAViewer
    {
        size_t count = 0;
    public:
        #define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
        PS_AGONY_ROD_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
        #undef DECLARE_VIEWER_FIELD

        RodSoAViewer() = default;
        explicit RodSoAViewer(const RodSoA& data) noexcept;

        size_t getCount() const noexcept { return count; }
    };
}
