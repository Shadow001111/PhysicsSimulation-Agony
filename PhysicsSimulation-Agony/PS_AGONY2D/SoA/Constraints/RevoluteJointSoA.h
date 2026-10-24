#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/HandleTable.h"

// Define all scalar SoA fields.
#define PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(F) \
    F(bodyIndexA, ObjectDenseIndex) \
    F(bodyIndexB, ObjectDenseIndex) \
    F(localAnchorA, Vec2) \
    F(localAnchorB, Vec2) \
    F(stiffness, Real) \
    F(damping, Real)

namespace PS_AGONY2D
{
    // Revolute joint: pins localAnchorA (on bodyA) to localAnchorB (on bodyB) together,
    // leaving rotation free.
    struct RevoluteJointSoA
    {
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(DECLARE_FIELD)
        #undef DECLARE_FIELD

        HandleTable handles;

        ObjectDenseIndex append(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Vec2 anchorA, Vec2 anchorB, Real stiffness, Real damping);
        void swapWithBack(size_t index);
        void popBack();
        size_t swapRemove(size_t index);
        bool isValid(Handle h) const noexcept { return handles.isValid(h); }
        uint32_t indexOf(Handle h) const noexcept { return handles.indexOf(h); }
        uint32_t indexOfUnchecked(Handle h) const noexcept { return handles.indexOfUnchecked(h); }
        Handle handleAt(size_t index) const noexcept { return handles.handleAt(index); }


        size_t getCount() const noexcept { return bodyIndexA.size(); }
        size_t getMemoryUsage() const noexcept;
    };

    class RevoluteJointSoAViewer
    {
        size_t count = 0;
    public:
        #define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
        PS_AGONY_REVOLUTE_JOINT_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
        #undef DECLARE_VIEWER_FIELD

        RevoluteJointSoAViewer() = default;
        explicit RevoluteJointSoAViewer(const RevoluteJointSoA& data) noexcept;

        size_t getCount() const noexcept { return count; }
    };
}
