#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/HandleTable.h"
#include <cstdint>

// Define all scalar SoA fields.
#define PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(F) \
    F(bodyIndexA, ObjectDenseIndex) \
    F(bodyIndexB, ObjectDenseIndex) \
    F(localAnchorA, Vec2) \
    F(localAnchorB, Vec2) \
    F(localAxisA, Vec2) \
    F(referenceAngle, Real) \
    F(stiffness, Real) \
    F(damping, Real) \
    F(lowerLimit, Real) \
    F(upperLimit, Real) \
    F(enableLimit, uint8_t)

namespace PS_AGONY2D
{
    // A prismatic (slider / piston) joint. Locks relative translation between two
    // bodies to a single axis attached to body A (so the axis rotates with A), and
    // locks relative rotation, like a piston rod sliding through a cylinder.
    // Optionally limited to [lowerLimit, upperLimit] travel along the axis.
    struct PrismaticJointSoA
    {
        // localAxisA: unit axis in body A's local space; translation is locked to this axis.
        // referenceAngle: (bodyB.rotation - bodyA.rotation) captured at creation, wrapped to (-pi, pi].
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(DECLARE_FIELD)
        #undef DECLARE_FIELD

        HandleTable handles;

        ObjectDenseIndex append(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Vec2 anchorA, Vec2 anchorB, Vec2 axisA, Real referenceAngle, Real stiffness, Real damping, Real lowerLimit, Real upperLimit, bool enableLimit);
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

    class PrismaticJointSoAViewer
    {
        size_t count = 0;
    public:
        #define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
        PS_AGONY_PRISMATIC_JOINT_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
        #undef DECLARE_VIEWER_FIELD

        PrismaticJointSoAViewer() = default;
        explicit PrismaticJointSoAViewer(const PrismaticJointSoA& data) noexcept;

        size_t getCount() const noexcept { return count; }
    };
}
