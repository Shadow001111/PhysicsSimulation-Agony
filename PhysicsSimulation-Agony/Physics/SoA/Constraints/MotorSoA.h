#pragma once
#include "Physics/ContainerUtilities.h"
#include "Physics/Types/Types.h"

// Define all scalar SoA fields.
#define PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(F) \
    F(bodyIndex, ObjectIndex) \
    F(targetSpeed, Real) \
    F(maxTorque, Real)

// Fields written by the solver through the viewer.
#define PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(F) \
    F(accumulatedImpulse, Real)

namespace PS_AGONY
{
    // Single-body angular motor: drives angular velocity toward targetSpeed, torque limited by maxTorque.
    struct MotorSoA
    {
        // accumulatedImpulse is persistent (warm-starting) and written by the solver, so its viewer pointer is mutable.
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(DECLARE_FIELD)
        PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(DECLARE_FIELD)
        #undef DECLARE_FIELD

        ObjectIndex append(ObjectIndex index, Real speed, Real torqueLimit);
        size_t swapRemove(size_t index);

        size_t getCount() const noexcept { return bodyIndex.size(); }
        size_t getMemoryUsage() const noexcept;
    };

    class MotorSoAViewer
    {
        size_t count = 0;
    public:
        #define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
        PS_AGONY_MOTOR_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
        #undef DECLARE_VIEWER_FIELD

        #define DECLARE_VIEWER_MUTABLE_FIELD(name, type) mutable const type* name = nullptr;
        PS_AGONY_MOTOR_SOA_MUTABLE_FIELDS(DECLARE_VIEWER_MUTABLE_FIELD)
        #undef DECLARE_VIEWER_MUTABLE_FIELD

        MotorSoAViewer() = default;
        explicit MotorSoAViewer(const MotorSoA& data) noexcept;

        size_t getCount() const noexcept { return count; }
    };
}
