#include "LeftOutFunctions.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include "PS_AGONY2D/Constants.h"

namespace PS_AGONY2D
{
    using RealSimd = Ecstasy::Core::Simd<Real>;
    constexpr size_t LANES = RealSimd::lanes;

    void wrapBodyRotations(BodySoA& bodies)
    {
        TRACY_SCOPE_NC("Wrap body rotations", PS_AGONY2D::Profiling::Color::Cyan);

        // Get pointers.
        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();
        Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount.data();

        // Precompute.
        const RealSimd oneV(1);
        const RealSimd twoPIV(Constants::TWO_PI);
        const RealSimd invTwoPIV(Real(1) / Constants::TWO_PI);

        // Wrap.
        const size_t bodyCount = bodies.getCount();
        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            RealSimd rot = RealSimd::load(rotationPtr + i);
            RealSimd oldWrapCount = RealSimd::load(rotationWrapCountPtr + i);

            RealSimd wrapCount = RealSimd::roundTowardsZero(rot * invTwoPIV);
            rot = RealSimd::negMulAdd(
                wrapCount,
                twoPIV,
                rot
            );

            RealSimd isRotNegativeMask = rot < RealSimd(0);
            rot += isRotNegativeMask & twoPIV;
            wrapCount -= isRotNegativeMask & oneV;

            rot.store(rotationPtr + i);
            (oldWrapCount + wrapCount).store(rotationWrapCountPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            Real rot = rotationPtr[i];
            Real wrapCount = std::trunc(rot * Constants::INV_TWO_PI);
            rot -= wrapCount * Constants::TWO_PI;

            Real isRotNegativeMask = rot < 0;
            rot += isRotNegativeMask * Constants::TWO_PI;
            wrapCount -= isRotNegativeMask;

            rotationPtr[i] = rot;
            rotationWrapCountPtr[i] += wrapCount;
        }
    }

    void wrapColliderRotations(ColliderSoA& colliders)
    {
        TRACY_SCOPE_NC("Wrap collider rotations", PS_AGONY2D::Profiling::Color::Cyan);

        // Get pointers.
        Real* ECSTASY_RESTRICT rotationPtr = colliders.worldRotation.data();

        // Precompute.
        const RealSimd twoPIV(Constants::TWO_PI);
        const RealSimd invTwoPIV(Real(1) / Constants::TWO_PI);

        // Wrap.
        const size_t colliderCount = colliders.getCount();
        size_t i = 0;
        for (; i + LANES <= colliderCount; i += LANES)
        {
            RealSimd rot = RealSimd::load(rotationPtr + i);

            RealSimd wrapCount = RealSimd::roundTowardsZero(rot * invTwoPIV);
            rot = RealSimd::negMulAdd(
                wrapCount,
                twoPIV,
                rot
            );

            RealSimd isRotNegativeMask = rot < RealSimd(0);
            rot += isRotNegativeMask & twoPIV;

            rot.store(rotationPtr + i);
        }
        for (; i < colliderCount; i++)
        {
            Real rot = rotationPtr[i];
            Real wrapCount = std::trunc(rot * Constants::INV_TWO_PI);
            rot -= wrapCount * Constants::TWO_PI;

            Real isRotNegativeMask = rot < 0;
            rot += isRotNegativeMask * Constants::TWO_PI;

            rotationPtr[i] = rot;
        }
    }
}