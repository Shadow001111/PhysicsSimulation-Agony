#include "Integrator.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include "Physics/Constants.h"
#include "Physics/FastCosSin.h"

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;
    constexpr size_t LANES = RealSimd::lanes;

	void Integrator::integrateVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Integrate velocities", Ecstasy::Core::Color::Red);

        // Get pointers.
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

        // Precompute.
        const Vec2 gravityDelta = settings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };
        const RealSimd zeros = RealSimd(Real(0));

        // Integrate.
        const size_t bodyCount = bodies.getCount();
        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            RealSimd velX = RealSimd::load(velocityXPtr + i);
            RealSimd velY = RealSimd::load(velocityYPtr + i);

            const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            const auto movableMask = invMassV != zeros;

            RealSimd newVelX = velX + gravityDeltaXV;
            RealSimd newVelY = velY + gravityDeltaYV;

            velX = RealSimd::blendv(velX, newVelX, movableMask);
            velY = RealSimd::blendv(velY, newVelY, movableMask);

            velX.store(velocityXPtr + i);
            velY.store(velocityYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real invMass = invMassPtr[i];
            const Real movableMask = invMass != Real(0.0);

            velocityXPtr[i] += gravityDelta.x * movableMask;
            velocityYPtr[i] += gravityDelta.y * movableMask;
        }

        // Damp angular velocity.
        if (settings.angularVelocityDamping >= 0 && settings.angularVelocityDamping < 1)
        {
            Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();
            Real* ECSTASY_RESTRICT invInertiaPtr = bodies.invInertia.data();

            const Real damping = std::pow(settings.angularVelocityDamping, deltaTime);
            const RealSimd dampingV(damping);

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                RealSimd angVel = RealSimd::load(angularVelocityPtr + i);

                const RealSimd invInertiaV = RealSimd::load(invInertiaPtr + i);
                const auto movableMask = invInertiaV != zeros;

                RealSimd newAngVel = angVel * dampingV;

                newAngVel = RealSimd::blendv(angVel, newAngVel, movableMask);

                newAngVel.store(angularVelocityPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                const Real invInertia = invInertiaPtr[i];
                const Real movableMask = invInertia != Real(0.0);

                const Real angVel = angularVelocityPtr[i];

                Real newAngVel = angVel * damping;

                newAngVel = movableMask * newAngVel + (Real(1) - movableMask) * angVel;

                angularVelocityPtr[i] = newAngVel;
            }
        }
    }

    void Integrator::integrateKinematics(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Intergrate kinematics", Ecstasy::Core::Color::Blue);

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

        const Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        const Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

        // Precompute.
        const RealSimd deltaTimeV{ deltaTime };

        // Integrate.
        const size_t bodyCount = bodies.getCount();
        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            const RealSimd velX = RealSimd::load(velocityXPtr + i);
            const RealSimd velY = RealSimd::load(velocityYPtr + i);
            const RealSimd angVel = RealSimd::load(angularVelocityPtr + i);

            RealSimd posX = RealSimd::load(positionXPtr + i);
            RealSimd posY = RealSimd::load(positionYPtr + i);
            RealSimd rot = RealSimd::load(rotationPtr + i);

            posX = RealSimd::mulAdd(velX, deltaTimeV, posX);
            posY = RealSimd::mulAdd(velY, deltaTimeV, posY);
            rot = RealSimd::mulAdd(angVel, deltaTimeV, rot);

            posX.store(positionXPtr + i);
            posY.store(positionYPtr + i);
            rot.store(rotationPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            positionXPtr[i] += velocityXPtr[i] * deltaTime;
            positionYPtr[i] += velocityYPtr[i] * deltaTime;
            rotationPtr[i] += angularVelocityPtr[i] * deltaTime;
        }
    }

    void Integrator::wrapRotation(BodySoA& bodies)
    {
        TRACY_SCOPE_NC("Wrap rotation", Ecstasy::Core::Color::Cyan);

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
            rot = rot - wrapCount * twoPIV;

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
            wrapCount -= isRotNegativeMask; // * Real(1);

            rotationPtr[i] = rot;
            rotationWrapCountPtr[i] += wrapCount;
        }
    }

    void Integrator::computeRotationCosSin(BodySoA& bodies)
    {
        TRACY_SCOPE_NC("Compute rotation cos/sin", Ecstasy::Core::Color::Teal);

        FastCosSin::order4Array(
            bodies.rotation.data(),
            bodies.rotationCos.data(),
            bodies.rotationSin.data(),
            bodies.getCount()
        );
    }
}