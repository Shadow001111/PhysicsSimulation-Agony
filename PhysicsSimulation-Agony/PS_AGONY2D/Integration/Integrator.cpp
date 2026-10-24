#include "Integrator.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include "PS_AGONY2D/FastCosSin.h"

namespace PS_AGONY2D::Integrator
{
    using RealSimd = Ecstasy::Core::Simd<Real>;
    constexpr size_t LANES = RealSimd::lanes;

    void integrateVelocities_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Integrate velocities (semi-implicit)", PS_AGONY2D::Profiling::Color::Red);

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

            velX += gravityDeltaXV & movableMask;
            velY += gravityDeltaYV & movableMask;

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
    }

    void integrateKinematics_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Integrate kinematics (semi-implicit)", PS_AGONY2D::Profiling::Color::Blue);

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

    void integratePhase1_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Integrate phase1 (velocity-verlet)", PS_AGONY2D::Profiling::Color::Red);

        // Get pointers.
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

        // Precompute.
        const Real halfDT = deltaTime * Real(0.5);
        const Vec2 gravityDelta = settings.gravity * halfDT;
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

            velX += gravityDeltaXV & movableMask;
            velY += gravityDeltaYV & movableMask;

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
    }

    void integratePhase2_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        TRACY_SCOPE_NC("Integrate phase2 (velocity-verlet)", PS_AGONY2D::Profiling::Color::Red);

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

        // Precompute.
        const Real halfDT = deltaTime * Real(0.5);
        const Vec2 gravityDelta = settings.gravity * halfDT;
        const RealSimd deltaTimeV{ deltaTime };
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };
        const RealSimd zeros = RealSimd(Real(0));

        // Integrate.
        const size_t bodyCount = bodies.getCount();
        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            // Full position step.
            RealSimd velX = RealSimd::load(velocityXPtr + i);
            RealSimd velY = RealSimd::load(velocityYPtr + i);
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

            // Half velocity step.
            const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            const auto movableMask = invMassV != zeros;

            velX += gravityDeltaXV & movableMask;
            velY += gravityDeltaYV & movableMask;

            velX.store(velocityXPtr + i);
            velY.store(velocityYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            // Full position step.
            positionXPtr[i] += velocityXPtr[i] * deltaTime;
            positionYPtr[i] += velocityYPtr[i] * deltaTime;
            rotationPtr[i] += angularVelocityPtr[i] * deltaTime;

            // Half velocity step.
            const Real invMass = invMassPtr[i];
            const Real movableMask = invMass != Real(0.0);

            velocityXPtr[i] += gravityDelta.x * movableMask;
            velocityYPtr[i] += gravityDelta.y * movableMask;
        }
    }

    // TODO: Remove functions below from this file.
    void dampAngularVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings)
    {
        if (!(settings.angularVelocityDamping >= 0 && settings.angularVelocityDamping < 1))
        {
            return;
        }

        TRACY_SCOPE_NC("Damp angular velocities", PS_AGONY2D::Profiling::Color::Red);

        // Get pointers.
        Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();
        Real* ECSTASY_RESTRICT invInertiaPtr = bodies.invInertia.data();

        // Precompute.
        const RealSimd zeros = RealSimd(Real(0));

        const Real damping = std::pow(settings.angularVelocityDamping, deltaTime);
        const RealSimd dampingV(damping);

        const size_t bodyCount = bodies.getCount();
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

    void computeRotationCosSin(BodySoA& bodies)
    {
        TRACY_SCOPE_NC("Compute rotation cos/sin", PS_AGONY2D::Profiling::Color::Teal);

        FastCosSin::order5Array(
            bodies.rotation.data(),
            bodies.rotationCos.data(),
            bodies.rotationSin.data(),
            bodies.getCount()
        );
    }
}