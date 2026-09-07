#include "ObjectManager.h"

namespace PS_AGONY
{
    std::optional<ObjectIndex> ObjectManager::createBody(const BodyCreateParams& params)
    {
        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());

        const Real mass = std::fmax(Real(0), params.mass);
        const Vec2 centerOfMass = params.centerOfMass.value_or(Vec2(0));
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;

        // No shape attached yet, so there's nothing to derive inertia from: it starts
        // at zero (infinite resistance to rotation change is NOT implied - it just
        // means no angular response until a collider/inertia is set some other way).
        bodies.append(
            params.position, params.velocity, params.rotation, params.angularVelocity,
            mass, invMass, Real(0), Real(0), centerOfMass
        );

        return newBodyIndex;
    }
}