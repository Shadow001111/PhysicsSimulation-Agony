#pragma once
#include "Types.h"

#include <cstdint>
#include <limits>

namespace PS_AGONY
{
	// 0 is reserved: "no collision callback".
	using CollisionCallbackIndex = uint16_t;

	// Collider default: never fires.
	inline constexpr Real NO_COLLISION_EVENT_THRESHOLD = std::numeric_limits<Real>::max();

	// One event per opted-in collider side, from that side's perspective.
	struct CollisionEvent
	{
		ColliderIndex self, other;
		ObjectIndex selfBody, otherBody;
		Vec2 point;            // Average of the manifold's contact points.
		Vec2 normal;           // Points from self towards other.
		Real impulse;          // Impact impulse: max over contacts of (approach speed * normal effective mass), pre-solve.
		CollisionCallbackIndex callbackId;
	};

	// Handlers run at the end of the physics step. Do NOT create/destroy bodies or colliders inside them
	// (see the TODO at the top of Simulation.cpp). Changing velocities etc. is fine.
	using CollisionCallback = void(*)(const CollisionEvent& event, void* userData);
}
