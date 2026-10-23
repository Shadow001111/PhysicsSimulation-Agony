#pragma once
#include "Physics/SoA/BodySoA.h"
#include "Physics/SoA/ColliderSoA.h"

namespace PS_AGONY2D
{
	void wrapBodyRotations(BodySoA& bodies);

	void wrapColliderRotations(ColliderSoA& colliders);
}

