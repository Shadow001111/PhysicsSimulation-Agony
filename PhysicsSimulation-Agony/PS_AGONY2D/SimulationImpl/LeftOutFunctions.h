#pragma once
#include "PS_AGONY2D/SoA/BodySoA.h"
#include "PS_AGONY2D/SoA/ColliderSoA.h"

namespace PS_AGONY2D
{
	void wrapBodyRotations(BodySoA& bodies);

	void wrapColliderRotations(ColliderSoA& colliders);
}

