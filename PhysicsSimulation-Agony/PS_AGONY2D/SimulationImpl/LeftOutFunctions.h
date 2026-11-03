#pragma once
#include "PS_AGONY2D/SoA/BodySoA.h"
#include "PS_AGONY2D/SoA/ColliderSoA.h"

namespace PS_AGONY2D
{
	void wrapBodyRotations(BodySoA& bodies, IndexRange range);

	void wrapColliderRotations(ColliderSoA& colliders);
}

