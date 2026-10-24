#pragma once
#include "PS_AGONY2D/SoA/ColliderSoA.h"

#include "PS_AGONY2D/SoA/Shapes/CircleSoA.h"
#include "PS_AGONY2D/SoA/Shapes/BoxSoA.h"
#include "PS_AGONY2D/SoA/Shapes/PolygonSoA.h"
#include "PS_AGONY2D/SoA/Shapes/CapsuleSoA.h"

namespace PS_AGONY2D
{
	void buildCircleAABBs (ColliderSoA& colliders, const CircleSoA& circles);
	void buildBoxAABBs	  (ColliderSoA& colliders, const BoxSoA& boxes);
	void buildPolygonAABBs(ColliderSoA& colliders, const PolygonSoA& polygons);
	void buildCapsuleAABBs(ColliderSoA& colliders, const CapsuleSoA& capsules);
}

