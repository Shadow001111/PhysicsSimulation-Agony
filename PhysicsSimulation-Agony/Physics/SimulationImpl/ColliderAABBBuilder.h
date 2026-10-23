#pragma once
#include "Physics/SoA/ColliderSoA.h"

#include "Physics/SoA/Shapes/CircleSoA.h"
#include "Physics/SoA/Shapes/BoxSoA.h"
#include "Physics/SoA/Shapes/PolygonSoA.h"
#include "Physics/SoA/Shapes/CapsuleSoA.h"

namespace PS_AGONY2D
{
	void buildCircleAABBs (ColliderSoA& colliders, const CircleSoA& circles);
	void buildBoxAABBs	  (ColliderSoA& colliders, const BoxSoA& boxes);
	void buildPolygonAABBs(ColliderSoA& colliders, const PolygonSoA& polygons);
	void buildCapsuleAABBs(ColliderSoA& colliders, const CapsuleSoA& capsules);
}

