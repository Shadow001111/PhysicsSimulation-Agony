#pragma once
#include "PS_AGONY2D/SoA/ColliderSoA.h"

#include "PS_AGONY2D/SoA/Shapes/CircleSoA.h"
#include "PS_AGONY2D/SoA/Shapes/BoxSoA.h"
#include "PS_AGONY2D/SoA/Shapes/PolygonSoA.h"
#include "PS_AGONY2D/SoA/Shapes/CapsuleSoA.h"

namespace PS_AGONY2D
{
	void buildCircleAABBs (ColliderSoA& colliders, const CircleSoA& circles	 , IndexRange shapeRange);
	void buildBoxAABBs	  (ColliderSoA& colliders, const BoxSoA& boxes		 , IndexRange shapeRange);
	void buildPolygonAABBs(ColliderSoA& colliders, const PolygonSoA& polygons, IndexRange shapeRange);
	void buildCapsuleAABBs(ColliderSoA& colliders, const CapsuleSoA& capsules, IndexRange shapeRange);
}

