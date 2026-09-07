#pragma once
#include "Physics/ObjectSoA.h"

namespace PS_AGONY
{
	class ObjectManager
	{
	public:
		BodySoA bodies;
		ColliderSoA colliders;
		CircleSoA circles;
		BoxSoA boxes;
		PolygonSoA polygons;

		std::vector<ObjectDeletion> deletedBodies;
		std::vector<ObjectDeletion> deletedColliders;
	};
}

