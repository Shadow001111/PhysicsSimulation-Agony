#pragma once
#include "BvhNode.h"

#include "PS_AGONY2D/ObjectSoA.h"

namespace PS_AGONY2D::BroadPhaseInternal
{
	// Non-owning pointers into a BvhTree. Valid until the tree is rebuilt.
	struct BvhTreeView
	{
		const BvhNode* nodes = nullptr;
		const Real* leafMinX = nullptr; // Full leaf AABB SoA; node.leafIndex selects the slice.
		const Real* leafMaxX = nullptr;
		const Real* leafMinY = nullptr;
		const Real* leafMaxY = nullptr;
		const ObjectIndex* indices = nullptr; // Collider indices sorted by Morton code.
	};
}
