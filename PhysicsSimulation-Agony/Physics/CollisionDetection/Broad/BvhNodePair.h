#pragma once
#include "BvhNode.h"
#include "Math.h"

namespace PS_AGONY2D::BroadPhaseInternal
{
	// A pair of BVH node indices awaiting overlap testing/traversal.
	// Shared by the single-threaded and threaded broad-phase query paths.
	struct BvhNodePair
	{
		uint32_t a, b;

		BvhNodePair swap() const noexcept { return { b, a }; }
	};

	// Upper bound on traversal stack depth for a tree over up to UINT32_MAX
	// colliders with leaves of size BvhNode::KD_LEAF_SIZE.
	static constexpr uint64_t MAX_STACK_CAPACITY =
		2ull * (32ull + ceilLog2(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;
}