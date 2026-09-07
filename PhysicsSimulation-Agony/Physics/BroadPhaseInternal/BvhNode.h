#pragma once
#include "Ecstasy/Core/Simd.h"
#include "Physics/GlmTypes.h"

namespace PS_AGONY::BroadPhaseInternal
{
	// Note: Splitting on cold and hot didn't help.
	struct BvhNode
	{
		// Max KD_LEAF_SIZE is 32. Larger size will fuck up bitwise mask.
		// (We can change mask to me uint64_t to allow max KD_LEAF_SIZE to be 64, but increasing KD_LEAF_SIZE leads to perfomance decrease in narrow phase.)
		static constexpr uint32_t KD_LEAF_SIZE = 16;
		static_assert((KD_LEAF_SIZE% Ecstasy::Core::Simd<Real>::lanes) == 0, "KD_LEAF_SIZE must be multiple of Simd<Real>::lanes.");

		static constexpr uint32_t INVALID_INDEX = -1;

		// AABB data must stay first.
		Real minX, maxX, minY, maxY; // Merged AABB of all colliders in this subtree.
		uint32_t leftChildIndex = INVALID_INDEX; // INVALID_INDEX for leaves.
		// rightChildIndex = leftChildIndex + 1.
		uint32_t start, end; // Range in kdIndices: [start, end).
		uint32_t leafIndex; // If node is a leaf, it's its index.

		BvhNode() :
			start(0), end(0)
		{
		}

		BvhNode(uint32_t start, uint32_t end) :
			start(start), end(end)
		{
		}
	};
}