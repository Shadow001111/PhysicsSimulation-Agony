#pragma once
#include "BvhNode.h"
#include "BvhTreeView.h"
#include "Math.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <array>
#include <bit>
#include <cstdint>
#include <vector>

namespace PS_AGONY2D::BroadPhaseInternal
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	inline constexpr uint32_t upperTriMask(uint32_t i, uint32_t j_base, uint32_t LANES, uint32_t ALL_LANES_MASK)
	{
		if (i < j_base)          return ALL_LANES_MASK;
		if (i >= j_base + LANES) return 0u;
		const uint32_t offset = i - j_base; // 0 .. LANES-1.
		return (ALL_LANES_MASK << (offset + 1)) & ALL_LANES_MASK;
	}

	template<uint32_t W, uint32_t H>
	constexpr auto makeMaskArray()
	{
		constexpr uint32_t LANES = RealSimd::lanes;
		constexpr uint32_t ALL_LANES_MASK = (1u << LANES) - 1u;
		constexpr uint32_t LANES_LOG2 = ceilLog2(LANES);

		std::array<std::array<uint32_t, H>, W> arr{};
		for (uint32_t i = 0; i < W; i++)
		{
			for (uint32_t j_base = 0; j_base < H; j_base++)
			{
				arr[i][j_base] = upperTriMask(i, j_base << LANES_LOG2, LANES, ALL_LANES_MASK);
			}
		}
		return arr;
	}

	inline constexpr uint32_t LEAF_TEST_LANES = RealSimd::lanes;
	inline constexpr uint32_t LEAF_TEST_LANES_LOG2 = ceilLog2(LEAF_TEST_LANES);
	inline constexpr auto leafSelfCrossMaskArray = makeMaskArray<BvhNode::KD_LEAF_SIZE, BvhNode::KD_LEAF_SIZE / LEAF_TEST_LANES>();
	inline constexpr size_t PAIR_PUSH_BUFFER_CAPACITY = 256;

	// Batches ObjectPair writes and flushes them to 'dest' in bulk. One instance per thread.
	struct PairPushBuffer
	{
		ObjectPair buffer[PAIR_PUSH_BUFFER_CAPACITY];
		uint32_t size = 0;
		CacheLineAlignedVector<ObjectPair>& dest;

		explicit PairPushBuffer(CacheLineAlignedVector<ObjectPair>& destIn) noexcept : dest(destIn) {}

		inline void push(ObjectDenseIndex a, ObjectDenseIndex b) noexcept
		{
			buffer[size++] = { a, b };
			if (size == PAIR_PUSH_BUFFER_CAPACITY) flush();
		}
		inline void flush() noexcept
		{
			dest.insert(dest.end(), buffer, buffer + size);
			size = 0;
		}
	};

	// Tests all colliders within one leaf against each other.
	inline void testLeafSelfCross(
		const BvhNode& node,
		const BvhTreeView& tree,
		const ObjectDenseIndex* ECSTASY_RESTRICT colliderOwnerPtr,
		uint32_t* ECSTASY_RESTRICT masks,
		PairPushBuffer& pushBuffer)
	{
		const size_t srcIndex = node.leafIndex * BvhNode::KD_LEAF_SIZE;
		const Real* ECSTASY_RESTRICT leafMinX = tree.leafMinX + srcIndex;
		const Real* ECSTASY_RESTRICT leafMaxX = tree.leafMaxX + srcIndex;
		const Real* ECSTASY_RESTRICT leafMinY = tree.leafMinY + srcIndex;
		const Real* ECSTASY_RESTRICT leafMaxY = tree.leafMaxY + srcIndex;
		const ObjectDenseIndex* ECSTASY_RESTRICT indicesPtr = tree.indices;
		const uint32_t count = node.end - node.start;

		for (uint32_t i = 0; i < count; i++)
		{
			const RealSimd vMinXi(leafMinX[i]);
			const RealSimd vMaxXi(leafMaxX[i]);
			const RealSimd vMinYi(leafMinY[i]);
			const RealSimd vMaxYi(leafMaxY[i]);

			// Note: "jStart = i & LANES_UPPER_MASK" slows everything down.
			auto maskRow = leafSelfCrossMaskArray[i];
			uint32_t mask = 0;
			for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LEAF_TEST_LANES)
			{
				const RealSimd vMinXj = RealSimd::load(leafMinX + j);
				const RealSimd vMaxXj = RealSimd::load(leafMaxX + j);
				const RealSimd vMinYj = RealSimd::load(leafMinY + j);
				const RealSimd vMaxYj = RealSimd::load(leafMaxY + j);

				const RealSimd overlap =
					(vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
					(vMinYi < vMaxYj) & (vMaxYi > vMinYj);

				const uint32_t localMask = overlap.movemask() & maskRow[j >> LEAF_TEST_LANES_LOG2];
				mask |= localMask << j;
			}
			masks[i] = mask;
		}
		for (uint32_t i = 0; i < count; i++)
		{
			uint32_t mask = masks[i];
			while (mask)
			{
				const uint32_t lane = std::countr_zero(mask);
				mask &= mask - 1;
				const ObjectDenseIndex idxA = indicesPtr[node.start + i];
				const ObjectDenseIndex idxB = indicesPtr[node.start + lane];
				if (colliderOwnerPtr[idxA] == colliderOwnerPtr[idxB]) continue; // Same body.
				pushBuffer.push(idxA, idxB);
			}
		}
	}

	// Tests every collider in leaf A against every collider in leaf B.
	// CheckSameBody = true for leaves of one tree; false for leaves of different trees
	// (a body lives in one tree only), where 'colliderOwnerPtr' may be null.
	// Pairs are pushed as (A, B).
	template<bool CheckSameBody>
	inline void testLeafPairCross(
		const BvhNode& nodeA,
		const BvhTreeView& treeA,
		const BvhNode& nodeB,
		const BvhTreeView& treeB,
		[[maybe_unused]] const ObjectDenseIndex* ECSTASY_RESTRICT colliderOwnerPtr,
		uint32_t* ECSTASY_RESTRICT masks,
		PairPushBuffer& pushBuffer)
	{
		const uint32_t countA = nodeA.end - nodeA.start;

		const size_t srcIndexA = nodeA.leafIndex * BvhNode::KD_LEAF_SIZE;
		const Real* ECSTASY_RESTRICT leafAMinX = treeA.leafMinX + srcIndexA;
		const Real* ECSTASY_RESTRICT leafAMaxX = treeA.leafMaxX + srcIndexA;
		const Real* ECSTASY_RESTRICT leafAMinY = treeA.leafMinY + srcIndexA;
		const Real* ECSTASY_RESTRICT leafAMaxY = treeA.leafMaxY + srcIndexA;

		const size_t srcIndexB = nodeB.leafIndex * BvhNode::KD_LEAF_SIZE;
		const Real* ECSTASY_RESTRICT leafBMinX = treeB.leafMinX + srcIndexB;
		const Real* ECSTASY_RESTRICT leafBMaxX = treeB.leafMaxX + srcIndexB;
		const Real* ECSTASY_RESTRICT leafBMinY = treeB.leafMinY + srcIndexB;
		const Real* ECSTASY_RESTRICT leafBMaxY = treeB.leafMaxY + srcIndexB;

		for (uint32_t i = 0; i < countA; i++)
		{
			const RealSimd vMinXi(leafAMinX[i]);
			const RealSimd vMaxXi(leafAMaxX[i]);
			const RealSimd vMinYi(leafAMinY[i]);
			const RealSimd vMaxYi(leafAMaxY[i]);

			uint32_t mask = 0;
			for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LEAF_TEST_LANES)
			{
				const RealSimd vMinXj = RealSimd::load(leafBMinX + j);
				const RealSimd vMaxXj = RealSimd::load(leafBMaxX + j);
				const RealSimd vMinYj = RealSimd::load(leafBMinY + j);
				const RealSimd vMaxYj = RealSimd::load(leafBMaxY + j);

				const RealSimd overlap =
					(vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
					(vMinYi < vMaxYj) & (vMaxYi > vMinYj);

				mask |= overlap.movemask() << j;
			}
			masks[i] = mask;
		}
		for (uint32_t i = 0; i < countA; i++)
		{
			uint32_t mask = masks[i];
			while (mask)
			{
				const uint32_t lane = std::countr_zero(mask);
				mask &= mask - 1;
				const ObjectDenseIndex idxA = treeA.indices[nodeA.start + i];
				const ObjectDenseIndex idxB = treeB.indices[nodeB.start + lane];
				if constexpr (CheckSameBody)
				{
					if (colliderOwnerPtr[idxA] == colliderOwnerPtr[idxB]) continue;
				}
				pushBuffer.push(idxA, idxB);
			}
		}
	}
}
