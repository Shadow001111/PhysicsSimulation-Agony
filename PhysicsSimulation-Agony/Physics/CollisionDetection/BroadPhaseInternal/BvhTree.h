#pragma once
#include "BvhNode.h"
#include "BvhNodePair.h"
#include "BvhTreeView.h"

#include "Physics/ObjectSoA.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <array>
#include <bit>
#include <cstdint>
#include <span>
#include <vector>

namespace PS_AGONY::BroadPhaseInternal
{
	enum class BvhTreeTypeFlag : uint8_t
	{
		None = 0,
		Dynamic = 1 << 0,
		Static = 1 << 1,
		All = Static | Dynamic
	};

	constexpr BvhTreeTypeFlag operator|(BvhTreeTypeFlag a, BvhTreeTypeFlag b) noexcept
	{
		return static_cast<BvhTreeTypeFlag>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
	}

	// Morton-ordered BVH over a subset of collider AABBs.
	class BvhTree
	{
	public:
		// Rebuilds the tree over 'ids' (collider indices into 'aabbs').
		void build(const AABBSoAViewer& aabbs, std::span<const ColliderIndex> ids);

		// Refits to the current AABBs. Membership must match the last build().
		void refit(const AABBSoAViewer& aabbs);

		void clear();

		bool empty() const noexcept { return nodes.empty(); }
		size_t colliderCount() const noexcept { return colliderIndices.size(); }
		size_t nodeCount() const noexcept { return nodes.size(); }

		BvhTreeView view() const noexcept;

		void getLeafNodeAABBs(std::vector<AABB>& outAABBs) const;

		template <typename ShapeQuery>
		void queryCollidersInShape(const ShapeQuery& shape, std::vector<ColliderIndex>& outColliders) const;

		template <typename ShapeQuery>
		void queryLeafNodeAABBsInShape(const ShapeQuery& shape, std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;

	private:
		struct Bounds
		{
			Real minX, maxX, minY, maxY;
		};

		struct PackedIndex
		{
			uint32_t key;
			uint32_t index; // Position in 'ids' passed to build().
		};

		struct LeafData
		{
			std::array<Real, BvhNode::KD_LEAF_SIZE> data;
		};

		struct LeafAABBSoA
		{
			CacheLineAlignedVector<LeafData> minX;
			CacheLineAlignedVector<LeafData> maxX;
			CacheLineAlignedVector<LeafData> minY;
			CacheLineAlignedVector<LeafData> maxY;
		};

		static constexpr uint64_t MAX_STACK_CAPACITY = BroadPhaseInternal::MAX_STACK_CAPACITY;

		Bounds gatherCentroids(const AABBSoAViewer& aabbs, std::span<const ColliderIndex> ids);
		void transformCentroids(uint32_t count, const Bounds& bounds);
		void computeMortonCodes(uint32_t count);
		void sortByMortonCodes(std::span<const ColliderIndex> ids);
		uint32_t buildNodes(uint32_t count); // Returns leaf count.
		void fitNodes(const AABBSoAViewer& aabbs, bool isRebuild);

		std::vector<BvhNode> nodes;
		std::vector<ObjectIndex> colliderIndices; // Collider indices, sorted by Morton code.
		LeafAABBSoA leafAABBs;

		// Build scratch.
		CacheLineAlignedVector<Real> centroidX;
		CacheLineAlignedVector<Real> centroidY;
		CacheLineAlignedVector<uint32_t> mortonCodes;
		CacheLineAlignedVector<PackedIndex> packedA;
		CacheLineAlignedVector<PackedIndex> packedB;
	};

	template <typename ShapeQuery>
	inline void BvhTree::queryCollidersInShape(const ShapeQuery& shape, std::vector<ColliderIndex>& outColliders) const
	{
		using RealSimd = Ecstasy::Core::Simd<Real>;

		if (nodes.empty()) return;

		// Quick escape if root node does not overlap shape.
		if (!shape.overlaps(nodes[0])) return;

		const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<const Real*>(leafAABBs.minX.data());
		const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<const Real*>(leafAABBs.maxX.data());
		const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<const Real*>(leafAABBs.minY.data());
		const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<const Real*>(leafAABBs.maxY.data());
		const BvhNode* ECSTASY_RESTRICT nodePtr = nodes.data();
		const ObjectIndex* ECSTASY_RESTRICT indicesPtr = colliderIndices.data();

		uint32_t stack[MAX_STACK_CAPACITY];
		uint32_t stackSize = 0;

		stack[stackSize++] = 0;

		while (stackSize > 0)
		{
			const uint32_t nodeIdx = stack[--stackSize];
			const BvhNode& node = nodePtr[nodeIdx];

			// Fully contained node: bulk-insert all of its colliders.
			if (shape.contains(node))
			{
				outColliders.insert(outColliders.end(), indicesPtr + node.start, indicesPtr + node.end);
				continue;
			}

			if (node.leftChildIndex == BvhNode::INVALID_INDEX) // Leaf node.
			{
				const uint32_t count = node.end - node.start;

				const size_t srcIndex = node.leafIndex * BvhNode::KD_LEAF_SIZE;
				const Real* leafMinX = leafMinXPtr + srcIndex;
				const Real* leafMaxX = leafMaxXPtr + srcIndex;
				const Real* leafMinY = leafMinYPtr + srcIndex;
				const Real* leafMaxY = leafMaxYPtr + srcIndex;

				uint32_t mask = 0;
				for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += RealSimd::lanes)
				{
					const RealSimd minXV = RealSimd::load(leafMinX + j);
					const RealSimd maxXV = RealSimd::load(leafMaxX + j);
					const RealSimd minYV = RealSimd::load(leafMinY + j);
					const RealSimd maxYV = RealSimd::load(leafMaxY + j);

					const auto overlap = shape.simdOverlap(minXV, maxXV, minYV, maxYV);

					mask |= overlap.movemask() << j;
				}

				// Mask out padded elements past count.
				mask &= static_cast<uint32_t>((1ULL << count) - 1ULL);

				while (mask)
				{
					const uint32_t lane = std::countr_zero(mask);
					mask &= mask - 1;
					outColliders.push_back(indicesPtr[node.start + lane]);
				}
			}
			else // Internal node.
			{
				const BvhNode& left = nodePtr[node.leftChildIndex];
				const BvhNode& right = nodePtr[node.leftChildIndex + 1];

				if (shape.overlaps(right))
				{
					stack[stackSize++] = node.leftChildIndex + 1;
				}
				if (shape.overlaps(left))
				{
					stack[stackSize++] = node.leftChildIndex;
				}
			}
		}
	}

	template <typename ShapeQuery>
	inline void BvhTree::queryLeafNodeAABBsInShape(const ShapeQuery& shape, std::vector<AABB>& outAABBs) const
	{
		if (nodes.empty()) return;

		if (!shape.overlaps(nodes[0])) return;

		const BvhNode* ECSTASY_RESTRICT nodePtr = nodes.data();

		uint32_t stack[MAX_STACK_CAPACITY];
		uint32_t stackSize = 0;

		stack[stackSize++] = 0;

		while (stackSize > 0)
		{
			const uint32_t nodeIdx = stack[--stackSize];
			const BvhNode& node = nodePtr[nodeIdx];

			if (node.leftChildIndex == BvhNode::INVALID_INDEX) // Leaf node.
			{
				outAABBs.push_back(AABB{ node.minX, node.minY, node.maxX, node.maxY });
			}
			else // Internal node.
			{
				const BvhNode& left = nodePtr[node.leftChildIndex];
				const BvhNode& right = nodePtr[node.leftChildIndex + 1];

				if (shape.overlaps(right))
				{
					stack[stackSize++] = node.leftChildIndex + 1;
				}
				if (shape.overlaps(left))
				{
					stack[stackSize++] = node.leftChildIndex;
				}
			}
		}
	}
}
