#include "BvhTree.h"

#include "Ecstasy/Core/TracyProfiler.h"

#include <algorithm>
#include <limits>
#include <type_traits>

namespace PS_AGONY::BroadPhaseInternal
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	namespace
	{
		// Used by transformCentroids and fitNodes.
		constexpr uint32_t LANES = RealSimd::lanes;

		using MortonU32Simd = std::conditional_t<
			std::is_same_v<Real, float>,
			Ecstasy::Core::Simd<uint32_t>,
			Ecstasy::Core::Simd<uint32_t, 128> // Double.
		>;

		using MortonI32Simd = std::conditional_t<
			std::is_same_v<Real, float>,
			Ecstasy::Core::Simd<int32_t>,
			Ecstasy::Core::Simd<int32_t, 128> // Double.
		>;

		inline uint32_t part1By1(uint32_t x)
		{
			x &= 0x0000ffffu;
			x = (x | (x << 8)) & 0x00FF00FFu;
			x = (x | (x << 4)) & 0x0F0F0F0Fu;
			x = (x | (x << 2)) & 0x33333333u;
			x = (x | (x << 1)) & 0x55555555u;
			return x;
		}

		inline uint32_t morton2D(uint32_t x, uint32_t y)
		{
			return (part1By1(y) << 1) | part1By1(x);
		}

		inline MortonU32Simd part1By1Simd(MortonU32Simd x)
		{
			x &= MortonU32Simd(0x0000ffffu);
			x = (x | (x << 8)) & MortonU32Simd(0x00FF00FFu);
			x = (x | (x << 4)) & MortonU32Simd(0x0F0F0F0Fu);
			x = (x | (x << 2)) & MortonU32Simd(0x33333333u);
			x = (x | (x << 1)) & MortonU32Simd(0x55555555u);
			return x;
		}

		inline MortonU32Simd morton2DSimd(const MortonU32Simd& x, const MortonU32Simd& y)
		{
			return (part1By1Simd(y) << 1) | part1By1Simd(x);
		}
	}

	void BvhTree::clear()
	{
		nodes.clear();
		colliderIndices.clear();
	}

	BvhTreeView BvhTree::view() const noexcept
	{
		return {
			nodes.data(),
			reinterpret_cast<const Real*>(leafAABBs.minX.data()),
			reinterpret_cast<const Real*>(leafAABBs.maxX.data()),
			reinterpret_cast<const Real*>(leafAABBs.minY.data()),
			reinterpret_cast<const Real*>(leafAABBs.maxY.data()),
			colliderIndices.data()
		};
	}

	void BvhTree::build(const AABBSoAViewer& aabbs, std::span<const ColliderIndex> ids)
	{
		TRACY_SCOPE_N("Build tree");

		if (ids.empty())
		{
			clear();
			return;
		}

		const uint32_t count = static_cast<uint32_t>(ids.size());

		nodes.clear();
		nodes.reserve(count / 4 + 1);
		colliderIndices.resize(count);

		const Bounds bounds = gatherCentroids(aabbs, ids);
		transformCentroids(count, bounds);
		computeMortonCodes(count);
		sortByMortonCodes(ids);

		const uint32_t leafCount = buildNodes(count); // Handles count == 1 (single leaf).

		leafAABBs.minX.resize(leafCount);
		leafAABBs.maxX.resize(leafCount);
		leafAABBs.minY.resize(leafCount);
		leafAABBs.maxY.resize(leafCount);
		fitNodes(aabbs, true);
	}

	void BvhTree::refit(const AABBSoAViewer& aabbs)
	{
		if (nodes.empty()) return;

		fitNodes(aabbs, false);
	}

	void BvhTree::getLeafNodeAABBs(std::vector<AABB>& outAABBs) const
	{
		outAABBs.reserve(outAABBs.size() + nodes.size());
		for (const auto& node : nodes)
		{
			if (node.leftChildIndex == BvhNode::INVALID_INDEX)
			{
				outAABBs.emplace_back(node.minX, node.minY, node.maxX, node.maxY);
			}
		}
	}

	size_t BvhTree::getMemoryUsage() const
	{
		size_t total = 0;

		total += getVectorMemoryUsage(nodes);
		total += getVectorMemoryUsage(colliderIndices);

		total += getVectorMemoryUsage(leafAABBs.minX);
		total += getVectorMemoryUsage(leafAABBs.maxX);
		total += getVectorMemoryUsage(leafAABBs.minY);
		total += getVectorMemoryUsage(leafAABBs.maxY);

		total += getVectorMemoryUsage(centroidX);
		total += getVectorMemoryUsage(centroidY);
		total += getVectorMemoryUsage(mortonCodes);
		total += getVectorMemoryUsage(packedA);
		total += getVectorMemoryUsage(packedB);

		return total;
	}

	// Gathers 2*centroid (min + max) through 'ids' into contiguous arrays and returns the
	// bounds of the gathered AABBs. This is the only indirected pass of the build.
	BvhTree::Bounds BvhTree::gatherCentroids(const AABBSoAViewer& aabbs, std::span<const ColliderIndex> ids)
	{
		TRACY_SCOPE_N("Gather centroids");

		const size_t count = ids.size();
		centroidX.resize(count);
		centroidY.resize(count);

		const Real* ECSTASY_RESTRICT minXPtr = aabbs.minX;
		const Real* ECSTASY_RESTRICT maxXPtr = aabbs.maxX;
		const Real* ECSTASY_RESTRICT minYPtr = aabbs.minY;
		const Real* ECSTASY_RESTRICT maxYPtr = aabbs.maxY;
		const ColliderIndex* ECSTASY_RESTRICT idsPtr = ids.data();

		Real* ECSTASY_RESTRICT sumXPtr = centroidX.data();
		Real* ECSTASY_RESTRICT sumYPtr = centroidY.data();

		Bounds b{
			std::numeric_limits<Real>::max(), -std::numeric_limits<Real>::max(),
			std::numeric_limits<Real>::max(), -std::numeric_limits<Real>::max() };

		for (size_t i = 0; i < count; i++)
		{
			const ColliderIndex id = idsPtr[i];
			const Real minX = minXPtr[id];
			const Real maxX = maxXPtr[id];
			const Real minY = minYPtr[id];
			const Real maxY = maxYPtr[id];

			sumXPtr[i] = minX + maxX;
			sumYPtr[i] = minY + maxY;

			b.minX = std::fmin(b.minX, minX);
			b.maxX = std::fmax(b.maxX, maxX);
			b.minY = std::fmin(b.minY, minY);
			b.maxY = std::fmax(b.maxY, maxY);
		}
		return b;
	}

	// In place: 2*centroid -> centroid scaled to [0, 0xFFFF].
	void BvhTree::transformCentroids(uint32_t count, const Bounds& bounds)
	{
		TRACY_SCOPE_N("Transform centroids");

		// Prevent division by zero for degenerate scenes.
		constexpr Real kEps = Real(1e-5);
		const Real maxX = std::fmax(bounds.maxX, bounds.minX + kEps);
		const Real maxY = std::fmax(bounds.maxY, bounds.minY + kEps);

		constexpr Real clampMax = Real(0xFFFFu);
		const Real scaleX = clampMax / (maxX - bounds.minX);
		const Real scaleY = clampMax / (maxY - bounds.minY);

		const RealSimd scaledMinXV(bounds.minX * scaleX);
		const RealSimd scaledMinYV(bounds.minY * scaleY);
		const RealSimd halfScaleXV(scaleX * Real(0.5));
		const RealSimd halfScaleYV(scaleY * Real(0.5));
		const RealSimd clampMaxV(clampMax);

		Real* ECSTASY_RESTRICT cxPtr = centroidX.data();
		Real* ECSTASY_RESTRICT cyPtr = centroidY.data();

		size_t i = 0;
		for (; i + LANES <= count; i += LANES)
		{
			const RealSimd sx = RealSimd::load(cxPtr + i);
			const RealSimd sy = RealSimd::load(cyPtr + i);

			const RealSimd tx = RealSimd::mulSub(sx, halfScaleXV, scaledMinXV);
			const RealSimd ty = RealSimd::mulSub(sy, halfScaleYV, scaledMinYV);

			RealSimd::clamp(tx, RealSimd(0), clampMaxV).store(cxPtr + i);
			RealSimd::clamp(ty, RealSimd(0), clampMaxV).store(cyPtr + i);
		}
		for (; i < count; i++)
		{
			const Real tx = (cxPtr[i] * Real(0.5) - bounds.minX) * scaleX;
			const Real ty = (cyPtr[i] * Real(0.5) - bounds.minY) * scaleY;

			cxPtr[i] = std::clamp(tx, Real(0), clampMax);
			cyPtr[i] = std::clamp(ty, Real(0), clampMax);
		}
	}

	void BvhTree::computeMortonCodes(uint32_t count)
	{
		mortonCodes.resize(count);

		uint32_t* ECSTASY_RESTRICT mortonCodePtr = mortonCodes.data();
		const Real* ECSTASY_RESTRICT cxPtr = centroidX.data();
		const Real* ECSTASY_RESTRICT cyPtr = centroidY.data();

		TRACY_SCOPE_N("Compute morton codes");

		size_t i = 0;

		// SIMD loop. Runs only when lane counts match.
		if constexpr (MortonU32Simd::lanes == RealSimd::lanes)
		{
			for (; i + RealSimd::lanes <= count; i += RealSimd::lanes)
			{
				// Centroids are in range [0; 2^16].
				const MortonU32Simd qx = RealSimd::load(cxPtr + i).to<MortonI32Simd>().as<MortonU32Simd>();
				const MortonU32Simd qy = RealSimd::load(cyPtr + i).to<MortonI32Simd>().as<MortonU32Simd>();

				morton2DSimd(qx, qy).store(mortonCodePtr + i);
			}
		}

		for (; i < count; i++)
		{
			mortonCodePtr[i] = morton2D(static_cast<uint32_t>(cxPtr[i]), static_cast<uint32_t>(cyPtr[i]));
		}
	}

	void BvhTree::sortByMortonCodes(std::span<const ColliderIndex> ids)
	{
		TRACY_SCOPE_N("Sort indices by morton codes");

		constexpr uint32_t RADIX_BITS = 8;
		constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
		constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

		const uint32_t count = static_cast<uint32_t>(ids.size());

		uint32_t* ECSTASY_RESTRICT mortonCodePtr = mortonCodes.data();

		packedA.resize(count);
		packedB.resize(count);

		PackedIndex* ECSTASY_RESTRICT a = packedA.data();
		PackedIndex* ECSTASY_RESTRICT b = packedB.data();

		for (uint32_t i = 0; i < count; i++)
		{
			a[i].key = mortonCodePtr[i];
			a[i].index = i;
		}

		alignas(64) uint32_t counts[4][RADIX_SIZE] = {};

		// Note: Combining packing pass with counting pass worsens performance.
		{
			TRACY_SCOPE_N("Counting pass");
			for (uint32_t i = 0; i < count; i++)
			{
				const uint32_t k = a[i].key;
				counts[0][k & 0xFF]++;
				counts[1][(k >> 8) & 0xFF]++;
				counts[2][(k >> 16) & 0xFF]++;
				counts[3][(k >> 24) & 0xFF]++;
			}
		}

		auto radixPass = [&](
			uint32_t shift,
			const PackedIndex* ECSTASY_RESTRICT src,
			PackedIndex* ECSTASY_RESTRICT dst,
			uint32_t* ECSTASY_RESTRICT countSegment)
			{
				// Exclusive prefix sum.
				uint32_t sum = 0;
				for (uint32_t i = 0; i < RADIX_SIZE; i++)
				{
					const uint32_t c = countSegment[i];
					countSegment[i] = sum;
					sum += c;
				}

				// Stable scatter.
				for (uint32_t i = 0; i < count; i++)
				{
					const PackedIndex item = src[i];
					dst[countSegment[(item.key >> shift) & RADIX_MASK]++] = item;
				}
			};

		{
			TRACY_SCOPE_N("Sort");

			radixPass(0, a, b, counts[0]);
			radixPass(8, b, a, counts[1]);
			radixPass(16, a, b, counts[2]);
			radixPass(24, b, a, counts[3]);
		}

		// Write sorted collider indices and keys back.
		ObjectIndex* ECSTASY_RESTRICT outIndexPtr = colliderIndices.data();
		const ColliderIndex* ECSTASY_RESTRICT idsPtr = ids.data();
		for (uint32_t i = 0; i < count; i++)
		{
			const PackedIndex item = a[i];
			outIndexPtr[i] = idsPtr[item.index];
			mortonCodePtr[i] = item.key;
		}
	}

	// Top-down split on the highest differing Morton bit of the node's first and last code.
	// Equal codes fall back to a median split.
	uint32_t BvhTree::buildNodes(uint32_t count)
	{
		TRACY_SCOPE_N("Build nodes");

		const uint32_t* ECSTASY_RESTRICT mortonCodePtr = mortonCodes.data();

		uint32_t leafCount = 0;

		uint32_t stack[MAX_STACK_CAPACITY];
		uint32_t stackSize = 0;

		nodes.emplace_back(0u, count);
		stack[stackSize++] = 0u;

		while (stackSize > 0)
		{
			const uint32_t nodeIdx = stack[--stackSize];
			BvhNode& node = nodes[nodeIdx];

			const uint32_t nodeStart = node.start;
			const uint32_t nodeEnd = node.end;
			const uint32_t rangeSize = nodeEnd - nodeStart;

			if (rangeSize <= BvhNode::KD_LEAF_SIZE)
			{
				node.leafIndex = leafCount++;
				continue;
			}

			const uint32_t mcFirst = mortonCodePtr[nodeStart];
			const uint32_t mcLast = mortonCodePtr[nodeEnd - 1];

			uint32_t mid;
			if (mcFirst == mcLast) [[unlikely]]
			{
				mid = nodeStart + (rangeSize >> 1);
			}
			else
			{
				// Codes are sorted: [nodeStart, mid) have splitBit clear, [mid, nodeEnd) have it set.
				const uint32_t splitBit = std::bit_floor(mcFirst ^ mcLast);

				uint32_t lo = nodeStart, hi = nodeEnd - 1;
				while (lo < hi)
				{
					const uint32_t m = (lo + hi) >> 1;
					if ((mortonCodePtr[m] & splitBit) == 0u)
						lo = m + 1;
					else
						hi = m;
				}

				mid = std::clamp(lo, nodeStart + 1u, nodeEnd - 1u); // Non-empty children.
			}

			const uint32_t leftIdx = static_cast<uint32_t>(nodes.size());
			node.leftChildIndex = leftIdx; // Before emplace_back: 'node' may dangle after it.

			stack[stackSize++] = leftIdx + 1;
			stack[stackSize++] = leftIdx;

			nodes.emplace_back(nodeStart, mid);
			nodes.emplace_back(mid, nodeEnd);
		}

		return leafCount;
	}

	void BvhTree::fitNodes(const AABBSoAViewer& aabbs, bool isRebuild)
	{
		TRACY_SCOPE_N("Fit bvh nodes");

		// InternalRealSimd must hold exactly 4 Reals.
		using InternalRealSimd = std::conditional_t<
			std::is_same_v<Real, float>,
			Ecstasy::Core::Simd<Real, 128>,
			Ecstasy::Core::Simd<Real, 256>
		>;

		using InternalUintSimd = std::conditional_t<
			std::is_same_v<Real, float>,
			Ecstasy::Core::Simd<uint32_t, 128>,
			Ecstasy::Core::Simd<uint64_t, 256>
		>;

		// 'Set' stores in reverse order.
		const InternalRealSimd minMaxBlendMask = InternalUintSimd::set(-1, 0, -1, 0).as<InternalRealSimd>();

		const ObjectIndex* ECSTASY_RESTRICT indicesPtr = colliderIndices.data();
		const Real* ECSTASY_RESTRICT bodyMinXPtr = aabbs.minX;
		const Real* ECSTASY_RESTRICT bodyMaxXPtr = aabbs.maxX;
		const Real* ECSTASY_RESTRICT bodyMinYPtr = aabbs.minY;
		const Real* ECSTASY_RESTRICT bodyMaxYPtr = aabbs.maxY;

		const size_t nodeCount = nodes.size();

		constexpr Real DEAD_MIN = std::numeric_limits<Real>::max();
		constexpr Real DEAD_MAX = -DEAD_MIN;
		if (isRebuild)
		{
			// Faster (1%) to write the whole range at once than the leftovers in the loop.
			TRACY_SCOPE_N("Dead writes");

			const size_t totalReals = leafAABBs.minX.size() * BvhNode::KD_LEAF_SIZE;

			const RealSimd deadMinV(DEAD_MIN);
			const RealSimd deadMaxV(DEAD_MAX);

			Real* minXPtr = reinterpret_cast<Real*>(leafAABBs.minX.data());
			Real* maxXPtr = reinterpret_cast<Real*>(leafAABBs.maxX.data());
			Real* minYPtr = reinterpret_cast<Real*>(leafAABBs.minY.data());
			Real* maxYPtr = reinterpret_cast<Real*>(leafAABBs.maxY.data());
			for (size_t i = 0; i < totalReals; i += LANES)
			{
				deadMinV.store(minXPtr + i);
				deadMaxV.store(maxXPtr + i);
				deadMinV.store(minYPtr + i);
				deadMaxV.store(maxYPtr + i);
			}
		}

		for (size_t idx = nodeCount; idx-- > 0; ) // Reverse order.
		{
			BvhNode& node = nodes[idx];
			if (node.leftChildIndex != BvhNode::INVALID_INDEX)
			{
				// Internal: merge children.
				const BvhNode& left = nodes[node.leftChildIndex];
				const BvhNode& right = nodes[node.leftChildIndex + 1];

				const InternalRealSimd leftV = InternalRealSimd::loadu(&left.minX);
				const InternalRealSimd rightV = InternalRealSimd::loadu(&right.minX);
				const auto minV = InternalRealSimd::min(leftV, rightV);
				const auto maxV = InternalRealSimd::max(leftV, rightV);
				const auto result = InternalRealSimd::blendv(minV, maxV, minMaxBlendMask);
				result.storeu(&node.minX);
				continue;
			}

			// Leaf: copy collider AABBs into the leaf SoA and merge them.
			Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafAABBs.minX.data() + node.leafIndex);
			Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafAABBs.maxX.data() + node.leafIndex);
			Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafAABBs.minY.data() + node.leafIndex);
			Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafAABBs.maxY.data() + node.leafIndex);

			node.minX = DEAD_MIN;
			node.maxX = DEAD_MAX;
			node.minY = DEAD_MIN;
			node.maxY = DEAD_MAX;

			const uint32_t nodeStart = node.start;
			const uint32_t nodeRange = node.end - nodeStart;

			for (uint32_t k = 0; k < nodeRange; k++)
			{
				const ObjectIndex colliderIndex = indicesPtr[nodeStart + k];

				const Real bodyMinX = bodyMinXPtr[colliderIndex];
				const Real bodyMaxX = bodyMaxXPtr[colliderIndex];
				const Real bodyMinY = bodyMinYPtr[colliderIndex];
				const Real bodyMaxY = bodyMaxYPtr[colliderIndex];

				leafMinXPtr[k] = bodyMinX;
				leafMaxXPtr[k] = bodyMaxX;
				leafMinYPtr[k] = bodyMinY;
				leafMaxYPtr[k] = bodyMaxY;

				node.minX = std::fmin(node.minX, bodyMinX);
				node.maxX = std::fmax(node.maxX, bodyMaxX);
				node.minY = std::fmin(node.minY, bodyMinY);
				node.maxY = std::fmax(node.maxY, bodyMaxY);
			}
		}
	}
}
