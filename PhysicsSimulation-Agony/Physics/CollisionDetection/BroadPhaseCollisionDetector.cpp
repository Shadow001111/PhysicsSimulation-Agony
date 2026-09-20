#include "BroadPhaseCollisionDetector.h"

#include "BroadPhaseInternal/LeafPairTesting.h"
#include "BroadPhaseInternal/BvhPairTraversal.h"

#include "Physics/Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"

#include <numeric>
#include <algorithm>
#include <type_traits>

namespace PS_AGONY
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	// Used by computeCentroidsWithTransformations and fitBvhNodeAABBs. Unrelated
	// to BroadPhaseInternal::LEAF_TEST_LANES, which is the leaf-test SIMD width.
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


	static inline uint32_t part1By1(uint32_t x)
	{
		x &= 0x0000ffffu;
		x = (x | (x << 8)) & 0x00FF00FFu;
		x = (x | (x << 4)) & 0x0F0F0F0Fu;
		x = (x | (x << 2)) & 0x33333333u;
		x = (x | (x << 1)) & 0x55555555u;
		return x;
	}

	static inline uint32_t morton2D(uint32_t x, uint32_t y)
	{
		return (part1By1(y) << 1) | part1By1(x);
	}

	static inline MortonU32Simd part1By1Simd(MortonU32Simd x)
	{
		x &= MortonU32Simd(0x0000ffffu);
		x = (x | (x << 8)) & MortonU32Simd(0x00FF00FFu);
		x = (x | (x << 4)) & MortonU32Simd(0x0F0F0F0Fu);
		x = (x | (x << 2)) & MortonU32Simd(0x33333333u);
		x = (x | (x << 1)) & MortonU32Simd(0x55555555u);
		return x;
	}

	static inline MortonU32Simd morton2DSimd(const MortonU32Simd& x, const MortonU32Simd& y)
	{
		return (part1By1Simd(y) << 1) | part1By1Simd(x);
	}

	void BroadPhaseCollisionDetector::setDataViewers(const AABBSoAViewer& aabbs, const ObjectIndex* colliderBodyIndexIn)
	{
		collidersAABB = aabbs;
		colliderBodyIndex = colliderBodyIndexIn;
	}

	void BroadPhaseCollisionDetector::clearData()
	{
		collisionData.clear();
		bvhFunctionResources.nodes.clear();
		bvhFunctionResources.mainColliderIndices.clear();
	}

	void BroadPhaseCollisionDetector::buildTree(bool rebuild)
	{
		TRACY_SCOPE_N("Build broad-phase tree");

		const size_t colliderCount = collidersAABB.getCount();

		if (colliderCount == 0)
		{
			// No colliders: drop the old tree so fetchAABBs/fetchCollidersInCircle/
			// fetchCollidersInAABB don't keep serving a stale, non-existent scene.
			bvhFunctionResources.nodes.clear();
			bvhFunctionResources.mainColliderIndices.clear();
			return;
		}

		if (rebuild)
		{
			auto& nodes = bvhFunctionResources.nodes;
			auto& indices = bvhFunctionResources.mainColliderIndices;
			nodes.clear();
			nodes.reserve(colliderCount / 4);

			indices.resize(colliderCount);
			std::iota(indices.begin(), indices.end(), 0);

			buildBvhTree(static_cast<uint32_t>(colliderCount)); // Handles colliderCount == 1 (single leaf) too.
		}
		else
		{
			fitBvhNodeAABBs(false);
		}
	}

	const std::vector<ObjectPair>& BroadPhaseCollisionDetector::findCollisions(ExecutionPolicy executionPolicy)
	{
		TRACY_SCOPE_N("Find broad-phase collisions");

		collisionData.clear();

		const size_t colliderCount = collidersAABB.getCount();
		if (colliderCount < 2) return collisionData; // No pairs to check. Caller should already guard this; kept defensively.

		collisionData.reserve(colliderCount);

		bool useThreading = false;

		if (executionPolicy == ExecutionPolicy::ForceMultiThreaded)
		{
			useThreading = true;
		}
		else if (executionPolicy == ExecutionPolicy::ForceSingleThreaded)
		{
			useThreading = false;
		}
		else
		{
			useThreading = bvhFunctionResources.nodes.size() > 400;
		}

		if (useThreading)
		{
			queryBvhPairsThreaded();
		}
		else
		{
			queryBvhPairs();
		}

		return collisionData;
	}

	void BroadPhaseCollisionDetector::getAllLeafNodeAABBs(std::vector<AABB>& outAABBs) const
	{
		// Collect BVH nodes (leafs) AABBs from previous time.
		const auto& nodes = bvhFunctionResources.nodes;
		outAABBs.reserve(outAABBs.size() + nodes.size());
		for (const auto& node : nodes)
		{
			if (node.leftChildIndex == BvhNode::INVALID_INDEX) // Leaf check.
			{
				outAABBs.emplace_back(node.minX, node.minY, node.maxX, node.maxY);
			}
		}
	}

	size_t BroadPhaseCollisionDetector::getMemoryUsage() const
	{
		size_t total = sizeof(BroadPhaseCollisionDetector);

		total += getVectorMemoryUsage(bvhFunctionResources.nodes);

		total += getVectorMemoryUsage(bvhFunctionResources.transformedCentroidX);
		total += getVectorMemoryUsage(bvhFunctionResources.transformedCentroidY);

		total += getVectorMemoryUsage(bvhFunctionResources.mortonCodes);

		total += getVectorMemoryUsage(bvhFunctionResources.mainColliderIndices);
		total += getVectorMemoryUsage(bvhFunctionResources.tempPackedBodyIndicesToSort);
		total += getVectorMemoryUsage(bvhFunctionResources.tempPackedBodyIndicesToSort2);

		total += getVectorMemoryUsage(bvhFunctionResources.nodePairsToTraverse);
		total += getVectorMemoryUsage(bvhFunctionResources.leafPairsToTestCollisions);
		total += getVectorMemoryUsage(bvhFunctionResources.leavesToTestCollisions);

		total += getVectorMemoryUsage(queryPairsThreadedResources.workerData);
		for (const auto& wData : queryPairsThreadedResources.workerData)
		{
			total += getVectorMemoryUsage(wData.nodePairsToTraverse);
			total += getVectorMemoryUsage(wData.leafPairsToTestCollisions);
			total += getVectorMemoryUsage(wData.collisionData);
		}

		total += getVectorMemoryUsage(leafBodyAABBs.minX);
		total += getVectorMemoryUsage(leafBodyAABBs.maxX);
		total += getVectorMemoryUsage(leafBodyAABBs.minY);
		total += getVectorMemoryUsage(leafBodyAABBs.maxY);

		total += getVectorMemoryUsage(collisionData);
		return total;
	}

	void BroadPhaseCollisionDetector::computeCentroidsWithTransformations(uint32_t colliderCount, Vec2 globalMin, Vec2 scale, Real clampMax)
	{
		// Get pointers.
		const Real* ECSTASY_RESTRICT aabbMinXPtr = collidersAABB.minX;
		const Real* ECSTASY_RESTRICT aabbMinYPtr = collidersAABB.minY;
		const Real* ECSTASY_RESTRICT aabbMaxXPtr = collidersAABB.maxX;
		const Real* ECSTASY_RESTRICT aabbMaxYPtr = collidersAABB.maxY;

		Real* ECSTASY_RESTRICT centroidXPtr = nullptr;
		Real* ECSTASY_RESTRICT centroidYPtr = nullptr;
		{
			auto& centroidX = bvhFunctionResources.transformedCentroidX;
			auto& centroidY = bvhFunctionResources.transformedCentroidY;
			centroidX.resize(colliderCount);
			centroidY.resize(colliderCount);
			centroidXPtr = centroidX.data();
			centroidYPtr = centroidY.data();
		}

		// Start tracing here, closer for capturing computation.
		TRACY_SCOPE_N("Compute centroids");

		// Vector variables.
		const RealSimd scaledGlobalMinXV(globalMin.x * scale.x);
		const RealSimd scaledGlobalMinYV(globalMin.y * scale.y);

		const RealSimd halfScaleXV(scale.x * Real(0.5));
		const RealSimd halfScaleYV(scale.y * Real(0.5));

		const RealSimd clampMaxV(clampMax);

		// Compute centroids.
		size_t i = 0;
		for (; i + LANES <= colliderCount; i += LANES)
		{
			const RealSimd minX = RealSimd::load(aabbMinXPtr + i);
			const RealSimd maxX = RealSimd::load(aabbMaxXPtr + i);
			const RealSimd minY = RealSimd::load(aabbMinYPtr + i);
			const RealSimd maxY = RealSimd::load(aabbMaxYPtr + i);

			const RealSimd tx = RealSimd::mulSub(minX + maxX, halfScaleXV, scaledGlobalMinXV);
			const RealSimd ty = RealSimd::mulSub(minY + maxY, halfScaleYV, scaledGlobalMinYV);

			const RealSimd clampedX = RealSimd::clamp(tx, RealSimd(0), clampMaxV);
			const RealSimd clampedY = RealSimd::clamp(ty, RealSimd(0), clampMaxV);

			clampedX.store(centroidXPtr + i);
			clampedY.store(centroidYPtr + i);
		}
		for (; i < colliderCount; i++)
		{
			const Real cx = (aabbMinXPtr[i] + aabbMaxXPtr[i]) * Real(0.5);
			const Real cy = (aabbMinYPtr[i] + aabbMaxYPtr[i]) * Real(0.5);

			const Real tx = (cx - globalMin.x) * scale.x;
			const Real ty = (cy - globalMin.y) * scale.y;

			const Real clampedX = std::clamp(tx, Real(0), clampMax);
			const Real clampedY = std::clamp(ty, Real(0), clampMax);

			centroidXPtr[i] = clampedX;
			centroidYPtr[i] = clampedY;
		}
	}

	template<std::floating_point TReal>
	void BroadPhaseCollisionDetector::computeMortonCodes(uint32_t colliderCount)
	{
		using TRealSimd = Ecstasy::Core::Simd<TReal>;

		bvhFunctionResources.mortonCodes.resize(colliderCount);

		uint32_t* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

		const TReal* ECSTASY_RESTRICT centroidXPtr = bvhFunctionResources.transformedCentroidX.data();
		const TReal* ECSTASY_RESTRICT centroidYPtr = bvhFunctionResources.transformedCentroidY.data();

		TRACY_SCOPE_N("Compute morton codes");

		size_t i = 0;

		// SIMD loop. Runs only when lane counts match.
		if constexpr (MortonU32Simd::lanes == TRealSimd::lanes)
		{
			for (; i + TRealSimd::lanes <= colliderCount; i += TRealSimd::lanes)
			{
				// Cx/cy are in range [0; 2^16]
				const MortonU32Simd qx = TRealSimd::load(centroidXPtr + i).template to<MortonI32Simd>().template as<MortonU32Simd>();
				const MortonU32Simd qy = TRealSimd::load(centroidYPtr + i).template to<MortonI32Simd>().template as<MortonU32Simd>();

				const MortonU32Simd code = morton2DSimd(qx, qy);

				code.store(mortonCodePtr + i);
			}
		}

		// Scalar loop. Handles remaining elements.
		for (; i < colliderCount; i++)
		{
			const uint32_t qx = static_cast<uint32_t>(centroidXPtr[i]);
			const uint32_t qy = static_cast<uint32_t>(centroidYPtr[i]);
			mortonCodePtr[i] = morton2D(qx, qy);
		}
	}

	template void BroadPhaseCollisionDetector::computeMortonCodes<Real>(uint32_t);

	void BroadPhaseCollisionDetector::sortBodyIndicesByMortonCodes(uint32_t colliderCount)
	{
		TRACY_SCOPE_N("Sort indices by morton codes");

		constexpr uint32_t RADIX_BITS = 8;
		constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
		constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

		uint32_t* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

		// Build packed key/index array once.
		bvhFunctionResources.tempPackedBodyIndicesToSort.resize(colliderCount);
		bvhFunctionResources.tempPackedBodyIndicesToSort2.resize(colliderCount);

		PackedBodyIndex* ECSTASY_RESTRICT packedA = bvhFunctionResources.tempPackedBodyIndicesToSort.data();
		PackedBodyIndex* ECSTASY_RESTRICT packedB = bvhFunctionResources.tempPackedBodyIndicesToSort2.data();

		// Pack.
		for (uint32_t i = 0; i < colliderCount; i++)
		{
			PackedBodyIndex& pbi = packedA[i];
			pbi.key = mortonCodePtr[i];
			pbi.index = i;
		}

		// Sort.
		alignas(64) uint32_t count[4][RADIX_SIZE] = {};

		// Note: Combining packing pass with counting pass worsens perfomance.
		{
			TRACY_SCOPE_N("Counting pass");
			for (uint32_t i = 0; i < colliderCount; i++)
			{
				const uint32_t k = packedA[i].key;
				count[0][k & 0xFF]++;
				count[1][(k >> 8) & 0xFF]++;
				count[2][(k >> 16) & 0xFF]++;
				count[3][(k >> 24) & 0xFF]++;
			}
		}

		auto radixPass = [&](
			uint32_t shift,
			const PackedBodyIndex* ECSTASY_RESTRICT src,
			PackedBodyIndex* ECSTASY_RESTRICT dst,
			uint32_t* ECSTASY_RESTRICT countSegment
			)
			{
				// Exclusive prefix sum.
				uint32_t sum = 0;
				for (uint32_t i = 0; i < RADIX_SIZE; i++)
				{
					const uint32_t c = countSegment[i];
					countSegment[i] = sum;
					sum += c;
				}

				// Scatter (stable).
				for (uint32_t i = 0; i < colliderCount; i++)
				{
					const PackedBodyIndex item = src[i];
					const uint32_t key = (item.key >> shift) & RADIX_MASK;
					dst[countSegment[key]++] = item;
				}
			};

		{
			TRACY_SCOPE_N("Sort");

			radixPass(0, packedA, packedB, count[0]);
			radixPass(8, packedB, packedA, count[1]);
			radixPass(16, packedA, packedB, count[2]);
			radixPass(24, packedB, packedA, count[3]);
		}

		// Write sorted indices and keys(codes) back.
		ObjectIndex* ECSTASY_RESTRICT indexPtr = bvhFunctionResources.mainColliderIndices.data();
		for (uint32_t i = 0; i < colliderCount; i++)
		{
			const PackedBodyIndex item = packedA[i];
			indexPtr[i] = item.index;
			mortonCodePtr[i] = item.key;
		}
	}

	void BroadPhaseCollisionDetector::buildBvhTree(const uint32_t colliderCount)
	{
		TRACY_SCOPE_N("Build tree");

		const Real* ECSTASY_RESTRICT bodyMinXPtr = collidersAABB.minX;
		const Real* ECSTASY_RESTRICT bodyMaxXPtr = collidersAABB.maxX;
		const Real* ECSTASY_RESTRICT bodyMinYPtr = collidersAABB.minY;
		const Real* ECSTASY_RESTRICT bodyMaxYPtr = collidersAABB.maxY;

		// Compute world AABB.
		Real globalMinX, globalMaxX, globalMinY, globalMaxY;
		{
			TRACY_SCOPE_N("Compute world AABB");
			globalMinX = std::numeric_limits<Real>::max();
			globalMaxX = -std::numeric_limits<Real>::max();
			globalMinY = std::numeric_limits<Real>::max();
			globalMaxY = -std::numeric_limits<Real>::max();

			for (size_t i = 0; i < colliderCount; i++)
			{
				globalMinX = std::fmin(globalMinX, bodyMinXPtr[i]);
				globalMaxX = std::fmax(globalMaxX, bodyMaxXPtr[i]);
				globalMinY = std::fmin(globalMinY, bodyMinYPtr[i]);
				globalMaxY = std::fmax(globalMaxY, bodyMaxYPtr[i]);
			}

			// Prevent division by zero for degenerate scenes.
			constexpr Real kEps = Real(1e-5);
			globalMaxX = std::fmax(globalMaxX, globalMinX + kEps);
			globalMaxY = std::fmax(globalMaxY, globalMinY + kEps);
		}

		// Compute centroids.
		{
			const Real scaleX = Real(0xFFFFu) / (globalMaxX - globalMinX);
			const Real scaleY = Real(0xFFFFu) / (globalMaxY - globalMinY);
			computeCentroidsWithTransformations(colliderCount, { globalMinX, globalMinY }, { scaleX, scaleY }, Real(0xFFFFu));
		}

		// Compute morton codes.
		computeMortonCodes<Real>(colliderCount);

		// Sort indices by morton code.
		sortBodyIndicesByMortonCodes(colliderCount);
		const uint32_t* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

		// Top-down tree build with Morton-code binary split.
		// For a node covering sorted range [nodeStart, nodeEnd):
		//   • XOR the first and last Morton codes to find the highest bit
		//     where they differ (the "split bit").
		//   • Binary-search for the boundary between codes that have the
		//     split bit clear (left child) and those that have it set
		//     (right child).
		//   • Fall back to a median split when all codes in the range are
		//     identical (perfectly overlapping colliders).
		size_t leafCount = 0;
		{
			TRACY_SCOPE_N("Stack loop");
			struct BuildTask { uint32_t nodeIdx; };

			std::array<BuildTask, MAX_STACK_CAPACITY> stack;
			uint32_t stackSize = 0;

			bvhFunctionResources.nodes.emplace_back(0u, colliderCount);
			stack[stackSize++] = { 0u };

			while (stackSize > 0)
			{
				const BuildTask task = stack[--stackSize];
				BvhNode& node = bvhFunctionResources.nodes[task.nodeIdx];

				const uint32_t nodeStart = node.start;
				const uint32_t nodeEnd = node.end;
				const uint32_t rangeSize = nodeEnd - nodeStart;

				if (rangeSize <= BvhNode::KD_LEAF_SIZE)
				{
					// Leaf - nothing more to split.
					node.leafIndex = leafCount++;
					continue;
				}

				// Find morton codes
				const uint32_t mcFirst = mortonCodePtr[nodeStart];
				const uint32_t mcLast = mortonCodePtr[nodeEnd - 1];

				// Find the split position.
				uint32_t mid;
				if (mcFirst == mcLast) [[unlikely]]
				{
					// All colliders hash to the same Morton cell; equal codes can't
					// be meaningfully split, so fall back to a balanced median.
					mid = nodeStart + (rangeSize >> 1);
				}
				else
				{
					// Highest bit where the first and last codes disagree.
					// Because the array is sorted, all codes in [nodeStart, mid)
					// have this bit clear and all in [mid, nodeEnd) have it set.
					const uint32_t splitBit = std::bit_floor(mcFirst ^ mcLast);

					// Binary-search for the first index with splitBit set.
					uint32_t lo = nodeStart, hi = nodeEnd - 1;
					while (lo < hi)
					{
						const uint32_t m = (lo + hi) >> 1;
						if ((mortonCodePtr[m] & splitBit) == 0u)
							lo = m + 1;
						else
							hi = m;
					}

					// Clamp defensively to guarantee non-empty children.
					mid = std::clamp(lo, nodeStart + 1u, nodeEnd - 1u);
				}

				const uint32_t leftIdx = static_cast<uint32_t>(bvhFunctionResources.nodes.size());
				node.leftChildIndex = leftIdx;

				stack[stackSize++] = { leftIdx + 1 };
				stack[stackSize++] = { leftIdx };

				bvhFunctionResources.nodes.emplace_back(nodeStart, mid);
				bvhFunctionResources.nodes.emplace_back(mid, nodeEnd);
			}
		}

		leafBodyAABBs.minX.resize(leafCount);
		leafBodyAABBs.maxX.resize(leafCount);
		leafBodyAABBs.minY.resize(leafCount);
		leafBodyAABBs.maxY.resize(leafCount);
		fitBvhNodeAABBs(true);
	}

	void BroadPhaseCollisionDetector::fitBvhNodeAABBs(bool isRebuild)
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

		// Setting mask. 'Set' stores in reverse order.
		const InternalRealSimd minMaxBlendMask = InternalUintSimd::set(-1, 0, -1, 0).as<InternalRealSimd>();

		const ObjectIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainColliderIndices.data();
		const Real* ECSTASY_RESTRICT bodyMinXPtr = collidersAABB.minX;
		const Real* ECSTASY_RESTRICT bodyMaxXPtr = collidersAABB.maxX;
		const Real* ECSTASY_RESTRICT bodyMinYPtr = collidersAABB.minY;
		const Real* ECSTASY_RESTRICT bodyMaxYPtr = collidersAABB.maxY;

		const size_t nodeCount = bvhFunctionResources.nodes.size();

		constexpr Real DEAD_MIN = std::numeric_limits<Real>::max();
		constexpr Real DEAD_MAX = -DEAD_MIN;
		if (isRebuild)
		{
			// It's faster (1%) to write whole range at once than doing it for leftovers in loop.
			TRACY_SCOPE_N("Dead writes");

			// Total number of individual 'Real' elements in one vector.
			const size_t totalReals = leafBodyAABBs.minX.size() * BvhNode::KD_LEAF_SIZE;

			const RealSimd deadMinV(DEAD_MIN);
			const RealSimd deadMaxV(DEAD_MAX);

			Real* minXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
			Real* maxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
			Real* minYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
			Real* maxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
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
			BvhNode& node = bvhFunctionResources.nodes[idx];
			if (node.leftChildIndex != BvhNode::INVALID_INDEX)
			{
				// Not leaf: compute AABB from its children.
				const BvhNode& left = bvhFunctionResources.nodes[node.leftChildIndex];
				const BvhNode& right = bvhFunctionResources.nodes[node.leftChildIndex + 1];

				const InternalRealSimd leftV = InternalRealSimd::loadu(&left.minX);
				const InternalRealSimd rightV = InternalRealSimd::loadu(&right.minX);
				const auto minV = InternalRealSimd::min(leftV, rightV);
				const auto maxV = InternalRealSimd::max(leftV, rightV);
				const auto result = InternalRealSimd::blendv(minV, maxV, minMaxBlendMask);
				result.storeu(&node.minX);
				continue;
			}

			// Leaf: compute AABB from its colliders.
			const uint32_t leafIndex = node.leafIndex;
			Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data() + leafIndex);
			Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data() + leafIndex);
			Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data() + leafIndex);
			Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data() + leafIndex);

			Real minX = DEAD_MIN;
			Real maxX = DEAD_MAX;
			Real minY = DEAD_MIN;
			Real maxY = DEAD_MAX;

			const uint32_t nodeStart = node.start;
			const uint32_t nodeRange = node.end - nodeStart;

			for (uint32_t leafBodyIndex = 0; leafBodyIndex < nodeRange; leafBodyIndex++)
			{
				const ObjectIndex colliderIndex = indicesPtr[nodeStart + leafBodyIndex];

				const Real bodyMinX = bodyMinXPtr[colliderIndex];
				const Real bodyMaxX = bodyMaxXPtr[colliderIndex];
				const Real bodyMinY = bodyMinYPtr[colliderIndex];
				const Real bodyMaxY = bodyMaxYPtr[colliderIndex];

				leafMinXPtr[leafBodyIndex] = bodyMinX;
				leafMaxXPtr[leafBodyIndex] = bodyMaxX;
				leafMinYPtr[leafBodyIndex] = bodyMinY;
				leafMaxYPtr[leafBodyIndex] = bodyMaxY;

				minX = std::fmin(minX, bodyMinX);
				maxX = std::fmax(maxX, bodyMaxX);
				minY = std::fmin(minY, bodyMinY);
				maxY = std::fmax(maxY, bodyMaxY);

			}
			node.minX = minX; node.maxX = maxX;
			node.minY = minY; node.maxY = maxY;
		}
	}

	void BroadPhaseCollisionDetector::queryBvhPairs()
	{
		TRACY_SCOPE_N("Query pairs");

		traverseNodesToGetOverlappingLeafPairs();
		testCollisionsInLeaves();
	}

	void BroadPhaseCollisionDetector::queryBvhPairsThreaded()
	{
		TRACY_SCOPE_N("Query pairs (Threaded)");

		auto& threadPool = Threading::getGlobalThreadPool();
		const size_t workerCount = threadPool.getThreadCount() - 1;

		// Clear.
		bvhFunctionResources.nodePairsToTraverse.clear();
		bvhFunctionResources.leafPairsToTestCollisions.clear();
		bvhFunctionResources.leavesToTestCollisions.clear();

		// Get pointers.
		const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
		const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
		const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
		const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
		const BvhNode* ECSTASY_RESTRICT nodesPtr = bvhFunctionResources.nodes.data();
		const ObjectIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainColliderIndices.data();
		const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliderBodyIndex;

		// Initial traverse to get independent sub-trees. Same routine the
		// single-threaded path runs (see traverseNodesToGetOverlappingLeafPairs).
		{
			TRACY_SCOPE_N("Initial traverse");
			BroadPhaseInternal::traverseRootForInitialPairs(
				nodesPtr,
				bvhFunctionResources.nodePairsToTraverse,
				bvhFunctionResources.leafPairsToTestCollisions,
				bvhFunctionResources.leavesToTestCollisions);
		}

		// Initialize workers.
		queryPairsThreadedResources.workerData.resize(workerCount);

		// Launch workers to traverse independent sub-trees.
		alignas(64) std::atomic<uint32_t> globalWorkerIndex{ 0 };
		{
			TRACY_SCOPE_N("Launch workers");

			auto workerFunc = [&]()
				{
					TRACY_SCOPE_N("Query pairs worker thread");

					constexpr size_t TASK_STEAL_RANGE = 1; // From main thread.

					const size_t maxPairsToTraverse = bvhFunctionResources.nodePairsToTraverse.size();
					const size_t maxLeavesToTest = bvhFunctionResources.leavesToTestCollisions.size();
					const size_t maxLeafPairsToTest = bvhFunctionResources.leafPairsToTestCollisions.size();

					// Worker data.
					const auto workerIndex = globalWorkerIndex.fetch_add(1, std::memory_order_relaxed);
					auto& wData = queryPairsThreadedResources.workerData[workerIndex];

					// Masks and push buffer, shared across all leaf tests this worker runs.
					std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;
					BroadPhaseInternal::PairPushBuffer pushBuffer(wData.collisionData);

					// Traverse. Same routine the single-threaded path runs once over the
					// whole frontier (see traverseNodesToGetOverlappingLeafPairs); here each
					// worker drains one stolen task (an independent sub-tree) at a time.
					{
						TRACY_SCOPE_N("Traverse");

						size_t taskIndex = workerIndex;
						while (taskIndex < maxPairsToTraverse)
						{
							wData.nodePairsToTraverse.clear();
							wData.nodePairsToTraverse.push_back(bvhFunctionResources.nodePairsToTraverse[taskIndex]);
							BroadPhaseInternal::traverseNodePairStack(nodesPtr, wData.nodePairsToTraverse, wData.leafPairsToTestCollisions);

							taskIndex += workerCount;
						}
					}

					// Test initial leaf self cross collisions. Same routine the single-threaded
					// path runs (see testCollisionsInLeaves).
					{
						TRACY_SCOPE_N("Test initial leaf self cross collisions");

						size_t taskIndex = workerIndex * TASK_STEAL_RANGE;
						while (taskIndex < maxLeavesToTest)
						{
							const size_t end = std::min<size_t>(taskIndex + TASK_STEAL_RANGE, maxLeavesToTest);

							for (size_t testIndex = taskIndex; testIndex < end; testIndex++)
							{
								auto nodeIndex = bvhFunctionResources.leavesToTestCollisions[testIndex];
								BroadPhaseInternal::testLeafSelfCross(
									bvhFunctionResources.nodes[nodeIndex],
									leafMinXPtr, leafMaxXPtr, leafMinYPtr, leafMaxYPtr,
									indicesPtr, colliderOwnerPtr, masks, pushBuffer);
							}

							taskIndex += workerCount * TASK_STEAL_RANGE;
						}
					}

					// Test initial leaf pair cross collisions. Same routine the single-threaded
					// path runs (see testCollisionsInLeaves).
					{
						TRACY_SCOPE_N("Test initial leaf pair cross collisions");

						size_t taskIndex = workerIndex * TASK_STEAL_RANGE;
						while (taskIndex < maxLeafPairsToTest)
						{
							const size_t end = std::min<size_t>(taskIndex + TASK_STEAL_RANGE, maxLeafPairsToTest);

							for (size_t testIndex = taskIndex; testIndex < end; testIndex++)
							{
								auto [nodeIndexA, nodeIndexB] = bvhFunctionResources.leafPairsToTestCollisions[testIndex];
								BroadPhaseInternal::testLeafPairCross(
									nodesPtr[nodeIndexA], nodesPtr[nodeIndexB],
									leafMinXPtr, leafMaxXPtr, leafMinYPtr, leafMaxYPtr,
									indicesPtr, colliderOwnerPtr, masks, pushBuffer);
							}

							taskIndex += workerCount * TASK_STEAL_RANGE;
						}
					}

					// Test collisions found while draining this worker's own sub-tree.
					// Same routine as above (testLeafPairCross), just a different source container.
					if (!wData.leafPairsToTestCollisions.empty())
					{
						TRACY_SCOPE_N("Test collisions");

						for (auto [nodeIndexA, nodeIndexB] : wData.leafPairsToTestCollisions)
						{
							BroadPhaseInternal::testLeafPairCross(
								nodesPtr[nodeIndexA], nodesPtr[nodeIndexB],
								leafMinXPtr, leafMaxXPtr, leafMinYPtr, leafMaxYPtr,
								indicesPtr, colliderOwnerPtr, masks, pushBuffer);
						}
						wData.leafPairsToTestCollisions.clear();

						// Last flush.
						if (pushBuffer.size > 0) pushBuffer.flush();
					}

					// Last flush.
					if (pushBuffer.size > 0) pushBuffer.flush();

					// Done.
					wData.isDone.store(true, std::memory_order_release);
					wData.isDone.notify_one();
				};

			// Init workers.
			for (auto& wData : queryPairsThreadedResources.workerData)
			{
				wData.nodePairsToTraverse.clear();
				wData.leafPairsToTestCollisions.clear();
				wData.collisionData.clear();
				wData.isDone = false;
			}

			// Launch workers.
			std::vector<Ecstasy::Core::Threading::Task> tasks;
			tasks.reserve(workerCount);

			for (size_t i = 0; i < workerCount; i++)
			{
				tasks.emplace_back(workerFunc);
			}
			threadPool.enqueueBulk(tasks);
		}

		// Wait for workers to finish and combine their data.
		{
			TRACY_SCOPE_N("Wait for workers to finish and combine data");
			for (auto& wData : queryPairsThreadedResources.workerData)
			{
				wData.isDone.wait(false, std::memory_order_acquire);
				{
					TRACY_SCOPE_N("Combine");
					collisionData.insert(
						collisionData.end(),
						wData.collisionData.begin(),
						wData.collisionData.end()
					);
				}
			}
		}
	}

	void BroadPhaseCollisionDetector::traverseNodesToGetOverlappingLeafPairs()
	{
		bvhFunctionResources.nodePairsToTraverse.clear();
		bvhFunctionResources.leafPairsToTestCollisions.clear();
		bvhFunctionResources.leavesToTestCollisions.clear();

		// Traverse 1: build the initial frontier from the root.
		{
			TRACY_SCOPE_N("Traverse 1");
			BroadPhaseInternal::traverseRootForInitialPairs(
				bvhFunctionResources.nodes.data(),
				bvhFunctionResources.nodePairsToTraverse,
				bvhFunctionResources.leafPairsToTestCollisions,
				bvhFunctionResources.leavesToTestCollisions);
		}

		// Traverse 2: drain the frontier down to leaf/leaf pairs.
		// Same routine the threaded path runs per worker task (see queryBvhPairsThreaded).
		{
			TRACY_SCOPE_N("Traverse 2");
			BroadPhaseInternal::traverseNodePairStack(
				bvhFunctionResources.nodes.data(),
				bvhFunctionResources.nodePairsToTraverse,
				bvhFunctionResources.leafPairsToTestCollisions);
		}
	}

	void BroadPhaseCollisionDetector::testCollisionsInLeaves()
	{
		TRACY_SCOPE_N("Test collisions in leaves");

		// Get pointers.
		const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
		const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
		const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
		const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
		const ObjectIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainColliderIndices.data();
		const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliderBodyIndex;

		std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;
		BroadPhaseInternal::PairPushBuffer pushBuffer(collisionData);

		// Test self cross. Same routine the threaded path runs per worker (see queryBvhPairsThreaded).
		{
			TRACY_SCOPE_N("Self cross");
			for (const uint32_t nodeIndex : bvhFunctionResources.leavesToTestCollisions)
			{
				BroadPhaseInternal::testLeafSelfCross(
					bvhFunctionResources.nodes[nodeIndex],
					leafMinXPtr, leafMaxXPtr, leafMinYPtr, leafMaxYPtr,
					indicesPtr, colliderOwnerPtr, masks, pushBuffer);
			}
		}

		// Test AxB cross. Same routine the threaded path runs per worker (see queryBvhPairsThreaded).
		{
			TRACY_SCOPE_N("AxB cross");
			for (auto [nodeIndexA, nodeIndexB] : bvhFunctionResources.leafPairsToTestCollisions)
			{
				BroadPhaseInternal::testLeafPairCross(
					bvhFunctionResources.nodes[nodeIndexA], bvhFunctionResources.nodes[nodeIndexB],
					leafMinXPtr, leafMaxXPtr, leafMinYPtr, leafMaxYPtr,
					indicesPtr, colliderOwnerPtr, masks, pushBuffer);
			}
		}

		// Last flush.
		if (pushBuffer.size > 0) pushBuffer.flush();
	}
}