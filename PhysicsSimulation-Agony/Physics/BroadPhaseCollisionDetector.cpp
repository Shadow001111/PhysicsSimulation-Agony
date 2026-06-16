#include "BroadPhaseCollisionDetector.h"

#include "EcstasyCore/Portablity.h"

#include <numeric>
#include <bit>
#include <algorithm>
#include <array>

namespace PS_AGONY
{
    using RealSimd = Simd<Real>;
    using MortonU32Simd = Simd<uint32_t>;

    static constexpr uint64_t integralLog2(uint64_t n)
    {
        uint64_t d = 0ull;
        while (n > 1) { n = (n + 1ull) >> 1ull; d++; }
        return d;
    }

    static constexpr uint64_t bvhDepth(uint64_t n, uint64_t leafSize)
    {
        uint64_t d = 0ull;
        while (n > leafSize) { n = (n + 1ull) >> 1ull; d++; }
        return d;
    }

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
        x &= 0x0000ffffu;
        x = (x | (x << 8)) & 0x00FF00FFu;
        x = (x | (x << 4)) & 0x0F0F0F0Fu;
        x = (x | (x << 2)) & 0x33333333u;
        x = (x | (x << 1)) & 0x55555555u;
        return x;
    }

    static inline MortonU32Simd morton2DSimd(const MortonU32Simd& x, const MortonU32Simd& y)
    {
        return (part1By1Simd(y) << 1) | part1By1Simd(x);
    }

    static constexpr uint32_t upperTriMask(uint32_t i, uint32_t j_base, uint32_t LANES, uint32_t ALL_LANES_MASK)
    {
        if (i < j_base)          return ALL_LANES_MASK;
        if (i >= j_base + LANES) return 0u;
        const uint32_t offset = i - j_base; // 0 .. LANES-1.
        return (ALL_LANES_MASK << (offset + 1)) & ALL_LANES_MASK;
    };

    template<uint32_t W, uint32_t H>
    constexpr auto makeMaskArray()
    {
        constexpr uint32_t LANES = RealSimd::lanes;
        constexpr uint32_t ALL_LANES_MASK = (1u << LANES) - 1u;
        constexpr uint32_t LANES_LOG2 = integralLog2(LANES);

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



    void BroadPhaseCollisionDetector::setDataViewers(const AABBSoAViewer& aabbs)
    {
        bodiesAABB = aabbs;
    }

    const std::vector<BodyPair>& BroadPhaseCollisionDetector::findCollisions(bool rebuild)
    {
        TRACY_SCOPE_N("Broad phase");

        collisionData.clear();

        const size_t bodyCount = bodiesAABB.getCount();
        if (bodyCount < 2) return collisionData; // No pairs to check.

        collisionData.reserve(bodyCount);

        if (rebuild)
        {
            auto& nodes = bvhFunctionResources.nodes;
            auto& indices = bvhFunctionResources.mainBodyIndices;
            nodes.clear();
            nodes.reserve(2 * bodyCount);

            indices.resize(bodyCount);
            std::iota(indices.begin(), indices.end(), 0);

            buildBvhTree(bodyCount);
        }
        else
        {
            TRACY_SCOPE_N("Refit tree");

            refitBvhNodeAABBS();
        }

        const bool useThreading = USE_THREADING;
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

    void BroadPhaseCollisionDetector::fetchAABBs(std::vector<AABB>& outAABBs) const
    {
        // Collect BVH nodes (leafs) AABBs from previous time.
		const auto& nodes = bvhFunctionResources.nodes;
		outAABBs.reserve(outAABBs.size() + nodes.size());
        for (const auto& node : nodes)
        {
            if (node.leftChildIndex == BvhNode::INVALID_INDEX) // Leaf check.
            {
                outAABBs.push_back({ node.minX, node.minY, node.maxX, node.maxY });
            }
        }
    }

    size_t BroadPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = 0;

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.nodes);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.transformedCentroidX);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.transformedCentroidY);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.mortonCodes);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.mainBodyIndices);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.tempBodyIndicesToSort);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.nodePairsToTraverse);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leafPairsToTestCollisions);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leavesToTestCollisions);

        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.minX);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.maxX);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.minY);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.maxY);

        total += PS_AGONY::getVectorMemoryUsage(collisionData);
        return total;
    }

    void BroadPhaseCollisionDetector::computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax)
    {
        // Get pointers.
        const Real* ECSTASY_RESTRICT aabbMinXPtr = bodiesAABB.minX;
        const Real* ECSTASY_RESTRICT aabbMinYPtr = bodiesAABB.minY;
        const Real* ECSTASY_RESTRICT aabbMaxXPtr = bodiesAABB.maxX;
        const Real* ECSTASY_RESTRICT aabbMaxYPtr = bodiesAABB.maxY;

        Real* ECSTASY_RESTRICT centroidXPtr = nullptr;
        Real* ECSTASY_RESTRICT centroidYPtr = nullptr;
        {
            auto& centroidX = bvhFunctionResources.transformedCentroidX;
            auto& centroidY = bvhFunctionResources.transformedCentroidY;
            centroidX.resize(bodyCount);
            centroidY.resize(bodyCount);
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
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            const RealSimd minX = RealSimd::load(aabbMinXPtr + i);
            const RealSimd maxX = RealSimd::load(aabbMaxXPtr + i);
            const RealSimd minY = RealSimd::load(aabbMinYPtr + i);
            const RealSimd maxY = RealSimd::load(aabbMaxYPtr + i);

            // Note: Optimization here makes my deterministic simulation make different results. Rounding probably. I hope it doesn't slow down simulation.
            // t = ((min + max) * 0.5 - globalMin) * scale
            // t = (min + max) * 0.5 * scale - globalMin * scale
            // t = (min + max) * halfScale - scaledGlobalMin
            const RealSimd tx = RealSimd::mul_sub(minX + maxX, halfScaleXV, scaledGlobalMinXV);
            const RealSimd ty = RealSimd::mul_sub(minY + maxY, halfScaleYV, scaledGlobalMinYV);

            const RealSimd clampedX = RealSimd::clamp(tx, RealSimd(0), clampMaxV);
            const RealSimd clampedY = RealSimd::clamp(ty, RealSimd(0), clampMaxV);

            clampedX.store(centroidXPtr + i);
            clampedY.store(centroidYPtr + i);
        }
        for (; i < bodyCount; i++)
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
    void BroadPhaseCollisionDetector::computeMortonCodes(uint32_t bodyCount)
    {
        using TRealSimd = Simd<TReal>;

        bvhFunctionResources.mortonCodes.resize(bodyCount);

        MortonCode* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

        const TReal* ECSTASY_RESTRICT centroidXPtr = bvhFunctionResources.transformedCentroidX.data();
        const TReal* ECSTASY_RESTRICT centroidYPtr = bvhFunctionResources.transformedCentroidY.data();

        TRACY_SCOPE_N("Compute morton codes");

        size_t i = 0;
        if constexpr (TRealSimd::lanes == MortonU32Simd::lanes)
        {   // SIMD PATH.
            for (; i + RealSimd::lanes <= bodyCount; i += TRealSimd::lanes)
            {
                const TRealSimd cx = TRealSimd::load(centroidXPtr + i);
                const TRealSimd cy = TRealSimd::load(centroidYPtr + i);

                const MortonU32Simd qx = cx.to_uint32();
                const MortonU32Simd qy = cy.to_uint32();

                const MortonU32Simd code = morton2DSimd(qx, qy);

                code.store(mortonCodePtr + i);
            }
        }
        for (; i < bodyCount; i++)
        {
            const uint32_t qx = static_cast<uint32_t>(centroidXPtr[i]);
            const uint32_t qy = static_cast<uint32_t>(centroidYPtr[i]);
            mortonCodePtr[i] = morton2D(qx, qy);
        }
    }

    template void BroadPhaseCollisionDetector::computeMortonCodes<Real>(uint32_t);

    void BroadPhaseCollisionDetector::sortBodyIndicesByMortonCodes(uint32_t bodyCount)
    {
        bvhFunctionResources.tempBodyIndicesToSort.resize(bodyCount);

        TRACY_SCOPE_N("Sort indices by morton codes");

        constexpr uint32_t RADIX_BITS = 8;
        constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
        constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

        const MortonCode* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

        alignas(64) std::array<uint32_t, RADIX_SIZE> count;

        auto radixPass = [&](uint32_t shift, const BodyIndex* ECSTASY_RESTRICT src, BodyIndex* ECSTASY_RESTRICT dst)
            {
                count.fill(0);

                // Count buckets.
                for (uint32_t i = 0; i < bodyCount; i++)
                {
                    const BodyIndex idx = src[i];
                    const uint32_t key = (mortonCodePtr[idx] >> shift) & RADIX_MASK;
                    count[key]++;
                }

                // Exclusive prefix sum.
                uint32_t sum = 0;
                for (uint32_t i = 0; i < RADIX_SIZE; i++)
                {
                    const uint32_t c = count[i];
                    count[i] = sum;
                    sum += c;
                }

                // Scatter (stable).
                for (uint32_t i = 0; i < bodyCount; i++)
                {
                    const BodyIndex idx = src[i];
                    const uint32_t key = (mortonCodePtr[idx] >> shift) & RADIX_MASK;
                    dst[count[key]++] = idx;
                }
            };

        {
            BodyIndex* ECSTASY_RESTRICT indexPtr = bvhFunctionResources.mainBodyIndices.data();
            BodyIndex* ECSTASY_RESTRICT indexTempPtr = bvhFunctionResources.tempBodyIndicesToSort.data();

            radixPass(0,  indexPtr, indexTempPtr);
            radixPass(8,  indexTempPtr, indexPtr);
            radixPass(16, indexPtr, indexTempPtr);
            radixPass(24, indexTempPtr, indexPtr);
        }
    }

    void BroadPhaseCollisionDetector::buildBvhTree(const uint32_t bodyCount)
    {
        TRACY_SCOPE_N("Build tree");

        const Real* ECSTASY_RESTRICT bodyMinXPtr = bodiesAABB.minX;
        const Real* ECSTASY_RESTRICT bodyMaxXPtr = bodiesAABB.maxX;
        const Real* ECSTASY_RESTRICT bodyMinYPtr = bodiesAABB.minY;
        const Real* ECSTASY_RESTRICT bodyMaxYPtr = bodiesAABB.maxY;

        // Compute world AABB.
        Real globalMinX, globalMaxX, globalMinY, globalMaxY;
        {
            TRACY_SCOPE_N("Compute world AABB");
            globalMinX =  std::numeric_limits<Real>::max();
            globalMaxX = -std::numeric_limits<Real>::max();
            globalMinY =  std::numeric_limits<Real>::max();
            globalMaxY = -std::numeric_limits<Real>::max();

            for (uint32_t i = 0; i < bodyCount; i++)
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
            computeCentroidsWithTransformations(bodyCount, { globalMinX, globalMinY }, { scaleX, scaleY }, Real(0xFFFFu));
        }

        // Compute morton codes.
        computeMortonCodes<Real>(bodyCount);

        // Sort indices by morton code.
        sortBodyIndicesByMortonCodes(bodyCount);
        const MortonCode* ECSTASY_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();
        const BodyIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainBodyIndices.data();

        // Top-down tree build with Morton-code binary split.
        // For a node covering sorted range [nodeStart, nodeEnd):
        //   • XOR the first and last Morton codes to find the highest bit
        //     where they differ (the "split bit").
        //   • Binary-search for the boundary between codes that have the
        //     split bit clear (left child) and those that have it set
        //     (right child).
        //   • Fall back to a median split when all codes in the range are
        //     identical (perfectly overlapping bodies).
        size_t leafCount = 0;
        {
            TRACY_SCOPE_N("Stack loop");
            struct BuildTask { uint32_t nodeIdx; };

            // LBVH depth bound: up to 32 bit-split levels (one per Morton-code bit)
            // plus bvhDepth() median-fallback levels for same-code body clusters.
            constexpr uint64_t MAX_STACK_CAPACITY =
                32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE) + 2ull;
            std::array<BuildTask, MAX_STACK_CAPACITY> stack;
            uint32_t stackSize = 0;

            bvhFunctionResources.nodes.emplace_back(0u, bodyCount);
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

                // Find the split position.
                const uint32_t mcFirst = mortonCodePtr[indicesPtr[nodeStart]];
                const uint32_t mcLast = mortonCodePtr[indicesPtr[nodeEnd - 1]];

                uint32_t mid;
                if (mcFirst == mcLast) [[unlikely]]
                {
                    // All bodies hash to the same Morton cell; equal codes can't
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
                        if ((mortonCodePtr[indicesPtr[m]] & splitBit) == 0u)
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
        {
            TRACY_SCOPE_N("Compute bvh node and leaf AABBs");

            leafBodyAABBs.minX.resize(leafCount);
            leafBodyAABBs.maxX.resize(leafCount);
            leafBodyAABBs.minY.resize(leafCount);
            leafBodyAABBs.maxY.resize(leafCount);

            refitBvhNodeAABBS();
        }
    }

    void BroadPhaseCollisionDetector::refitBvhNodeAABBS()
    {
        const BodyIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainBodyIndices.data();

        const Real* ECSTASY_RESTRICT bodyMinXPtr = bodiesAABB.minX;
        const Real* ECSTASY_RESTRICT bodyMaxXPtr = bodiesAABB.maxX;
        const Real* ECSTASY_RESTRICT bodyMinYPtr = bodiesAABB.minY;
        const Real* ECSTASY_RESTRICT bodyMaxYPtr = bodiesAABB.maxY;

        const size_t nodeCount = bvhFunctionResources.nodes.size();
        for (size_t idx = nodeCount; idx-- > 0; ) // Reverse order.
        {
            BvhNode& node = bvhFunctionResources.nodes[idx];
            if (node.leftChildIndex == BvhNode::INVALID_INDEX)
            {
                // Leaf: compute AABB from its bodies.
                constexpr Real DEAD_MAX = -std::numeric_limits<Real>::max();
                constexpr Real DEAD_MIN = std::numeric_limits<Real>::max();

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
                    const BodyIndex bodyIndex = indicesPtr[nodeStart + leafBodyIndex];

                    const Real bodyMinX = bodyMinXPtr[bodyIndex];
                    const Real bodyMaxX = bodyMaxXPtr[bodyIndex];
                    const Real bodyMinY = bodyMinYPtr[bodyIndex];
                    const Real bodyMaxY = bodyMaxYPtr[bodyIndex];

                    leafMinXPtr[leafBodyIndex] = bodyMinX;
                    leafMaxXPtr[leafBodyIndex] = bodyMaxX;
                    leafMinYPtr[leafBodyIndex] = bodyMinY;
                    leafMaxYPtr[leafBodyIndex] = bodyMaxY;

                    minX = std::fmin(minX, bodyMinX);
                    maxX = std::fmax(maxX, bodyMaxX);
                    minY = std::fmin(minY, bodyMinY);
                    maxY = std::fmax(maxY, bodyMaxY);
                }
                for (uint32_t leafBodyIndex = nodeRange; leafBodyIndex < BvhNode::KD_LEAF_SIZE; leafBodyIndex++)
                {
                    leafMinXPtr[leafBodyIndex] = DEAD_MIN;
                    leafMaxXPtr[leafBodyIndex] = DEAD_MAX;
                    leafMinYPtr[leafBodyIndex] = DEAD_MIN;
                    leafMaxYPtr[leafBodyIndex] = DEAD_MAX;
                }
                node.minX = minX; node.maxX = maxX;
                node.minY = minY; node.maxY = maxY;
            }
            else
            {
                // Not leaf: compute AABB from its children.
                const BvhNode& left  = bvhFunctionResources.nodes[node.leftChildIndex];
                const BvhNode& right = bvhFunctionResources.nodes[node.leftChildIndex + 1];
                node.minX = std::fmin(left.minX, right.minX);
                node.maxX = std::fmax(left.maxX, right.maxX);
                node.minY = std::fmin(left.minY, right.minY);
                node.maxY = std::fmax(left.maxY, right.maxY);
            }
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

        constexpr auto WORKER_COUNT = QueryPairsThreadedResources::WORKER_COUNT;

        constexpr uint32_t LANES = RealSimd::lanes;
        constexpr uint32_t LANES_LOG2 = integralLog2(LANES);
        constexpr auto maskArray = makeMaskArray<BvhNode::KD_LEAF_SIZE, BvhNode::KD_LEAF_SIZE / LANES>();

        const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
        const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
        const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
        const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
        const BodyIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainBodyIndices.data();

        // Get thread pool.
        auto& threadPool = Threading::getGlobalThreadPool();

        // Reset workers and their data.
        for (auto& wData : queryPairsThreadedResources.workerData)
        {
            std::lock_guard lock(wData.mutex);
            wData.stopRequested = false;
            wData.finished = false;

            wData.incomingSelfTasks.clear();
            wData.incomingCrossTasks.clear();
            wData.localSelfTasks.clear();
            wData.localCrossTasks.clear();
            wData.outCollisionData.clear();
        }

        // Launch workers.
        auto workerFunc = [&](size_t workerIndex)
            {
                auto& wData = queryPairsThreadedResources.workerData[workerIndex];
                wData.running = false;
                try
                {
                    std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;
                    while (true)
                    {
                        std::unique_lock lk(wData.mutex);
                        {
                            TRACY_SCOPE_NC("Wait for request", Ecstasy::Color::DarkViolet);
                            wData.cv.wait(lk, [&]
                                {
                                    return wData.stopRequested ||
                                        !wData.incomingSelfTasks.empty() ||
                                        !wData.incomingCrossTasks.empty();
                                });
                        }

                        if (wData.stopRequested &&
                            wData.incomingSelfTasks.empty() &&
                            wData.incomingCrossTasks.empty())
                        {
                            break;
                        }

                        wData.running = true;
                        wData.localSelfTasks.swap(wData.incomingSelfTasks);
                        wData.localCrossTasks.swap(wData.incomingCrossTasks);
                        lk.unlock();

                        // Execute tasks.
                        {
                            TRACY_SCOPE_NC("Execute tasks", Ecstasy::Color::Wheat);
                            for (const uint32_t nodeIdx : wData.localSelfTasks)
                            {
                                const BvhNode& node = bvhFunctionResources.nodes[nodeIdx];
                                const size_t srcIndex = node.leafIndex * BvhNode::KD_LEAF_SIZE;
                                const Real* leafMinX = leafMinXPtr + srcIndex;
                                const Real* leafMaxX = leafMaxXPtr + srcIndex;
                                const Real* leafMinY = leafMinYPtr + srcIndex;
                                const Real* leafMaxY = leafMaxYPtr + srcIndex;
                                const uint32_t count = node.end - node.start;

                                for (uint32_t i = 0; i < count; i++)
                                {
                                    const RealSimd vMinXi(leafMinX[i]);
                                    const RealSimd vMaxXi(leafMaxX[i]);
                                    const RealSimd vMinYi(leafMinY[i]);
                                    const RealSimd vMaxYi(leafMaxY[i]);

                                    auto maskRow = maskArray[i];
                                    uint32_t mask = 0;
                                    for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LANES)
                                    {
                                        const RealSimd vMinXj = RealSimd::load(leafMinX + j);
                                        const RealSimd vMaxXj = RealSimd::load(leafMaxX + j);
                                        const RealSimd vMinYj = RealSimd::load(leafMinY + j);
                                        const RealSimd vMaxYj = RealSimd::load(leafMaxY + j);

                                        const RealSimd overlap =
                                            (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                                            (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                                        const uint32_t localMask = overlap.movemask() & maskRow[j >> LANES_LOG2];
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
                                        wData.outCollisionData.emplace_back(
                                            indicesPtr[node.start + i],
                                            indicesPtr[node.start + lane]
                                        );
                                    }
                                }
                            }
                            for (const BvhNodePair nodePair : wData.localCrossTasks)
                            {
                                const uint32_t nodeAIdx = nodePair.a;
                                const uint32_t nodeBIdx = nodePair.b;
                                const BvhNode& nodeA = bvhFunctionResources.nodes[nodeAIdx];
                                const BvhNode& nodeB = bvhFunctionResources.nodes[nodeBIdx];
                                const uint32_t countA = nodeA.end - nodeA.start;

                                const size_t srcIndexA = nodeA.leafIndex * BvhNode::KD_LEAF_SIZE;
                                const Real* leafAMinX = leafMinXPtr + srcIndexA;
                                const Real* leafAMaxX = leafMaxXPtr + srcIndexA;
                                const Real* leafAMinY = leafMinYPtr + srcIndexA;
                                const Real* leafAMaxY = leafMaxYPtr + srcIndexA;

                                const size_t srcIndexB = nodeB.leafIndex * BvhNode::KD_LEAF_SIZE;
                                const Real* leafBMinX = leafMinXPtr + srcIndexB;
                                const Real* leafBMaxX = leafMaxXPtr + srcIndexB;
                                const Real* leafBMinY = leafMinYPtr + srcIndexB;
                                const Real* leafBMaxY = leafMaxYPtr + srcIndexB;

                                for (uint32_t i = 0; i < countA; i++)
                                {
                                    const RealSimd vMinXi(leafAMinX[i]);
                                    const RealSimd vMaxXi(leafAMaxX[i]);
                                    const RealSimd vMinYi(leafAMinY[i]);
                                    const RealSimd vMaxYi(leafAMaxY[i]);

                                    uint32_t mask = 0;
                                    for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LANES)
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
                                        wData.outCollisionData.emplace_back(
                                            indicesPtr[nodeA.start + i],
                                            indicesPtr[nodeB.start + lane]
                                        );
                                    }
                                }
                            }
                        }

                        wData.localSelfTasks.clear();
                        wData.localCrossTasks.clear();

                        lk.lock();
                        wData.running = false;
                        wData.cv.notify_one();
                    }
                }
                catch (...)
                {
                    throw;
                }
                {
                    std::lock_guard lk(wData.mutex);
                    wData.finished = true;
                }
                wData.cv.notify_one();
            };

        for (size_t i = 0; i < WORKER_COUNT; i++)
        {
            threadPool.enqueue(workerFunc, i);
        }

        traverseNodesToGetOverlappingLeafPairsThreaded();

        // Wait for workers to finish and stop them. Combine data.
        {
            TRACY_SCOPE_NC("Wait for workers to finish and combine data", Ecstasy::Color::Brown);
            for (auto& wData : queryPairsThreadedResources.workerData)
            {
                {
                    std::unique_lock lk(wData.mutex);
                    wData.stopRequested = true;
                }
                wData.cv.notify_one();
            }
            for (auto& wData : queryPairsThreadedResources.workerData)
            {
                {
                    TRACY_SCOPE_N("Wait");
                    std::unique_lock lk(wData.mutex);
                    wData.cv.wait(lk, [&]
                        {
                            return wData.finished;
                        });
                }
                {
                    TRACY_SCOPE_N("Combine data");
                    collisionData.insert(
                        collisionData.end(),
                        wData.outCollisionData.begin(),
                        wData.outCollisionData.end()
                    );
                }
            }
        }
    }

    void BroadPhaseCollisionDetector::traverseNodesToGetOverlappingLeafPairs()
    {
        auto overlaps = [](const BvhNode& a, const BvhNode& b) noexcept -> bool
            {
                return
                    a.minX < b.maxX && a.maxX > b.minX &&
                    a.minY < b.maxY && a.maxY > b.minY;
            };

        bvhFunctionResources.nodePairsToTraverse.clear();
        bvhFunctionResources.leafPairsToTestCollisions.clear();
        bvhFunctionResources.leavesToTestCollisions.clear();

        constexpr uint64_t MAX_STACK_CAPACITY = 2ull * (32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;

        // Traverse 1.
        {
            TRACY_SCOPE_N("Traverse 1");

            if (bvhFunctionResources.nodes[0].leftChildIndex == BvhNode::INVALID_INDEX) [[unlikely]] // Root is leaf.
            {
                bvhFunctionResources.leavesToTestCollisions.push_back(0);
            }
            else
            {
                uint32_t bvhNodeIndexStack[MAX_STACK_CAPACITY];
                uint32_t stackSize = 0;
                bvhNodeIndexStack[stackSize++] = { 0 }; // Root index.

                while (stackSize > 0)
                {
                    const uint32_t nodeIndex = bvhNodeIndexStack[--stackSize];

                    const BvhNode& node = bvhFunctionResources.nodes[nodeIndex];

                    const uint32_t L = node.leftChildIndex;
                    const uint32_t R = L + 1;

                    const BvhNode& nodeL = bvhFunctionResources.nodes[L];
                    const BvhNode& nodeR = bvhFunctionResources.nodes[R];

                    const bool lLeaf = nodeL.leftChildIndex == BvhNode::INVALID_INDEX;
                    const bool rLeaf = nodeR.leftChildIndex == BvhNode::INVALID_INDEX;


                    if (rLeaf)
                    {
                        bvhFunctionResources.leavesToTestCollisions.push_back(R);
                    }
                    else
                    {
                        bvhNodeIndexStack[stackSize++] = { R };
                    }
                    if (lLeaf)
                    {
                        bvhFunctionResources.leavesToTestCollisions.push_back(L);
                    }
                    else
                    {
                        bvhNodeIndexStack[stackSize++] = { L };
                    }

                    // Prune L and R nodes.
                    if (overlaps(nodeL, nodeR))
                    {
                        if (lLeaf && rLeaf)
                        {
                            bvhFunctionResources.leafPairsToTestCollisions.emplace_back(L, R);
                        }
                        else
                        {
                            bvhFunctionResources.nodePairsToTraverse.emplace_back(L, R);
                        }
                    }
                }
            }
        }

        // Traverse 2.
        {
            TRACY_SCOPE_N("Traverse 2");

            while (bvhFunctionResources.nodePairsToTraverse.size() > 0)
            {
                const BvhNodePair nodePair = bvhFunctionResources.nodePairsToTraverse.back();
                bvhFunctionResources.nodePairsToTraverse.pop_back();

                const BvhNode& nodeA = bvhFunctionResources.nodes[nodePair.a];
                const BvhNode& nodeB = bvhFunctionResources.nodes[nodePair.b];

                const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
                const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;
                if (aLeaf && bLeaf)
                {
                    bvhFunctionResources.leafPairsToTestCollisions.emplace_back(nodePair);
                    continue;
                }

                const Real nodeAreaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
                const Real nodeAreaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

                const bool splitB = aLeaf || (!bLeaf && (nodeAreaB > nodeAreaA));
                if (splitB)
                {
                    // Split node B: check overlap with each child before pushing.
                    const uint32_t leftChildB = nodeB.leftChildIndex;
                    const uint32_t rightChildB = leftChildB + 1;

                    const BvhNode& leftNodeB = bvhFunctionResources.nodes[leftChildB];
                    const BvhNode& rightNodeB = bvhFunctionResources.nodes[rightChildB];

                    if (overlaps(nodeA, leftNodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(nodePair.a, leftChildB);
                    }

                    if (overlaps(nodeA, rightNodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(nodePair.a, rightChildB);
                    }
                }
                else
                {
                    // Split node A: check overlap with each child before pushing.
                    const uint32_t leftChildA = nodeA.leftChildIndex;
                    const uint32_t rightChildA = leftChildA + 1;

                    const BvhNode& leftNodeA = bvhFunctionResources.nodes[leftChildA];
                    const BvhNode& rightNodeA = bvhFunctionResources.nodes[rightChildA];

                    if (overlaps(leftNodeA, nodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(leftChildA, nodePair.b);
                    }

                    if (overlaps(rightNodeA, nodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(rightChildA, nodePair.b);
                    }
                }
            }
        }
    }

    void BroadPhaseCollisionDetector::traverseNodesToGetOverlappingLeafPairsThreaded()
    {
        auto overlaps = [](const BvhNode& a, const BvhNode& b) noexcept -> bool
            {
                return
                    a.minX < b.maxX && a.maxX > b.minX &&
                    a.minY < b.maxY && a.maxY > b.minY;
            };

        bvhFunctionResources.nodePairsToTraverse.clear();

        constexpr size_t MAX_TRAVERSE_STACK_CAPACITY = 2ull * (32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;

        // Task stacks.
        constexpr size_t TASK_STACK_CAPACITY = 256;

        uint32_t selfTaskStack[TASK_STACK_CAPACITY];
        size_t selfTaskStackSize = 0;

        BvhNodePair crossTaskStack[TASK_STACK_CAPACITY];
        size_t crossTaskStackSize = 0;

        size_t workerIndex = 0;

        auto flushSelfTaskStack = [&]()
            {
                if (selfTaskStackSize == 0) return;
                
                TRACY_SCOPE_N("Flush self task stack");

                {
                    TRACY_SCOPE_N("Push tasks");
                    queryPairsThreadedResources.workerData[workerIndex].pushSelfTasks(selfTaskStack, selfTaskStackSize);
                }
                selfTaskStackSize = 0;

                workerIndex++;
                if (workerIndex >= QueryPairsThreadedResources::WORKER_COUNT)
                    workerIndex = 0;
            };

        auto pushSelfTask = [&](uint32_t task)
            {
                selfTaskStack[selfTaskStackSize++] = task;
                if (selfTaskStackSize >= TASK_STACK_CAPACITY)
                {
                    flushSelfTaskStack();
                }
            };

        auto flushCrossTaskStack = [&]()
            {
                if (crossTaskStackSize == 0) return;

                TRACY_SCOPE_N("Flush cross task stack");

                {
                    TRACY_SCOPE_N("Push tasks");
                    queryPairsThreadedResources.workerData[workerIndex].pushCrossTasks(crossTaskStack, crossTaskStackSize);
                }
                crossTaskStackSize = 0;

                workerIndex++;
                if (workerIndex >= QueryPairsThreadedResources::WORKER_COUNT)
                    workerIndex = 0;
            };

        auto pushCrossTask = [&](BvhNodePair task)
            {
                crossTaskStack[crossTaskStackSize++] = task;
                if (crossTaskStackSize >= TASK_STACK_CAPACITY)
                {
                    flushCrossTaskStack();
                }
            };

        // Traverse 1.
        {
            TRACY_SCOPE_N("Traverse 1");

            if (bvhFunctionResources.nodes[0].leftChildIndex == BvhNode::INVALID_INDEX) [[unlikely]] // Root is leaf.
            {
                pushSelfTask(0);
            }
            else
            {
                uint32_t bvhNodeIndexStack[MAX_TRAVERSE_STACK_CAPACITY];
                uint32_t stackSize = 0;
                bvhNodeIndexStack[stackSize++] = { 0 }; // Root index.

                while (stackSize > 0)
                {
                    const uint32_t nodeIndex = bvhNodeIndexStack[--stackSize];

                    const BvhNode& node = bvhFunctionResources.nodes[nodeIndex];

                    const uint32_t L = node.leftChildIndex;
                    const uint32_t R = L + 1;

                    const BvhNode& nodeL = bvhFunctionResources.nodes[L];
                    const BvhNode& nodeR = bvhFunctionResources.nodes[R];

                    const bool lLeaf = nodeL.leftChildIndex == BvhNode::INVALID_INDEX;
                    const bool rLeaf = nodeR.leftChildIndex == BvhNode::INVALID_INDEX;


                    if (rLeaf)
                    {
                        pushSelfTask(R);
                    }
                    else
                    {
                        bvhNodeIndexStack[stackSize++] = { R };
                    }
                    if (lLeaf)
                    {
                        pushSelfTask(L);
                    }
                    else
                    {
                        bvhNodeIndexStack[stackSize++] = { L };
                    }

                    // Prune L and R nodes.
                    if (overlaps(nodeL, nodeR))
                    {
                        if (lLeaf && rLeaf)
                        {
                            pushCrossTask({ L, R });
                        }
                        else
                        {
                            bvhFunctionResources.nodePairsToTraverse.emplace_back(L, R);
                        }
                    }
                }
            }
        }

        // Flush remaining self tasks.
        flushSelfTaskStack();

        // Traverse 2.
        {
            TRACY_SCOPE_N("Traverse 2");

            while (bvhFunctionResources.nodePairsToTraverse.size() > 0)
            {
                const BvhNodePair nodePair = bvhFunctionResources.nodePairsToTraverse.back();
                bvhFunctionResources.nodePairsToTraverse.pop_back();

                const BvhNode& nodeA = bvhFunctionResources.nodes[nodePair.a];
                const BvhNode& nodeB = bvhFunctionResources.nodes[nodePair.b];

                const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
                const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;
                if (aLeaf && bLeaf)
                {
                    pushCrossTask(nodePair);
                    continue;
                }

                const Real nodeAreaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
                const Real nodeAreaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

                const bool splitB = aLeaf || (!bLeaf && (nodeAreaB > nodeAreaA));
                if (splitB)
                {
                    // Split node B: check overlap with each child before pushing.
                    const uint32_t leftChildB = nodeB.leftChildIndex;
                    const uint32_t rightChildB = leftChildB + 1;

                    const BvhNode& leftNodeB = bvhFunctionResources.nodes[leftChildB];
                    const BvhNode& rightNodeB = bvhFunctionResources.nodes[rightChildB];

                    if (overlaps(nodeA, leftNodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(nodePair.a, leftChildB);
                    }

                    if (overlaps(nodeA, rightNodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(nodePair.a, rightChildB);
                    }
                }
                else
                {
                    // Split node A: check overlap with each child before pushing.
                    const uint32_t leftChildA = nodeA.leftChildIndex;
                    const uint32_t rightChildA = leftChildA + 1;

                    const BvhNode& leftNodeA = bvhFunctionResources.nodes[leftChildA];
                    const BvhNode& rightNodeA = bvhFunctionResources.nodes[rightChildA];

                    if (overlaps(leftNodeA, nodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(leftChildA, nodePair.b);
                    }

                    if (overlaps(rightNodeA, nodeB))
                    {
                        bvhFunctionResources.nodePairsToTraverse.emplace_back(rightChildA, nodePair.b);
                    }
                }
            }
        }

        // Flush remaining cross tasks.
        flushCrossTaskStack();
    }

    void BroadPhaseCollisionDetector::testCollisionsInLeaves()
    {
        TRACY_SCOPE_N("Test collisions in leaves");

        constexpr uint32_t LANES = RealSimd::lanes;
        constexpr uint32_t LANES_LOG2 = integralLog2(LANES);
        constexpr auto maskArray = makeMaskArray<BvhNode::KD_LEAF_SIZE, BvhNode::KD_LEAF_SIZE / LANES>();
        constexpr size_t PUSH_BUFFER_MAX_CAPACITY = 256;

        // Get pointers.
        const Real* ECSTASY_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
        const Real* ECSTASY_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
        const Real* ECSTASY_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
        const Real* ECSTASY_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
        const BodyIndex* ECSTASY_RESTRICT indicesPtr = bvhFunctionResources.mainBodyIndices.data();

        //
        std::array<uint32_t, BvhNode::KD_LEAF_SIZE> masks;

        BodyPair localPushBuffer[PUSH_BUFFER_MAX_CAPACITY];
        uint32_t localPushBufferSize = 0;

        auto flush = [&] {
            collisionData.insert(collisionData.end(), localPushBuffer, localPushBuffer + localPushBufferSize);
            localPushBufferSize = 0;
            };

        // Test self cross.
        {
            TRACY_SCOPE_N("Self cross");
            for (const uint32_t nodeIndex : bvhFunctionResources.leavesToTestCollisions)
            {
                const BvhNode& node = bvhFunctionResources.nodes[nodeIndex];
                const size_t srcIndex = node.leafIndex * BvhNode::KD_LEAF_SIZE;
                const Real* leafMinX = leafMinXPtr + srcIndex;
                const Real* leafMaxX = leafMaxXPtr + srcIndex;
                const Real* leafMinY = leafMinYPtr + srcIndex;
                const Real* leafMaxY = leafMaxYPtr + srcIndex;
                const uint32_t count = node.end - node.start;

                for (uint32_t i = 0; i < count; i++)
                {
                    const RealSimd vMinXi(leafMinX[i]);
                    const RealSimd vMaxXi(leafMaxX[i]);
                    const RealSimd vMinYi(leafMinY[i]);
                    const RealSimd vMaxYi(leafMaxY[i]);

                    auto maskRow = maskArray[i];
                    uint32_t mask = 0;
                    for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LANES)
                    {
                        const RealSimd vMinXj = RealSimd::load(leafMinX + j);
                        const RealSimd vMaxXj = RealSimd::load(leafMaxX + j);
                        const RealSimd vMinYj = RealSimd::load(leafMinY + j);
                        const RealSimd vMaxYj = RealSimd::load(leafMaxY + j);

                        const RealSimd overlap =
                            (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                            (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                        const uint32_t localMask = overlap.movemask() & maskRow[j >> LANES_LOG2];
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
                        localPushBuffer[localPushBufferSize++] = {
                            indicesPtr[node.start + i],
                            indicesPtr[node.start + lane]
                        };
                        if (localPushBufferSize == PUSH_BUFFER_MAX_CAPACITY) flush();
                    }
                }
            }
        }

        // Test AxB cross.
        {
            TRACY_SCOPE_N("AxB cross");
            for (auto [nodeIndexA, nodeIndexB] : bvhFunctionResources.leafPairsToTestCollisions)
            {
                const BvhNode& nodeA = bvhFunctionResources.nodes[nodeIndexA];
                const BvhNode& nodeB = bvhFunctionResources.nodes[nodeIndexB];
                const uint32_t countA = nodeA.end - nodeA.start;

                const size_t srcIndexA = nodeA.leafIndex * BvhNode::KD_LEAF_SIZE;
                const Real* leafAMinX = leafMinXPtr + srcIndexA;
                const Real* leafAMaxX = leafMaxXPtr + srcIndexA;
                const Real* leafAMinY = leafMinYPtr + srcIndexA;
                const Real* leafAMaxY = leafMaxYPtr + srcIndexA;

                const size_t srcIndexB = nodeB.leafIndex * BvhNode::KD_LEAF_SIZE;
                const Real* leafBMinX = leafMinXPtr + srcIndexB;
                const Real* leafBMaxX = leafMaxXPtr + srcIndexB;
                const Real* leafBMinY = leafMinYPtr + srcIndexB;
                const Real* leafBMaxY = leafMaxYPtr + srcIndexB;

                for (uint32_t i = 0; i < countA; i++)
                {
                    const RealSimd vMinXi(leafAMinX[i]);
                    const RealSimd vMaxXi(leafAMaxX[i]);
                    const RealSimd vMinYi(leafAMinY[i]);
                    const RealSimd vMaxYi(leafAMaxY[i]);

                    uint32_t mask = 0;
                    for (uint32_t j = 0; j < BvhNode::KD_LEAF_SIZE; j += LANES)
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
                        localPushBuffer[localPushBufferSize++] = {
                            indicesPtr[nodeA.start + i],
                            indicesPtr[nodeB.start + lane]
                        };
                        if (localPushBufferSize == PUSH_BUFFER_MAX_CAPACITY) flush();
                    }
                }
            }
        }

        // Last flush.
        if (localPushBufferSize > 0) flush();
    }
}
