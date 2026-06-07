#include "BroadPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"
#include "Core/Assert.h"

#include <numeric>
#include <bit>
#include <algorithm>
#include <array>
#include <iostream>

namespace PS_AGONY
{
    using RealSimd = Simd<Real>;
    using I32Simd = Simd<uint32_t>;
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

    const std::vector<BodyPair>& BroadPhaseCollisionDetector::findCollisions(const AABBSoAViewer& bodiesAABBViewer)
    {
        TRACY_SCOPE_N("Broad phase");

        bodiesAABB = bodiesAABBViewer;

        collisionData.clear();

        const size_t bodyCount = bodiesAABB.getCount();
        if (bodyCount < 2) return collisionData; // No pairs to check.

        collisionData.reserve(bodyCount);

        findCollisionsBVH(bodyCount);

        return collisionData;
    }

    void BroadPhaseCollisionDetector::fetchAABBs(std::vector<AABB>& outAABBs) const
    {
        // Collect BVH nodes (leafs) AABBs from previous time.
		const auto& nodes = bvhFunctionResources.nodeVector;
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

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.nodeVector);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.transformedCentroidX);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.transformedCentroidY);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.mortonCodes);

        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.bodyIndexVector1);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.bodyIndexVector2);

        total += PS_AGONY::getVectorMemoryUsage(bvhNodeLeafIndices);

        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.minX);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.maxX);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.minY);
        total += PS_AGONY::getVectorMemoryUsage(leafBodyAABBs.maxY);

        total += PS_AGONY::getVectorMemoryUsage(collisionData);

        return total;
    }

    void BroadPhaseCollisionDetector::findCollisionsBVH(size_t bodyCount)
    {
        TRACY_SCOPE_N("BVH");

        auto& nodes = bvhFunctionResources.nodeVector;
        auto& indices = bvhFunctionResources.bodyIndexVector1;

        nodes.clear();
        nodes.reserve(2 * bodyCount);

        indices.resize(bodyCount);
        std::iota(indices.begin(), indices.end(), 0);

        {
            TRACY_SCOPE_N("Build tree");
            buildBvhTree(nodes, indices, bodyCount);
        }
        {
            TRACY_SCOPE_N("Collect leaves AABBs");
            collectLeavesAABBs(indices);
        }
        {
            TRACY_SCOPE_N("Query pairs");
            queryBvhPairs(nodes, indices);
        }
    }

    void BroadPhaseCollisionDetector::computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax)
    {
        // Get pointers.
        const Real* CORE_RESTRICT aabbMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT aabbMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT aabbMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT aabbMaxYPtr = bodiesAABB.maxY;

        Real* CORE_RESTRICT centroidXPtr = nullptr;
        Real* CORE_RESTRICT centroidYPtr = nullptr;
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

        auto& mortonCodes = bvhFunctionResources.mortonCodes;
        mortonCodes.resize(bodyCount);

        MortonCode* CORE_RESTRICT mortonCodePtr = mortonCodes.data();

        const TReal* CORE_RESTRICT centroidXPtr = bvhFunctionResources.transformedCentroidX.data();
        const TReal* CORE_RESTRICT centroidYPtr = bvhFunctionResources.transformedCentroidY.data();

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
        TRACY_SCOPE_N("Sort Morton");

        constexpr uint32_t RADIX_BITS = 8;
        constexpr uint32_t RADIX_SIZE = 1u << RADIX_BITS;
        constexpr uint32_t RADIX_MASK = RADIX_SIZE - 1u;

        const MortonCode* CORE_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

        std::array<uint32_t, RADIX_SIZE> count;

        auto radixPass = [&](uint32_t shift, const BodyIndex* CORE_RESTRICT src, BodyIndex* CORE_RESTRICT dst)
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

        //
        auto& temp = bvhFunctionResources.bodyIndexVector2;
        temp.resize(bodyCount);

        {
            BodyIndex* CORE_RESTRICT indexPtr = bvhFunctionResources.bodyIndexVector1.data();
            BodyIndex* CORE_RESTRICT indexTempPtr = temp.data();

            radixPass(0, indexPtr, indexTempPtr);
            radixPass(8, indexTempPtr, indexPtr);
            radixPass(16, indexPtr, indexTempPtr);
            radixPass(24, indexTempPtr, indexPtr);
        }
    }

    void BroadPhaseCollisionDetector::buildBvhTree(
        std::vector<BvhNode>& nodes,
        std::vector<BodyIndex>& indices,
        const uint32_t bodyCount
    )
    {
        const Real* CORE_RESTRICT aabbMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT aabbMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT aabbMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT aabbMaxYPtr = bodiesAABB.maxY;

        //
        bvhNodeLeafIndices.clear();

        // Compute world AABB.
        Real globalMinX, globalMaxX, globalMinY, globalMaxY;
        {
            TRACY_SCOPE_N("World AABB");
            globalMinX =  std::numeric_limits<Real>::max();
            globalMaxX = -std::numeric_limits<Real>::max();
            globalMinY =  std::numeric_limits<Real>::max();
            globalMaxY = -std::numeric_limits<Real>::max();

            for (uint32_t i = 0; i < bodyCount; i++)
            {
                globalMinX = std::min(globalMinX, aabbMinXPtr[i]);
                globalMaxX = std::max(globalMaxX, aabbMaxXPtr[i]);
                globalMinY = std::min(globalMinY, aabbMinYPtr[i]);
                globalMaxY = std::max(globalMaxY, aabbMaxYPtr[i]);
            }

            // Prevent division by zero for degenerate scenes.
            constexpr Real kEps = Real(1e-5);
            globalMaxX = std::max(globalMaxX, globalMinX + kEps);
            globalMaxY = std::max(globalMaxY, globalMinY + kEps);
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
        const MortonCode* CORE_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

        // Top-down tree build with Morton-code binary split.
        // For a node covering sorted range [nodeStart, nodeEnd):
        //   • XOR the first and last Morton codes to find the highest bit
        //     where they differ (the "split bit").
        //   • Binary-search for the boundary between codes that have the
        //     split bit clear (left child) and those that have it set
        //     (right child).
        //   • Fall back to a median split when all codes in the range are
        //     identical (perfectly overlapping bodies).
        {
            TRACY_SCOPE_N("Stack loop");
            struct BuildTask { uint32_t nodeIdx; };

            // LBVH depth bound: up to 32 bit-split levels (one per Morton-code bit)
            // plus bvhDepth() median-fallback levels for same-code body clusters.
            constexpr uint64_t MAX_STACK_CAPACITY =
                32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE) + 2ull;
            std::array<BuildTask, MAX_STACK_CAPACITY> stack;
            uint32_t stackSize = 0;

            nodes.emplace_back(0u, bodyCount);
            stack[stackSize++] = { 0u };

            while (stackSize > 0)
            {
                const BuildTask task = stack[--stackSize];
                BvhNode& node = nodes[task.nodeIdx];

                const uint32_t nodeStart = node.start;
                const uint32_t nodeEnd = node.end;
                const uint32_t rangeSize = nodeEnd - nodeStart;

                if (rangeSize <= BvhNode::KD_LEAF_SIZE)
                {
                    // Leaf - nothing more to split.
                    node.leafIndex = bvhNodeLeafIndices.size();
                    bvhNodeLeafIndices.push_back(task.nodeIdx);
                    continue;
                }

                // Find the split position.
                const uint32_t mcFirst = mortonCodePtr[indices[nodeStart]];
                const uint32_t mcLast = mortonCodePtr[indices[nodeEnd - 1]];

                uint32_t mid;
                if (mcFirst == mcLast)
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
                        if ((mortonCodePtr[indices[m]] & splitBit) == 0u)
                            lo = m + 1;
                        else
                            hi = m;
                    }

                    // Clamp defensively to guarantee non-empty children.
                    mid = std::clamp(lo, nodeStart + 1u, nodeEnd - 1u);
                }

                const uint32_t leftIdx = static_cast<uint32_t>(nodes.size());
                node.leftChildIndex = leftIdx;

                stack[stackSize++] = { leftIdx + 1 };
                stack[stackSize++] = { leftIdx };

                nodes.emplace_back(nodeStart, mid);
                nodes.emplace_back(mid, nodeEnd);
            }
        }
        {
            TRACY_SCOPE_N("Compute bvh node AABBs");

            const size_t nodeCount = nodes.size();
            for (size_t idx = nodeCount; idx-- > 0; ) // Reverse order.
            {
                BvhNode& node = nodes[idx];
                if (node.leftChildIndex == BvhNode::INVALID_INDEX)
                {
                    // Leaf: compute AABB from its bodies.
                    Real minX =  std::numeric_limits<Real>::max();
                    Real maxX = -std::numeric_limits<Real>::max();
                    Real minY =  std::numeric_limits<Real>::max();
                    Real maxY = -std::numeric_limits<Real>::max();

                    for (uint32_t i = node.start; i < node.end; ++i)
                    {
                        const BodyIndex b = indices[i];
                        minX = std::min(minX, aabbMinXPtr[b]);
                        maxX = std::max(maxX, aabbMaxXPtr[b]);
                        minY = std::min(minY, aabbMinYPtr[b]);
                        maxY = std::max(maxY, aabbMaxYPtr[b]);
                    }
                    node.minX = minX; node.maxX = maxX;
                    node.minY = minY; node.maxY = maxY;
                }
                else
                {
                    // Not leaf: compute AABB from its children.
                    const BvhNode& left  = nodes[node.leftChildIndex    ];
                    const BvhNode& right = nodes[node.leftChildIndex + 1];
                    node.minX = std::min(left.minX, right.minX);
                    node.maxX = std::max(left.maxX, right.maxX);
                    node.minY = std::min(left.minY, right.minY);
                    node.maxY = std::max(left.maxY, right.maxY);
                }
            }
        }
    }

    void BroadPhaseCollisionDetector::collectLeavesAABBs(const std::vector<BodyIndex>& indices)
    {
        constexpr Real DEAD = -std::numeric_limits<Real>::max();

        const size_t leafCount = bvhNodeLeafIndices.size();

        leafBodyAABBs.minX.resize(leafCount);
        leafBodyAABBs.maxX.resize(leafCount);
        leafBodyAABBs.minY.resize(leafCount);
        leafBodyAABBs.maxY.resize(leafCount);

        const Real* CORE_RESTRICT bodyMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT bodyMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT bodyMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT bodyMaxYPtr = bodiesAABB.maxY;

        for (size_t leafIndex = 0; leafIndex < leafCount; leafIndex++)
        {
            const uint32_t nodeIndex = bvhNodeLeafIndices[leafIndex];
            const BvhNode& node = bvhFunctionResources.nodeVector[nodeIndex];

            const uint32_t nodeStart = node.start;
            const uint32_t nodeRange = node.end - nodeStart;

            Real* CORE_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data() + leafIndex);
            Real* CORE_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data() + leafIndex);
            Real* CORE_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data() + leafIndex);
            Real* CORE_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data() + leafIndex);

            for (uint32_t leafBodyIndex = 0; leafBodyIndex < nodeRange; leafBodyIndex++)
            {
                const BodyIndex bodyIndex = indices[nodeStart + leafBodyIndex];

                leafMinXPtr[leafBodyIndex] = bodyMinXPtr[bodyIndex];
                leafMaxXPtr[leafBodyIndex] = bodyMaxXPtr[bodyIndex];
                leafMinYPtr[leafBodyIndex] = bodyMinYPtr[bodyIndex];
                leafMaxYPtr[leafBodyIndex] = bodyMaxYPtr[bodyIndex];
            }
            for (uint32_t leafBodyIndex = nodeRange; leafBodyIndex < BvhNode::KD_LEAF_SIZE; leafBodyIndex++)
            {
                leafMinXPtr[leafBodyIndex] = DEAD;
                leafMaxXPtr[leafBodyIndex] = DEAD;
                leafMinYPtr[leafBodyIndex] = DEAD;
                leafMaxYPtr[leafBodyIndex] = DEAD;
            }
        }
    }

    void BroadPhaseCollisionDetector::queryBvhPairs(const std::vector<BvhNode>& nodes, const std::vector<BodyIndex>& indices)
    {
        constexpr uint32_t LANES = RealSimd::lanes;
        constexpr uint32_t LANES_LOG2 = integralLog2(LANES);

        constexpr uint32_t CAP = BvhNode::KD_LEAF_SIZE;

        constexpr auto maskArray = makeMaskArray<BvhNode::KD_LEAF_SIZE, CAP / LANES>();
        
        // Get pointers.
        const Real* CORE_RESTRICT leafMinXPtr = reinterpret_cast<Real*>(leafBodyAABBs.minX.data());
        const Real* CORE_RESTRICT leafMaxXPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxX.data());
        const Real* CORE_RESTRICT leafMinYPtr = reinterpret_cast<Real*>(leafBodyAABBs.minY.data());
        const Real* CORE_RESTRICT leafMaxYPtr = reinterpret_cast<Real*>(leafBodyAABBs.maxY.data());
        const BodyIndex* CORE_RESTRICT indicesPtr = indices.data();

        // Note: I tried to get rid of 'gatherLeaf' lambda and copy data directly instead doing it two times, but it was slower. Why? :C
        // I guess we are trading small copy overhead for cache efficiency.
        const auto gatherLeaf = [&leafMinXPtr, &leafMaxXPtr, &leafMinYPtr, &leafMaxYPtr](LeafBodyAABBs& out, uint32_t nodeLeafIndex)
            {
                constexpr size_t COPY_SIZE = sizeof(LeafBodyAABBSoA::LeafData);

                const size_t srcIndex = nodeLeafIndex * BvhNode::KD_LEAF_SIZE;
                std::memcpy(out.minX, leafMinXPtr + srcIndex, COPY_SIZE);
                std::memcpy(out.maxX, leafMaxXPtr + srcIndex, COPY_SIZE);
                std::memcpy(out.minY, leafMinYPtr + srcIndex, COPY_SIZE);
                std::memcpy(out.maxY, leafMaxYPtr + srcIndex, COPY_SIZE);
            };

        // Dual-node traversal stack.
        constexpr uint64_t MAX_STACK_CAPACITY = 2ull * (32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;
        BvhNodePair stack[MAX_STACK_CAPACITY];
        uint32_t stackSize = 0;

        stack[stackSize++] = { 0, 0 };

        while (stackSize > 0)
        {
            const BvhNodePair nodePair = stack[--stackSize];

            const BvhNode& nodeA = nodes[nodePair.a];
            const BvhNode& nodeB = nodes[nodePair.b];

            // Prune entire subtree pair if their bounding boxes don't overlap.
            if (nodeA.minX >= nodeB.maxX || nodeA.maxX <= nodeB.minX ||
                nodeA.minY >= nodeB.maxY || nodeA.maxY <= nodeB.minY)
                continue;

            const bool aLeaf = nodeA.leftChildIndex == BvhNode::INVALID_INDEX;
            const bool bLeaf = nodeB.leftChildIndex == BvhNode::INVALID_INDEX;

            if (aLeaf && bLeaf)
            {
                // Gather leaf A, it is needed in both paths.
                LeafBodyAABBs leafA;
                gatherLeaf(leafA, nodeA.leafIndex);
                const uint32_t countA = nodeA.end - nodeA.start;

                if (nodePair.a == nodePair.b)
                {
                    // Self-query: emit upper-triangle pairs only.
                    for (uint32_t i = 0; i < countA; i++)
                    {
                        const RealSimd vMinXi(leafA.minX[i]);
                        const RealSimd vMaxXi(leafA.maxX[i]);
                        const RealSimd vMinYi(leafA.minY[i]);
                        const RealSimd vMaxYi(leafA.maxY[i]);

                        auto maskRow = maskArray[i];
                        uint32_t mask = 0;
                        for (uint32_t j = 0; j < CAP; j += LANES)
                        {
                            const RealSimd vMinXj = RealSimd::load(leafA.minX + j);
                            const RealSimd vMaxXj = RealSimd::load(leafA.maxX + j);
                            const RealSimd vMinYj = RealSimd::load(leafA.minY + j);
                            const RealSimd vMaxYj = RealSimd::load(leafA.maxY + j);

                            const RealSimd overlap =
                                (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                                (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                            const uint32_t localMask = overlap.movemask() & maskRow[j >> LANES_LOG2];
                            mask |= localMask << j;
                        }
                        while (mask)
                        {
                            const uint32_t lane = std::countr_zero(mask);
                            mask &= mask - 1; // Clear lowest set bit.
                            collisionData.emplace_back(
                                indicesPtr[nodeA.start + i],
                                indicesPtr[nodeA.start + lane]);
                        }
                    }
                }
                else
                {
                    // Cross-query: all pairs between two distinct leaves.
                    LeafBodyAABBs leafB;
                    gatherLeaf(leafB, nodeB.leafIndex);

                    for (uint32_t i = 0; i < countA; i++)
                    {
                        const RealSimd vMinXi(leafA.minX[i]);
                        const RealSimd vMaxXi(leafA.maxX[i]);
                        const RealSimd vMinYi(leafA.minY[i]);
                        const RealSimd vMaxYi(leafA.maxY[i]);

                        uint32_t mask = 0;
                        for (uint32_t j = 0; j < CAP; j += LANES)
                        {
                            const RealSimd vMinXj = RealSimd::load(leafB.minX + j);
                            const RealSimd vMaxXj = RealSimd::load(leafB.maxX + j);
                            const RealSimd vMinYj = RealSimd::load(leafB.minY + j);
                            const RealSimd vMaxYj = RealSimd::load(leafB.maxY + j);

                            const RealSimd overlap =
                                (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                                (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                            mask |= overlap.movemask() << j;
                        }
                        while (mask)
                        {
                            const uint32_t lane = std::countr_zero(mask);
                            mask &= mask - 1;
                            collisionData.emplace_back(
                                indicesPtr[nodeA.start + i],
                                indicesPtr[nodeB.start + lane]);
                        }
                    }
                }
            }
            else if (nodePair.a == nodePair.b)
            {
                // Self-query internal node.
                const uint32_t L = nodeA.leftChildIndex;
                const uint32_t R = L + 1;
                stack[stackSize++] = { R, R };
                stack[stackSize++] = { L, R };
                stack[stackSize++] = { L, L };
            }
            else
            {
                const uint32_t areaA = nodeA.end - nodeA.start;
                const uint32_t areaB = nodeA.end - nodeA.start;

                //const Real areaA = (nodeA.maxX - nodeA.minX) * (nodeA.maxY - nodeA.minY);
                //const Real areaB = (nodeB.maxX - nodeB.minX) * (nodeB.maxY - nodeB.minY);

                const bool splitB = aLeaf || (!bLeaf && (areaB > areaA));
                if (splitB)
                {
                    // Split node B.
                    stack[stackSize++] = { nodePair.a, nodeB.leftChildIndex + 1 };
                    stack[stackSize++] = { nodePair.a, nodeB.leftChildIndex };
                }
                else
                {
                    // Split node A.
                    stack[stackSize++] = { nodeA.leftChildIndex + 1, nodePair.b };
                    stack[stackSize++] = { nodeA.leftChildIndex,  nodePair.b };
                }
            }
        }
    }
}
