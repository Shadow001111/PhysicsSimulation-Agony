#include "BroadPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include <numeric>
#include <bit>
#include <algorithm>
#include <array>

namespace PS_AGONY
{
    using RealSimd = Simd<Real>;
    using I32Simd = Simd<uint32_t>;
    using U32Simd = Simd<uint32_t>;

    static constexpr uint64_t bvhDepth(uint64_t n, uint64_t leafSize)
    {
        uint64_t d = 0ull;
        while (n > leafSize) { n = (n + 1ull) >> 1ull; ++d; } // Right child = ceil(n/2).
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

    static inline U32Simd part1By1Simd(U32Simd x)
    {
        x &= 0x0000ffffu;
        x = (x | (x << 8)) & 0x00FF00FFu;
        x = (x | (x << 4)) & 0x0F0F0F0Fu;
        x = (x | (x << 2)) & 0x33333333u;
        x = (x | (x << 1)) & 0x55555555u;
        return x;
    }

    static inline U32Simd morton2DSimd(const U32Simd& x, const U32Simd& y)
    {
        return (part1By1Simd(y) << 1) | part1By1Simd(x);
    }


    const std::vector<BodyPair>& BroadPhaseCollisionDetector::findCollisions(const AABBSoAViewer& bodiesAABBViewer)
    {
        TRACY_SCOPE_N("Broad phase");

        bodiesAABB = bodiesAABBViewer;

        broadCollisionData.clear();

        const size_t bodyCount = bodiesAABB.getCount();
        if (bodyCount < 2) return broadCollisionData; // No pairs to check.

        broadCollisionData.reserve(bodyCount);

        findCollisionsBVH(bodyCount);

        return broadCollisionData;
    }

    void BroadPhaseCollisionDetector::fetchAABBs(std::vector<AABB>& outAABBs) const
    {
        // Collect BVH nodes (leafs) AABBs from previous time.
		const auto& nodes = bvhFunctionResources.nodeVector;
		outAABBs.reserve(outAABBs.size() + nodes.size());
        for (const auto& node : nodes)
        {
            if (node.left == BvhNode::INVALID_INDEX && node.right == BvhNode::INVALID_INDEX) // Leaf check.
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
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leafMinX);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leafMaxX);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leafMinY);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.leafMaxY);
        total += PS_AGONY::getVectorMemoryUsage(bvhFunctionResources.bodyIndexVector1);

        total += PS_AGONY::getVectorMemoryUsage(broadCollisionData);

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
            TRACY_SCOPE_N("Build nodes");
            buildBvhTree(nodes, indices, bodyCount);
        }
        {
            TRACY_SCOPE_N("Reorder AABB by indices");
            reorderAABBByIndices(indices);
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

    void BroadPhaseCollisionDetector::computeMortonCodes(uint32_t bodyCount)
    {
        auto& mortonCodes = bvhFunctionResources.mortonCodes;
        mortonCodes.resize(bodyCount);

        MortonCode* CORE_RESTRICT mortonCodePtr = mortonCodes.data();

        const Real* CORE_RESTRICT centroidXPtr = bvhFunctionResources.transformedCentroidX.data();
        const Real* CORE_RESTRICT centroidYPtr = bvhFunctionResources.transformedCentroidY.data();

        TRACY_SCOPE_N("Compute morton codes");

        for (uint32_t b = 0; b < bodyCount; b++)
        {
            const uint32_t qx = static_cast<uint32_t>(centroidXPtr[b]);
            const uint32_t qy = static_cast<uint32_t>(centroidYPtr[b]);
            mortonCodePtr[b] = morton2D(qx, qy);
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
        computeMortonCodes(bodyCount);
        const MortonCode* CORE_RESTRICT mortonCodePtr = bvhFunctionResources.mortonCodes.data();

        // -------------------------------------------------------------------
        // Step 3: Sort body indices by Morton code - single O(N log N) pass.
        //
        // NOTE: a 2-pass 16-bit radix sort over the 32-bit codes would reduce
        //       this to O(N), which is worthwhile for large N.  std::sort is
        //       used here for simplicity; swap if profiling shows it as a
        //       bottleneck.
        // -------------------------------------------------------------------
        {
            TRACY_SCOPE_N("Sort Morton");
            // mortonCodes[b] is the code for body b; indices starts as 0..N-1.
            std::sort(indices.begin(), indices.end(),
                [mortonCodePtr](BodyIndex a, BodyIndex b)
                {
                    return mortonCodePtr[a] < mortonCodePtr[b];
                });
        }

        // -------------------------------------------------------------------
        // Step 4: Top-down tree build with Morton-code binary split.
        //
        // For a node covering sorted range [nodeStart, nodeEnd):
        //   • XOR the first and last Morton codes to find the highest bit
        //     where they differ (the "split bit").
        //   • Binary-search for the boundary between codes that have the
        //     split bit clear (left child) and those that have it set
        //     (right child).
        //   • Fall back to a median split when all codes in the range are
        //     identical (perfectly overlapping bodies).
        //
        // The resulting tree mirrors the Z-order hierarchy, so spatially
        // nearby bodies land in the same subtree - improving query pruning.
        // -------------------------------------------------------------------
        {
            TRACY_SCOPE_N("Build tree");
            struct BuildTask { uint32_t nodeIdx; };

            // LBVH depth bound: up to 32 bit-split levels (one per Morton-code bit)
            // plus bvhDepth() median-fallback levels for same-code body clusters.
            // The old formula (bvhDepth only, ~29) assumed a balanced median-split
            // tree; the LBVH can be far deeper, overflowing that stack.
            constexpr uint64_t MAX_STACK_CAPACITY =
                32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE) + 2ull;
            std::array<BuildTask, MAX_STACK_CAPACITY> stack;
            uint32_t stackSize = 0;

            nodes.emplace_back(0u, bodyCount);
            stack[stackSize++] = { 0u };

            while (stackSize > 0)
            {
                const BuildTask task = stack[--stackSize];
                BvhNode& node = nodes[task.nodeIdx]; // Safe: nodes is reserved in buildBVHTree.

                const uint32_t nodeStart = node.start;
                const uint32_t nodeEnd = node.end;
                const uint32_t rangeSize = nodeEnd - nodeStart;

                // Compute merged AABB for this node.
                Real minX =  std::numeric_limits<Real>::max();
                Real maxX = -std::numeric_limits<Real>::max();
                Real minY =  std::numeric_limits<Real>::max();
                Real maxY = -std::numeric_limits<Real>::max();

                for (uint32_t i = nodeStart; i < nodeEnd; i++)
                {
                    const BodyIndex b = indices[i];
                    minX = std::min(minX, aabbMinXPtr[b]);
                    maxX = std::max(maxX, aabbMaxXPtr[b]);
                    minY = std::min(minY, aabbMinYPtr[b]);
                    maxY = std::max(maxY, aabbMaxYPtr[b]);
                }
                node.minX = minX; node.maxX = maxX;
                node.minY = minY; node.maxY = maxY;

                if (rangeSize <= BvhNode::KD_LEAF_SIZE)
                    continue; // Leaf - nothing more to split.

                // Find the split position.
                const uint32_t mcFirst = mortonCodePtr[indices[nodeStart]];
                const uint32_t mcLast = mortonCodePtr[indices[nodeEnd - 1]];

                uint32_t mid;
                if (mcFirst == mcLast)
                {
                    // All bodies hash to the same Morton cell; equal codes can't
                    // be meaningfully split, so fall back to a balanced median.
                    mid = nodeStart + rangeSize / 2;
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
                const uint32_t rightIdx = leftIdx + 1u;
                node.left = leftIdx;
                node.right = rightIdx;

                nodes.emplace_back(nodeStart, mid);
                nodes.emplace_back(mid, nodeEnd);

                // Push right before left so left is processed first (depth-first).
                stack[stackSize++] = { rightIdx };
                stack[stackSize++] = { leftIdx };
            }
        }
    }

    void BroadPhaseCollisionDetector::reorderAABBByIndices(const std::vector<BodyIndex>& indices)
    {
        const size_t bodyCount = indices.size();

		Real* CORE_RESTRICT leafMinXPtr = nullptr;
		Real* CORE_RESTRICT leafMaxXPtr = nullptr;
		Real* CORE_RESTRICT leafMinYPtr = nullptr;
		Real* CORE_RESTRICT leafMaxYPtr = nullptr;

        {
            auto& leafMinX = bvhFunctionResources.leafMinX;
            auto& leafMaxX = bvhFunctionResources.leafMaxX;
            auto& leafMinY = bvhFunctionResources.leafMinY;
            auto& leafMaxY = bvhFunctionResources.leafMaxY;

            leafMinX.resize(bodyCount);
            leafMaxX.resize(bodyCount);
            leafMinY.resize(bodyCount);
            leafMaxY.resize(bodyCount);

			leafMinXPtr = leafMinX.data();
			leafMaxXPtr = leafMaxX.data();
            leafMinYPtr = leafMinY.data();
			leafMaxYPtr = leafMaxY.data();
        }

        const Real* CORE_RESTRICT srcMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT srcMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT srcMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT srcMaxYPtr = bodiesAABB.maxY;

        for (size_t i = 0; i < bodyCount; i++)
        {
            const BodyIndex b = indices[i];
            leafMinXPtr[i] = srcMinXPtr[b];
            leafMaxXPtr[i] = srcMaxXPtr[b];
            leafMinYPtr[i] = srcMinYPtr[b];
            leafMaxYPtr[i] = srcMaxYPtr[b];
        }
    }

    void BroadPhaseCollisionDetector::queryBvhPairs(const std::vector<BvhNode>& nodes, const std::vector<BodyIndex>& indices)
    {
        const Real* leafMinXPtr = bvhFunctionResources.leafMinX.data();
        const Real* leafMaxXPtr = bvhFunctionResources.leafMaxX.data();
        const Real* leafMinYPtr = bvhFunctionResources.leafMinY.data();
        const Real* leafMaxYPtr = bvhFunctionResources.leafMaxY.data();

        // Dual-node traversal stack.
        // Self-query pushes up to 3 items per pop (net +2), so worst-case depth
        // for a tree of depth D is 2D+1 items.  LBVH depth = 32 bit-split levels
        // + bvhDepth median-fallback levels (~61 total), so we need ~123 entries.
        // The old value (2*bvhDepth+1 ≈ 59) was sized for a balanced median tree
        // only and would overflow for a deep LBVH.
        constexpr uint64_t MAX_STACK_CAPACITY =
            2ull * (32ull + bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE)) + 1ull;
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

            const bool aLeaf = (nodeA.left == BvhNode::INVALID_INDEX);
            const bool bLeaf = (nodeB.left == BvhNode::INVALID_INDEX);

            if (aLeaf && bLeaf)
            {
                constexpr uint32_t LANES = RealSimd::lanes;
                constexpr uint32_t CAP = BvhNode::KD_LEAF_SIZE;

                constexpr uint32_t ALL_LANES_MASK = (1u << LANES) - 1u;

                // Returns which lanes in the chunk starting at j_base represent a j > i.
                // Bit k is set iff (j_base + k) > i.
                //
                //  i < j_base          -> whole chunk is strictly above i -> all bits set.
                //  i >= j_base + LANES -> whole chunk is at or below i  -> no bits set.
                //  otherwise           -> i falls inside the chunk; keep only k > (i - j_base).
                const auto upperTriMask = [&](uint32_t i, uint32_t j_base) -> uint32_t
                    {
                        if (i < j_base)          return ALL_LANES_MASK;
                        if (i >= j_base + LANES) return 0u;
                        const uint32_t offset = i - j_base; // 0 .. LANES-1
                        return (ALL_LANES_MASK << (offset + 1)) & ALL_LANES_MASK;
                    };

                const auto gatherLeaf = [&](LeafAABB& out, uint32_t nodeStart, uint32_t nodeEnd)
                    {
                        const uint32_t count = nodeEnd - nodeStart;

                        for (uint32_t k = 0; k < count; k++)
                        {
                            out.minX[k] = leafMinXPtr[nodeStart + k];
                            out.maxX[k] = leafMaxXPtr[nodeStart + k];
                            out.minY[k] = leafMinYPtr[nodeStart + k];
                            out.maxY[k] = leafMaxYPtr[nodeStart + k];
                        }

                        constexpr Real DEAD = -std::numeric_limits<Real>::max();
                        for (uint32_t k = count; k < CAP; k++)
                            out.minX[k] = out.maxX[k] = out.minY[k] = out.maxY[k] = DEAD;
						return count;
                    };

                // Gather leaf A, it is needed in both paths.
                LeafAABB leafA;
                const uint32_t countA = gatherLeaf(leafA, nodeA.start, nodeA.end);

                if (nodePair.a == nodePair.b)
                {
                    // Self-query: emit upper-triangle pairs only.
                    for (uint32_t i = 0; i < countA; i++)
                    {
                        const RealSimd vMinXi(leafA.minX[i]);
                        const RealSimd vMaxXi(leafA.maxX[i]);
                        const RealSimd vMinYi(leafA.minY[i]);
                        const RealSimd vMaxYi(leafA.maxY[i]);

                        for (uint32_t j = 0; j < CAP; j += LANES)
                        {
                            const RealSimd vMinXj = RealSimd::load(leafA.minX + j);
                            const RealSimd vMaxXj = RealSimd::load(leafA.maxX + j);
                            const RealSimd vMinYj = RealSimd::load(leafA.minY + j);
                            const RealSimd vMaxYj = RealSimd::load(leafA.maxY + j);

                            const RealSimd overlap =
                                (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                                (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                            uint32_t mask = overlap.movemask() & upperTriMask(i, j);
                            while (mask)
                            {
                                const uint32_t lane = std::countr_zero(mask);
                                mask &= mask - 1; // Clear lowest set bit.
                                broadCollisionData.emplace_back(
                                    indices[nodeA.start + i],
                                    indices[nodeA.start + j + lane]);
                            }
                        }
                    }
                }
                else
                {
                    // Cross-query: all pairs between two distinct leaves.
                    LeafAABB leafB;
                    gatherLeaf(leafB, nodeB.start, nodeB.end);

                    for (uint32_t i = 0; i < countA; i++)
                    {
                        const RealSimd vMinXi(leafA.minX[i]);
                        const RealSimd vMaxXi(leafA.maxX[i]);
                        const RealSimd vMinYi(leafA.minY[i]);
                        const RealSimd vMaxYi(leafA.maxY[i]);

                        for (uint32_t j = 0; j < CAP; j += LANES)
                        {
                            const RealSimd vMinXj = RealSimd::load(leafB.minX + j);
                            const RealSimd vMaxXj = RealSimd::load(leafB.maxX + j);
                            const RealSimd vMinYj = RealSimd::load(leafB.minY + j);
                            const RealSimd vMaxYj = RealSimd::load(leafB.maxY + j);

                            const RealSimd overlap =
                                (vMinXi < vMaxXj) & (vMaxXi > vMinXj) &
                                (vMinYi < vMaxYj) & (vMaxYi > vMinYj);

                            uint32_t mask = overlap.movemask();
                            while (mask)
                            {
                                const uint32_t lane = std::countr_zero(mask);
                                mask &= mask - 1;
                                broadCollisionData.emplace_back(
                                    indices[nodeA.start + i],
                                    indices[nodeB.start + j + lane]);
                            }
                        }
                    }
                }
            }
            else if (nodePair.a == nodePair.b)
            {
                // Self-query internal node.
                const uint32_t L = nodeA.left, R = nodeA.right;
                stack[stackSize++] = { R, R };
                stack[stackSize++] = { L, R };
                stack[stackSize++] = { L, L };
            }
            else if (aLeaf || (!bLeaf && (nodeA.end - nodeA.start) < (nodeB.end - nodeB.start)))
            {
                // Split the larger node B.
                stack[stackSize++] = { nodePair.a, nodeB.right };
                stack[stackSize++] = { nodePair.a, nodeB.left  };
            }
            else
            {
                // Split node A.
                stack[stackSize++] = { nodeA.right, nodePair.b };
                stack[stackSize++] = { nodeA.left,  nodePair.b };
            }
        }
    }
}
