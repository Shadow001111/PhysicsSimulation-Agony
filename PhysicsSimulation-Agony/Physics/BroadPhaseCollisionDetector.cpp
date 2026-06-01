#include "BroadPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include <numeric>
#include <iostream>
#include <bit>

namespace PS_AGONY
{
    static constexpr uint64_t bvhDepth(uint64_t n, uint64_t leafSize)
    {
        uint64_t d = 0ull;
        while (n > leafSize) { n = (n + 1ull) >> 1ull; ++d; } // Right child = ceil(n/2).
        return d;
    }


    const std::vector<BodyPair>& BroadPhaseCollisionDetector::findCollisions(const AABBSoAViewer& bodiesAABBViewer)
    {
        TRACY_SCOPE_N("Broad phase");

        bodiesAABB = bodiesAABBViewer;

        broadCollisionData.clear();

        const size_t bodyCount = bodiesAABB.getCount();
        if (bodyCount < 2) return broadCollisionData; // No pairs to check.

        broadCollisionData.reserve(bodyCount);

        boundVolumeHierarchy(bodyCount); // The best.

        return broadCollisionData;
    }

    void BroadPhaseCollisionDetector::sweepAndPruneXAxis(size_t bodyCount)
    {
        TRACY_SCOPE_N("Sweep and prune X");

        const Real* CORE_RESTRICT aabbMinX = bodiesAABB.minX;
        const Real* CORE_RESTRICT aabbMinY = bodiesAABB.minY;
        const Real* CORE_RESTRICT aabbMaxX = bodiesAABB.maxX;
        const Real* CORE_RESTRICT aabbMaxY = bodiesAABB.maxY;

        // Build list of body indices sorted by AABB minX.
        auto* CORE_RESTRICT sortedIndices = &functionResources.bodyIndexVector1;
        auto* CORE_RESTRICT activeList = &functionResources.bodyIndexVector2; // Bodies currently overlapping in X.

        sortedIndices->resize(bodyCount);

        std::iota(sortedIndices->begin(), sortedIndices->end(), 0);
        std::sort(sortedIndices->begin(), sortedIndices->end(),
            [&](BodyIndex a, BodyIndex b) {
                return aabbMinX[a] < aabbMinX[b];
            });

        activeList->clear();

        for (BodyIndex current : *sortedIndices)
        {
            const Real minXA = aabbMinX[current];
            const Real minYA = aabbMinY[current];
            const Real maxYA = aabbMaxY[current];

            // Remove from activeList any body whose maxX < current minX.
            // For some reason, this is faster than removing with swap and pop.
            activeList->erase(std::remove_if(activeList->begin(), activeList->end(),
                [&](BodyIndex active) {
                    return aabbMaxX[active] <= minXA;
                }), activeList->end());

            // Check against all active bodies (they overlap in X).
            for (BodyIndex active : *activeList)
            {
                const Real minYB = aabbMinY[active];
                const Real maxYB = aabbMaxY[active];

                const bool doesIntersect = (minYA < maxYB && maxYA > minYB);

                if (doesIntersect)
                {
                    broadCollisionData.emplace_back(current, active);
                }
            }

            activeList->push_back(current);
        }
    }

    void BroadPhaseCollisionDetector::boundVolumeHierarchy(size_t bodyCount)
    {
        TRACY_SCOPE_N("BVH");

        auto& nodes = functionResources.bvhNodeVector1;
        auto& indices = functionResources.bodyIndexVector1;

        nodes.clear();
        nodes.reserve(2 * bodyCount);

        indices.resize(bodyCount);
        std::iota(indices.begin(), indices.end(), 0);

        {
            TRACY_SCOPE_N("Build nodes");
            buildBvhNode(nodes, indices, bodyCount);
        }
        {
            TRACY_SCOPE_N("Query pairs");
            queryBvhPairs(nodes, indices); // Self-query the root finds all pairs.
        }
    }

    //void BroadPhaseCollisionDetector::uniformSpaceGrid(size_t bodyCount)
    //{
    //    TRACY_SCOPE_N("Uniform grid");
    //
    //    const Real* CORE_RESTRICT aabbMinX = bodiesAABB.minX;
    //    const Real* CORE_RESTRICT aabbMinY = bodiesAABB.minY;
    //    const Real* CORE_RESTRICT aabbMaxX = bodiesAABB.maxX;
    //    const Real* CORE_RESTRICT aabbMaxY = bodiesAABB.maxY;
    //
    //    auto* CORE_RESTRICT grid = &functionResources.spaceGrid;
    //
    //    // Compute world bounds from all AABBs.
    //    Real globalMinX =  std::numeric_limits<Real>::max();
    //    Real globalMaxX = -std::numeric_limits<Real>::max();
    //    Real globalMinY =  std::numeric_limits<Real>::max();
    //    Real globalMaxY = -std::numeric_limits<Real>::max();
    //
    //    Real totalExtent = Real(0.0);
    //    {
    //        TRACY_SCOPE_N("Determine world AABB and bodies' total extent");
    //        for (size_t i = 0; i < bodyCount; i++)
    //        {
    //            const Real minX = aabbMinX[i];
    //            const Real maxX = aabbMaxX[i];
    //            const Real minY = aabbMinY[i];
    //            const Real maxY = aabbMaxY[i];
    //
    //            globalMinX = std::min(globalMinX, minX);
    //            globalMaxX = std::max(globalMaxX, maxX);
    //            globalMinY = std::min(globalMinY, minY);
    //            globalMaxY = std::max(globalMaxY, maxY);
    //
    //            const Real extentX = maxX - minX;
    //            const Real extentY = maxY - minY;
    //            totalExtent += std::max(extentX, extentY);
    //        }
    //    }
    //
    //    const Real averageBodyExtent = totalExtent / Real(bodyCount);
    //
    //    const Real worldWidth = std::max(globalMaxX - globalMinX, Real(1e-3));
    //    const Real worldHeight = std::max(globalMaxY - globalMinY, Real(1e-3));
    //    const Real worldArea = worldWidth * worldHeight;
    //
    //    // Determine cell size.
    //    Real cellSize = std::max(
    //        std::sqrt(worldArea / Real(bodyCount)),
    //        averageBodyExtent * Real(2.0)
    //    );
    //    cellSize = std::max(cellSize, Real(0.25));
    //
    //    const Real invCellSize = Real(1.0) / cellSize;
    //
    //    // Grid size.
    //    const int gridWidth = std::max(1, static_cast<int>(std::ceil(worldWidth * invCellSize)));
    //    const int gridHeight = std::max(1, static_cast<int>(std::ceil(worldHeight * invCellSize)));
    //
    //    // Map each occupied cell to a list of body indices.
    //    grid->clear();
    //    grid->reserve(bodyCount * 4);
    //
    //    auto getCellKey = [](int cx, int cy) -> uint64_t {
    //        constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
    //        uint64_t h = (uint64_t)cx + addConst;
    //        h ^= (uint64_t)cy + addConst + (h << 6) + (h >> 2);
    //        return h;
    //        };
    //
    //    {
    //        TRACY_SCOPE_N("Put bodies into cells");
    //        for (BodyIndex bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
    //        {
    //            const Real minX = aabbMinX[bodyIndex];
    //            const Real maxX = aabbMaxX[bodyIndex];
    //            const Real minY = aabbMinY[bodyIndex];
    //            const Real maxY = aabbMaxY[bodyIndex];
    //
    //            int cx0 = static_cast<int>(std::floor((minX - globalMinX) * invCellSize));
    //            int cx1 = static_cast<int>(std::floor((maxX - globalMinX) * invCellSize));
    //            int cy0 = static_cast<int>(std::floor((minY - globalMinY) * invCellSize));
    //            int cy1 = static_cast<int>(std::floor((maxY - globalMinY) * invCellSize));
    //
    //            cx0 = std::clamp(cx0, 0, gridWidth - 1);
    //            cx1 = std::clamp(cx1, 0, gridWidth - 1);
    //            cy0 = std::clamp(cy0, 0, gridHeight - 1);
    //            cy1 = std::clamp(cy1, 0, gridHeight - 1);
    //
    //            for (int cx = cx0; cx <= cx1; cx++)
    //            for (int cy = cy0; cy <= cy1; cy++)
    //                (*grid)[getCellKey(cx, cy)].push_back(bodyIndex);
    //        }
    //    }
    //
    //    // For each cell, test all pairs inside it.
    //    auto* CORE_RESTRICT testedPairs = &functionResources.uint64Set;
    //    testedPairs->clear();
    //    testedPairs->reserve(bodyCount * 8);
    //
    //    auto pairKey = [](BodyIndex a, BodyIndex b) -> uint64_t
    //        {
    //            const uint32_t lo = static_cast<uint32_t>(std::min(a, b));
    //            const uint32_t hi = static_cast<uint32_t>(std::max(a, b));
    //
    //            constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
    //            uint64_t h = (uint64_t)lo + addConst;
    //            h ^= (uint64_t)hi + addConst + (h << 6) + (h >> 2);
    //            return h;
    //        };
    //
    //    {
    //        TRACY_SCOPE_N("Test pairs");
    //        for (auto& entry : *grid)
    //        {
    //            const std::vector<BodyIndex>& bodiesInCell = entry.second;
    //            const size_t bodyInCellCount = bodiesInCell.size();
    //            if (bodyInCellCount < 2) continue;
    //
    //            for (size_t i = 0; i < bodyInCellCount; i++)
    //            {
    //                const BodyIndex bodyIndexA = bodiesInCell[i];
    //                const Real minXA = aabbMinX[bodyIndexA];
    //                const Real minYA = aabbMinY[bodyIndexA];
    //                const Real maxXA = aabbMaxX[bodyIndexA];
    //                const Real maxYA = aabbMaxY[bodyIndexA];
    //
    //                for (size_t j = i + 1; j < bodyInCellCount; j++)
    //                {
    //                    const BodyIndex bodyIndexB = bodiesInCell[j];
    //                    const uint64_t key = pairKey(bodyIndexA, bodyIndexB);
    //                    if (!testedPairs->insert(key).second)
    //                        continue;
    //
    //                    const Real minXB = aabbMinX[bodyIndexB];
    //                    const Real minYB = aabbMinY[bodyIndexB];
    //                    const Real maxXB = aabbMaxX[bodyIndexB];
    //                    const Real maxYB = aabbMaxY[bodyIndexB];
    //
    //                    const bool doesIntersect =
    //                        (minXA < maxXB && maxXA > minXB) &&
    //                        (minYA < maxYB && maxYA > minYB);
    //
    //                    if (doesIntersect)
    //                    {
    //                        collidingBodyPairs.emplace_back(bodyIndexA, bodyIndexB);
    //                    }
    //                }
    //            }
    //        }
    //    }
    //}

    void BroadPhaseCollisionDetector::buildBvhNode(std::vector<BvhNode>& nodes, std::vector<BodyIndex>& indices, uint32_t bodyCount)
    {
        using RealSimd = Simd<Real>;

        const Real* CORE_RESTRICT aabbMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT aabbMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT aabbMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT aabbMaxYPtr = bodiesAABB.maxY;

        BodyIndex* CORE_RESTRICT indicesPtr = indices.data();

        // Compute centroids.
        // Note: Removed '*0.5' because it doesn't impact order.
        Real* CORE_RESTRICT centroidXPtr = nullptr;
        Real* CORE_RESTRICT centroidYPtr = nullptr;
        {
            auto& centroidX = functionResources.centroidX;
            auto& centroidY = functionResources.centroidY;
            centroidX.resize(bodyCount);
            centroidY.resize(bodyCount);
            centroidXPtr = centroidX.data();
            centroidYPtr = centroidY.data();
        }

        {
            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd minX = RealSimd::load(aabbMinXPtr + i);
                const RealSimd maxX = RealSimd::load(aabbMaxXPtr + i);
                const RealSimd minY = RealSimd::load(aabbMinYPtr + i);
                const RealSimd maxY = RealSimd::load(aabbMaxYPtr + i);

                const RealSimd centroidX = minX + maxX;
                const RealSimd centroidY = minY + maxY;

                centroidX.store(centroidXPtr + i);
                centroidY.store(centroidYPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                centroidXPtr[i] = aabbMinXPtr[i] + aabbMaxXPtr[i];
                centroidYPtr[i] = aabbMinYPtr[i] + aabbMaxYPtr[i];
            }
        }

        // Local stack.
        struct BuildTask
        {
            uint32_t start, end;
            uint32_t parentIdx; // INVALID_INDEX for the root.
            bool isRight; // Which child slot to fill in the parent.
        };

        constexpr uint64_t MAX_STACK_CAPACITY = bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE) + 1ull;
        BuildTask stack[MAX_STACK_CAPACITY];
        uint32_t stackSize = 0;

        stack[stackSize++] = { 0, bodyCount, BvhNode::INVALID_INDEX, false };

        while (stackSize > 0)
        {
            const BuildTask task = stack[--stackSize];

            // Compute merged bounding box.
            Real minX =  std::numeric_limits<Real>::max();
            Real maxX = -std::numeric_limits<Real>::max();
            Real minY =  std::numeric_limits<Real>::max();
            Real maxY = -std::numeric_limits<Real>::max();
            for (uint32_t i = task.start; i < task.end; i++)
            {
                const BodyIndex b = indicesPtr[i];
                minX = std::min(minX, aabbMinXPtr[b]);
                maxX = std::max(maxX, aabbMaxXPtr[b]);
                minY = std::min(minY, aabbMinYPtr[b]);
                maxY = std::max(maxY, aabbMaxYPtr[b]);
            }

            const uint32_t nodeIdx = nodes.size();
            nodes.emplace_back(minX, maxX, minY, maxY, BvhNode::INVALID_INDEX, BvhNode::INVALID_INDEX, task.start, task.end);

            // Wire into parent if one exists.
            if (task.parentIdx != BvhNode::INVALID_INDEX)
            {
                if (task.isRight) nodes[task.parentIdx].right = nodeIdx;
                else              nodes[task.parentIdx].left = nodeIdx;
            }

            // Check range.
            const uint32_t rangeSize = task.end - task.start;
            if (rangeSize <= BvhNode::KD_LEAF_SIZE)
                continue;

            // Partition on the widest axis at the median centroid.
            const bool splitX = (maxX - minX) >= (maxY - minY);
            const uint32_t mid = task.start + rangeSize / 2; // (task.start + task.end) / 2;

            if (splitX)
                {
                    std::nth_element(indicesPtr + task.start, indicesPtr + mid, indicesPtr + task.end,
                        [&](BodyIndex a, BodyIndex b) {
                        return centroidXPtr[a] < centroidXPtr[b];
                        });
                }
            else
                {
                    std::nth_element(indicesPtr + task.start, indicesPtr + mid, indicesPtr + task.end,
                        [&](BodyIndex a, BodyIndex b) {
                        return centroidYPtr[a] < centroidYPtr[b];
                        });
                }

            // Push right before left so left is popped and processed first (LIFO).
            stack[stackSize++] = { mid,        task.end, nodeIdx, true  }; // Right child.
            stack[stackSize++] = { task.start, mid,      nodeIdx, false }; // Left child.
        }
    }

    void BroadPhaseCollisionDetector::queryBvhPairs(const std::vector<BvhNode>& nodes, const std::vector<BodyIndex>& indices)
    {
        const Real* CORE_RESTRICT aabbMinXPtr = bodiesAABB.minX;
        const Real* CORE_RESTRICT aabbMinYPtr = bodiesAABB.minY;
        const Real* CORE_RESTRICT aabbMaxXPtr = bodiesAABB.maxX;
        const Real* CORE_RESTRICT aabbMaxYPtr = bodiesAABB.maxY;

        // Local stack.
        constexpr uint64_t MAX_STACK_CAPACITY = 2ull * bvhDepth(UINT32_MAX, BvhNode::KD_LEAF_SIZE) + 1ull;
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
                using RealSimd = Simd<Real>;
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
                            const BodyIndex b = indices[nodeStart + k];
                            out.minX[k] = aabbMinXPtr[b];
                            out.maxX[k] = aabbMaxXPtr[b];
                            out.minY[k] = aabbMinYPtr[b]; 
                            out.maxY[k] = aabbMaxYPtr[b];
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
