#include "BroadPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"

#include <numeric>
#include <cmath>
#include "robin_hood.h"

namespace PS_AGONY
{
    const std::vector<BodyPair>& BroadPhaseCollisionDetector::findCollisions(const AABBSoAViewer& bodiesAABBViewer)
    {
        TRACY_SCOPE_N("Broad phase");

        bodiesAABB = bodiesAABBViewer;

        collidingBodyPairs.clear();

        const size_t bodyCount = bodiesAABB.getCount();
        if (bodyCount < 2) return collidingBodyPairs; // No pairs to check.

        collidingBodyPairs.reserve(bodyCount);

        //sweepAndPruneXAxis(bodyCount);
        boundVolumeHierarchy(bodyCount); // The best.
        //uniformSpaceGrid(bodyCount); // Hella slow.

        return collidingBodyPairs;
    }

    void BroadPhaseCollisionDetector::sweepAndPruneXAxis(size_t bodyCount)
    {
        TRACY_SCOPE_N("Sweep and prune X");

        const Real* __restrict aabbMinX = bodiesAABB.minX;
        const Real* __restrict aabbMinY = bodiesAABB.minY;
        const Real* __restrict aabbMaxX = bodiesAABB.maxX;
        const Real* __restrict aabbMaxY = bodiesAABB.maxY;

        // Build list of body indices sorted by AABB minX.
        static std::vector<BodyIndex> sortedIndices;
        sortedIndices.resize(bodyCount);

        std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
        std::sort(sortedIndices.begin(), sortedIndices.end(),
            [&](BodyIndex a, BodyIndex b) {
                return aabbMinX[a] < aabbMinX[b];
            });

        static std::vector<BodyIndex> activeList; // Bodies currently overlapping in X.
        activeList.clear();

        for (BodyIndex current : sortedIndices)
        {
            const Real minXA = aabbMinX[current];
            const Real minYA = aabbMinY[current];
            const Real maxYA = aabbMaxY[current];

            // Remove from activeList any body whose maxX < current minX.
            // For some reason, this is faster than removing with swap and pop.
            activeList.erase(std::remove_if(activeList.begin(), activeList.end(),
                [&](BodyIndex active) {
                    return aabbMaxX[active] <= minXA;
                }), activeList.end());

            // Check against all active bodies (they overlap in X).
            for (BodyIndex active : activeList)
            {
                const Real minYB = aabbMinY[active];
                const Real maxYB = aabbMaxY[active];

                const bool doesIntersect = (minYA < maxYB && maxYA > minYB);

                if (doesIntersect)
                {
                    collidingBodyPairs.emplace_back(current, active);
                }
            }

            activeList.push_back(current);
        }
    }

    void BroadPhaseCollisionDetector::boundVolumeHierarchy(size_t bodyCount)
    {
        TRACY_SCOPE_N("BVH");

        static std::vector<BvhNode> nodes;
        static std::vector<BodyIndex> indices;

        nodes.clear();
        nodes.reserve(2 * bodyCount);

        indices.resize(bodyCount);
        std::iota(indices.begin(), indices.end(), 0);

        buildBvhNode(nodes, indices, 0, bodyCount);
        queryBvhPairs(nodes, indices, 0, 0); // Self-query the root finds all pairs.
    }

    void BroadPhaseCollisionDetector::uniformSpaceGrid(size_t bodyCount)
    {
        TRACY_SCOPE_N("Uniform grid");

        const Real* __restrict aabbMinX = bodiesAABB.minX;
        const Real* __restrict aabbMinY = bodiesAABB.minY;
        const Real* __restrict aabbMaxX = bodiesAABB.maxX;
        const Real* __restrict aabbMaxY = bodiesAABB.maxY;

        // Compute world bounds from all AABBs.
        Real globalMinX = std::numeric_limits<Real>::max();
        Real globalMaxX = -std::numeric_limits<Real>::max();
        Real globalMinY = std::numeric_limits<Real>::max();
        Real globalMaxY = -std::numeric_limits<Real>::max();

        Real totalExtent = Real(0.0);
        {
            TRACY_SCOPE_N("Determine world AABB and bodies' total extent");
            for (size_t i = 0; i < bodyCount; i++)
            {
                const Real minX = aabbMinX[i];
                const Real maxX = aabbMaxX[i];
                const Real minY = aabbMinY[i];
                const Real maxY = aabbMaxY[i];

                globalMinX = std::min(globalMinX, minX);
                globalMaxX = std::max(globalMaxX, maxX);
                globalMinY = std::min(globalMinY, minY);
                globalMaxY = std::max(globalMaxY, maxY);

                const Real extentX = maxX - minX;
                const Real extentY = maxY - minY;
                totalExtent += std::max(extentX, extentY);
            }
        }

        const Real averageBodyExtent = totalExtent / Real(bodyCount);

        const Real worldWidth = std::max(globalMaxX - globalMinX, Real(1e-3));
        const Real worldHeight = std::max(globalMaxY - globalMinY, Real(1e-3));
        const Real worldArea = worldWidth * worldHeight;

        // Determine cell size.
        Real cellSize = std::max(
            std::sqrt(worldArea / Real(bodyCount)),
            averageBodyExtent * Real(2.0)
        );
        cellSize = std::max(cellSize, Real(0.25));

        const Real invCellSize = Real(1.0) / cellSize;

        // Grid size.
        const int gridWidth = std::max(1, static_cast<int>(std::ceil(worldWidth * invCellSize)));
        const int gridHeight = std::max(1, static_cast<int>(std::ceil(worldHeight * invCellSize)));

        // Map each occupied cell to a list of body indices.
        static robin_hood::unordered_flat_map<uint64_t, std::vector<BodyIndex>> grid;
        grid.clear();
        grid.reserve(bodyCount * 4);

        auto getCellKey = [](int cx, int cy) -> uint64_t {
            constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
            uint64_t h = (uint64_t)cx + addConst;
            h ^= (uint64_t)cy + addConst + (h << 6) + (h >> 2);
            return h;
            };

        {
            TRACY_SCOPE_N("Put bodies into cells");
            for (BodyIndex bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
            {
                const Real minX = aabbMinX[bodyIndex];
                const Real maxX = aabbMaxX[bodyIndex];
                const Real minY = aabbMinY[bodyIndex];
                const Real maxY = aabbMaxY[bodyIndex];

                int cx0 = static_cast<int>(std::floor((minX - globalMinX) * invCellSize));
                int cx1 = static_cast<int>(std::floor((maxX - globalMinX) * invCellSize));
                int cy0 = static_cast<int>(std::floor((minY - globalMinY) * invCellSize));
                int cy1 = static_cast<int>(std::floor((maxY - globalMinY) * invCellSize));

                cx0 = std::clamp(cx0, 0, gridWidth - 1);
                cx1 = std::clamp(cx1, 0, gridWidth - 1);
                cy0 = std::clamp(cy0, 0, gridHeight - 1);
                cy1 = std::clamp(cy1, 0, gridHeight - 1);

                for (int cx = cx0; cx <= cx1; cx++)
                    for (int cy = cy0; cy <= cy1; cy++)
                        grid[getCellKey(cx, cy)].push_back(bodyIndex);
            }
        }

        // For each cell, test all pairs inside it.
        static robin_hood::unordered_flat_set<uint64_t> testedPairs;
        testedPairs.clear();
        testedPairs.reserve(bodyCount * 8);

        auto pairKey = [](BodyIndex a, BodyIndex b) -> uint64_t
            {
                const uint32_t lo = static_cast<uint32_t>(std::min(a, b));
                const uint32_t hi = static_cast<uint32_t>(std::max(a, b));

                constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
                uint64_t h = (uint64_t)lo + addConst;
                h ^= (uint64_t)hi + addConst + (h << 6) + (h >> 2);
                return h;
            };

        {
            TRACY_SCOPE_N("Test pairs");
            for (auto& entry : grid)
            {
                const std::vector<BodyIndex>& bodiesInCell = entry.second;
                const size_t bodyInCellCount = bodiesInCell.size();
                if (bodyInCellCount < 2) continue;

                for (size_t i = 0; i < bodyInCellCount; i++)
                {
                    const BodyIndex bodyIndexA = bodiesInCell[i];
                    const Real minXA = aabbMinX[bodyIndexA];
                    const Real minYA = aabbMinY[bodyIndexA];
                    const Real maxXA = aabbMaxX[bodyIndexA];
                    const Real maxYA = aabbMaxY[bodyIndexA];

                    for (size_t j = i + 1; j < bodyInCellCount; j++)
                    {
                        const BodyIndex bodyIndexB = bodiesInCell[j];
                        const uint64_t key = pairKey(bodyIndexA, bodyIndexB);
                        if (!testedPairs.insert(key).second)
                            continue;

                        const Real minXB = aabbMinX[bodyIndexB];
                        const Real minYB = aabbMinY[bodyIndexB];
                        const Real maxXB = aabbMaxX[bodyIndexB];
                        const Real maxYB = aabbMaxY[bodyIndexB];

                        const bool doesIntersect =
                            (minXA < maxXB && maxXA > minXB) &&
                            (minYA < maxYB && maxYA > minYB);

                        if (doesIntersect)
                        {
                            collidingBodyPairs.emplace_back(bodyIndexA, bodyIndexB);
                        }
                    }
                }
            }
        }
    }

    uint32_t BroadPhaseCollisionDetector::buildBvhNode(std::vector<BvhNode>& nodes, std::vector<BodyIndex>& indices, uint32_t start, uint32_t end)
    {
        const Real* __restrict aabbMinX = bodiesAABB.minX;
        const Real* __restrict aabbMinY = bodiesAABB.minY;
        const Real* __restrict aabbMaxX = bodiesAABB.maxX;
        const Real* __restrict aabbMaxY = bodiesAABB.maxY;

        // Compute merged bounding box.
        Real minX = std::numeric_limits<Real>::max();
        Real maxX = -std::numeric_limits<Real>::max();
        Real minY = std::numeric_limits<Real>::max();
        Real maxY = -std::numeric_limits<Real>::max();
        for (uint32_t i = start; i < end; i++)
        {
            const BodyIndex b = indices[i];
            minX = std::min(minX, aabbMinX[b]);
            maxX = std::max(maxX, aabbMaxX[b]);
            minY = std::min(minY, aabbMinY[b]);
            maxY = std::max(maxY, aabbMaxY[b]);
        }

        const uint32_t nodeIdx = nodes.size();
        nodes.emplace_back(minX, maxX, minY, maxY, BvhNode::INVALID_INDEX, BvhNode::INVALID_INDEX, start, end);

        const uint32_t rangeSize = end - start;
        if (rangeSize <= BvhNode::KD_LEAF_SIZE)
            return nodeIdx;

        // Partition on the widest axis at the median centroid.
        const bool splitX = (maxX - minX) >= (maxY - minY);
        const uint32_t mid = start + rangeSize / 2;// (start + end) / 2;

        // Removed '*0.5' because there's no difference for order.
        auto centroid = [&](BodyIndex i) -> float
            {
                if (splitX)
                    return aabbMinX[i] + aabbMaxX[i];
                else
                    return aabbMinY[i] + aabbMaxY[i];
            };

        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [&](BodyIndex a, BodyIndex b)
            {
                return centroid(a) < centroid(b);
            });

        const uint32_t left = buildBvhNode(nodes, indices, start, mid);
        const uint32_t right = buildBvhNode(nodes, indices, mid, end);

        // Re-access by index: recursive calls may have reallocated nodes.
        nodes[nodeIdx].left = left;
        nodes[nodeIdx].right = right;
        return nodeIdx;
    }

    void BroadPhaseCollisionDetector::queryBvhPairs(const std::vector<BvhNode>& nodes, const std::vector<BodyIndex>& indices, uint32_t nodeA, uint32_t nodeB)
    {
        const Real* __restrict aabbMinX = bodiesAABB.minX;
        const Real* __restrict aabbMinY = bodiesAABB.minY;
        const Real* __restrict aabbMaxX = bodiesAABB.maxX;
        const Real* __restrict aabbMaxY = bodiesAABB.maxY;

        const BvhNode& a = nodes[nodeA];
        const BvhNode& b = nodes[nodeB];

        // Prune entire subtree pair if their bounding boxes don't overlap.
        if (a.minX >= b.maxX || a.maxX <= b.minX ||
            a.minY >= b.maxY || a.maxY <= b.minY)
            return;

        const bool aLeaf = (a.left == BvhNode::INVALID_INDEX);
        const bool bLeaf = (b.left == BvhNode::INVALID_INDEX);

        if (aLeaf && bLeaf)
        {
            if (nodeA == nodeB)
            {
                // Self-query leaf: unique pairs only.
                for (uint32_t i = a.start; i < a.end; i++)
                {
                    const BodyIndex bi = indices[i];
                    const Real minXi = aabbMinX[bi];
                    const Real maxXi = aabbMaxX[bi];
                    const Real minYi = aabbMinY[bi];
                    const Real maxYi = aabbMaxY[bi];
                    for (uint32_t j = i + 1; j < a.end; j++)
                    {
                        const BodyIndex bj = indices[j];
                        const Real minXj = aabbMinX[bj];
                        const Real maxXj = aabbMaxX[bj];
                        const Real minYj = aabbMinY[bj];
                        const Real maxYj = aabbMaxY[bj];

                        if ((minXi < maxXj && maxXi > minXj) &&
                            (minYi < maxYj && maxYi > minYj))
                        {
                            collidingBodyPairs.emplace_back(bi, bj);
                        }
                    }
                }
            }
            else
            {
                // Cross-query: all pairs between two distinct leaves.
                for (uint32_t i = a.start; i < a.end; i++)
                {
                    const BodyIndex bi = indices[i];
                    const Real minXi = aabbMinX[bi];
                    const Real maxXi = aabbMaxX[bi];
                    const Real minYi = aabbMinY[bi];
                    const Real maxYi = aabbMaxY[bi];
                    for (uint32_t j = b.start; j < b.end; j++)
                    {
                        const BodyIndex bj = indices[j];
                        const Real minXj = aabbMinX[bj];
                        const Real maxXj = aabbMaxX[bj];
                        const Real minYj = aabbMinY[bj];
                        const Real maxYj = aabbMaxY[bj];

                        if ((minXi < maxXj && maxXi > minXj) &&
                            (minYi < maxYj && maxYi > minYj))
                        {
                            collidingBodyPairs.emplace_back(bi, bj);
                        }
                    }
                }
            }
            return;
        }

        if (nodeA == nodeB)
        {
            // Self-query internal node.
            const uint32_t L = a.left, R = a.right;
            queryBvhPairs(nodes, indices, L, L);
            queryBvhPairs(nodes, indices, L, R);
            queryBvhPairs(nodes, indices, R, R);
        }
        else if (aLeaf || (!bLeaf && (a.end - a.start) < (b.end - b.start)))
        {
            // Split the larger node B.
            queryBvhPairs(nodes, indices, nodeA, b.left);
            queryBvhPairs(nodes, indices, nodeA, b.right);
        }
        else
        {
            // Split node A.
            queryBvhPairs(nodes, indices, a.left, nodeB);
            queryBvhPairs(nodes, indices, a.right, nodeB);
        }
    }
}
