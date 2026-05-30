#include "Simulation.h"

#include "Core/TracyProfiler.h"

#include <numeric>
#include "robin_hood.h"

namespace PS_AGONY
{
    static Real calculateCircleInertia(Real radius, Real mass)
    {
        return Real(0.5) * radius * radius * mass;
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");

        updateTimeAccumulator += deltaTime;

        const Real fixedDeltaTime = simulationSettings.updateInterval;
        while (updateTimeAccumulator >= fixedDeltaTime)
        {
            updateTimeAccumulator -= fixedDeltaTime;
            physicsStep(fixedDeltaTime);
        }
    }

    BodyIndex Simulation::createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex)
    {
        mass = std::max(Real(0.0), mass);
        radius = std::max(Real(0.0), radius);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newCircleIndex = circles.getCount();

        bodies.positionX.push_back(position.x);
        bodies.positionY.push_back(position.y);

        bodies.velocityX.push_back(velocity.x);
        bodies.velocityY.push_back(velocity.y);

        bodies.rotation.push_back(rotation);

        bodies.angularVelocity.push_back(angularVelocity);

        bodies.mass.push_back(mass);
        bodies.invMass.push_back(mass == 0.0 ? 0.0 : 1.0 / mass);

        const Real inertia = calculateCircleInertia(radius, mass);
        bodies.inertia.push_back(inertia);
        bodies.invInertia.push_back(inertia == 0.0 ? 0.0 : 1.0 / inertia);

        bodies.materialIndex.push_back(materialIndex);

        bodies.aabb.minX.push_back(0.0);
        bodies.aabb.minY.push_back(0.0);
        bodies.aabb.maxX.push_back(0.0);
        bodies.aabb.maxY.push_back(0.0);

        bodies.bodyType.push_back(BodyType::Circle);
        bodies.shapeIndex.push_back(newCircleIndex);

        bodies.collisionDebug.push_back(0);

        circles.radius.push_back(radius);
        circles.bodyIndices.push_back(newBodyIndex);

        return newBodyIndex;
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_N("Physics step");

        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        // Self-explanatory.
        applyExternalForces(bodyCount, deltaTime);
        integrate(bodyCount, deltaTime);
        boundaryCollisionResolution(bodyCount);

        // Build AABBs.
        {
            TRACY_SCOPE_N("Build AABBs");
            buildCircleAABBs();
        }

        // Collision detection.
        broadPhaseCollisionDetection(bodyCount);
        narrowPhaseCollisionDetection();

        // Collision resolution.
        resolveCollisions();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_N("Apply external forces");

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityX[i] += gravityDelta.x;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.velocityY[i] += gravityDelta.y;
        }
    }

    void Simulation::integrate(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_N("Intergrate");

        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionX[i] += bodies.velocityX[i] * deltaTime;
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            bodies.positionY[i] += bodies.velocityY[i] * deltaTime;
        }
    }

    void Simulation::boundaryCollisionResolution(size_t bodyCount)
    {
        TRACY_SCOPE_N("Boundary collision");

        const Real boundary = 10.0f;
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real x = bodies.positionX[i];
            const Real absX = std::abs(x);

            if (absX > boundary)
            {
                const Real sign = bodies.positionX[i] > 0.0 ? 1.0 : -1.0;
                bodies.positionX[i] = boundary * sign;
                bodies.velocityX[i] = -bodies.velocityX[i];
            }
        }
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real y = bodies.positionY[i];
            const Real absY = std::abs(y);

            if (absY > boundary)
            {
                const Real sign = bodies.positionY[i] > 0.0 ? 1.0 : -1.0;
                bodies.positionY[i] = boundary * sign;
                bodies.velocityY[i] = -bodies.velocityY[i];
            }
        }
    }

    void Simulation::buildCircleAABBs()
    {
        TRACY_SCOPE_N("Build circle AABBs");

        const size_t circleCount = circles.getCount();

        for (size_t i = 0; i < circleCount; i++)
        {
            const Real radius = circles.radius[i];
            const BodyIndex bodyIndex = circles.bodyIndices[i];

            const Real x = bodies.positionX[bodyIndex];
            const Real y = bodies.positionY[bodyIndex];

            bodies.aabb.minX[bodyIndex] = x - radius;
            bodies.aabb.minY[bodyIndex] = y - radius;
            bodies.aabb.maxX[bodyIndex] = x + radius;
            bodies.aabb.maxY[bodyIndex] = y + radius;
        }
    }

    void Simulation::broadPhaseCollisionDetection(size_t bodyCount)
    {
        TRACY_SCOPE_N("Broad phase");

        std::fill(bodies.collisionDebug.begin(), bodies.collisionDebug.end(), 0);
        broadPhaseCollisions.clear();

        if (bodyCount < 2) return; // No pairs to check.

        broadPhaseCollisions.reserve(bodyCount);

        //justAABB(bodyCount);
        //sweepAndPrune(bodyCount);
        boundVolumeHierarchy(bodyCount);
        //uniformSpaceGrid(bodyCount); // Hella slow.
    }

    void Simulation::narrowPhaseCollisionDetection()
    {
        TRACY_SCOPE_N("Narrow phase");

        const size_t bodyPairCount = broadPhaseCollisions.size();
        if (bodyPairCount == 0) return;

        //for (size_t i = 0; i < bodyPairCount; i++)
        //{
        //    BodyPair bodyPair = broadPhaseCollisions[i];
        //}
    }

    void Simulation::resolveCollisions()
    {
        TRACY_SCOPE_N("Resolve collisions");
    }

    void Simulation::justAABB(size_t bodyCount)
    {
        TRACY_SCOPE_N("AABB checks");

        for (size_t bodyIndexA = 0; bodyIndexA < bodyCount - 1; bodyIndexA++)
        {
            const Real minXA = bodies.aabb.minX[bodyIndexA];
            const Real minYA = bodies.aabb.minY[bodyIndexA];
            const Real maxXA = bodies.aabb.maxX[bodyIndexA];
            const Real maxYA = bodies.aabb.maxY[bodyIndexA];

            for (size_t bodyIndexB = bodyIndexA + 1; bodyIndexB < bodyCount; bodyIndexB++)
            {
                const Real minXB = bodies.aabb.minX[bodyIndexB];
                const Real minYB = bodies.aabb.minY[bodyIndexB];
                const Real maxXB = bodies.aabb.maxX[bodyIndexB];
                const Real maxYB = bodies.aabb.maxY[bodyIndexB];

                const bool doesIntersect =
                    (minXA < maxXB && maxXA > minXB) &&
                    (minYA < maxYB && maxYA > minYB);

                if (doesIntersect)
                {
                    broadPhaseCollisions.emplace_back(bodyIndexA, bodyIndexB);
                    bodies.collisionDebug[bodyIndexA] = 1;
                    bodies.collisionDebug[bodyIndexB] = 1;
                }
            }
        }
    }

    void Simulation::sweepAndPruneXAxis(size_t bodyCount)
    {
        TRACY_SCOPE_N("Sweet and prune X");

        // Build list of body indices sorted by AABB minX.
        static std::vector<BodyIndex> sortedIndices;
        sortedIndices.resize(bodyCount);

        std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
        std::sort(sortedIndices.begin(), sortedIndices.end(),
            [this](BodyIndex a, BodyIndex b) {
                return bodies.aabb.minX[a] < bodies.aabb.minX[b];
            });

        static std::vector<BodyIndex> activeList; // Bodies currently overlapping in X.
        activeList.clear();

        for (BodyIndex current : sortedIndices)
        {
            const Real minXA = bodies.aabb.minX[current];
            const Real minYA = bodies.aabb.minY[current];
            const Real maxYA = bodies.aabb.maxY[current];

            // Remove from activeList any body whose maxX < current minX.
            // For some reason, this is faster than removing with swap and pop.
            activeList.erase(std::remove_if(activeList.begin(), activeList.end(),
                [this, minXA](BodyIndex active) {
                    return bodies.aabb.maxX[active] <= minXA;
                }), activeList.end());

            // Check against all active bodies (they overlap in X).
            for (BodyIndex active : activeList)
            {
                const Real minYB = bodies.aabb.minY[active];
                const Real maxYB = bodies.aabb.maxY[active];

                const bool doesIntersect = (minYA < maxYB && maxYA > minYB);

                if (doesIntersect)
                {
                    broadPhaseCollisions.emplace_back(current, active);
                    bodies.collisionDebug[current] = 1;
                    bodies.collisionDebug[active] = 1;
                }
            }

            activeList.push_back(current);
        }
    }

    void Simulation::boundVolumeHierarchy(size_t bodyCount)
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

    void Simulation::uniformSpaceGrid(size_t bodyCount)
    {
        TRACY_SCOPE_N("Uniform grid");

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
                const Real minX = bodies.aabb.minX[i];
                const Real maxX = bodies.aabb.maxX[i];
                const Real minY = bodies.aabb.minY[i];
                const Real maxY = bodies.aabb.maxY[i];

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
                const Real minX = bodies.aabb.minX[bodyIndex];
                const Real maxX = bodies.aabb.maxX[bodyIndex];
                const Real minY = bodies.aabb.minY[bodyIndex];
                const Real maxY = bodies.aabb.maxY[bodyIndex];

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
                    const Real minXA = bodies.aabb.minX[bodyIndexA];
                    const Real minYA = bodies.aabb.minY[bodyIndexA];
                    const Real maxXA = bodies.aabb.maxX[bodyIndexA];
                    const Real maxYA = bodies.aabb.maxY[bodyIndexA];

                    for (size_t j = i + 1; j < bodyInCellCount; j++)
                    {
                        const BodyIndex bodyIndexB = bodiesInCell[j];
                        const uint64_t key = pairKey(bodyIndexA, bodyIndexB);
                        if (!testedPairs.insert(key).second)
                            continue;

                        const Real minXB = bodies.aabb.minX[bodyIndexB];
                        const Real minYB = bodies.aabb.minY[bodyIndexB];
                        const Real maxXB = bodies.aabb.maxX[bodyIndexB];
                        const Real maxYB = bodies.aabb.maxY[bodyIndexB];

                        const bool doesIntersect =
                            (minXA < maxXB && maxXA > minXB) &&
                            (minYA < maxYB && maxYA > minYB);

                        if (doesIntersect)
                        {
                            broadPhaseCollisions.emplace_back(bodyIndexA, bodyIndexB);
                            bodies.collisionDebug[bodyIndexA] = 1;
                            bodies.collisionDebug[bodyIndexB] = 1;
                        }
                    }
                }
            }
        }
    }

    uint32_t Simulation::buildBvhNode(std::vector<BvhNode>& nodes, std::vector<BodyIndex>& indices, uint32_t start, uint32_t end)
    {
        // Compute merged bounding box.
        Real minX =  std::numeric_limits<Real>::max();
        Real maxX = -std::numeric_limits<Real>::max();
        Real minY =  std::numeric_limits<Real>::max();
        Real maxY = -std::numeric_limits<Real>::max();
        for (uint32_t i = start; i < end; i++)
        {
            const BodyIndex b = indices[i];
            minX = std::min(minX, bodies.aabb.minX[b]);
            maxX = std::max(maxX, bodies.aabb.maxX[b]);
            minY = std::min(minY, bodies.aabb.minY[b]);
            maxY = std::max(maxY, bodies.aabb.maxY[b]);
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
        auto centroid = [this, splitX](BodyIndex i) -> float
            {
                if (splitX)
                    return bodies.aabb.minX[i] + bodies.aabb.maxX[i];
                else
                    return bodies.aabb.minY[i] + bodies.aabb.maxY[i];
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

    void Simulation::queryBvhPairs(const std::vector<BvhNode>& nodes, const std::vector<BodyIndex>& indices, uint32_t nodeA, uint32_t nodeB)
    {
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
                    const Real minXi = bodies.aabb.minX[bi];
                    const Real maxXi = bodies.aabb.maxX[bi];
                    const Real minYi = bodies.aabb.minY[bi];
                    const Real maxYi = bodies.aabb.maxY[bi];
                    for (uint32_t j = i + 1; j < a.end; j++)
                    {
                        const BodyIndex bj = indices[j];
                        const Real minXj = bodies.aabb.minX[bj];
                        const Real maxXj = bodies.aabb.maxX[bj];
                        const Real minYj = bodies.aabb.minY[bj];
                        const Real maxYj = bodies.aabb.maxY[bj];

                        if ((minXi < maxXj && maxXi > minXj) &&
                            (minYi < maxYj && maxYi > minYj))
                        {
                            broadPhaseCollisions.emplace_back(bi, bj);
                            bodies.collisionDebug[bi] = 1;
                            bodies.collisionDebug[bj] = 1;
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
                    const Real minXi = bodies.aabb.minX[bi];
                    const Real maxXi = bodies.aabb.maxX[bi];
                    const Real minYi = bodies.aabb.minY[bi];
                    const Real maxYi = bodies.aabb.maxY[bi];
                    for (uint32_t j = b.start; j < b.end; j++)
                    {
                        const BodyIndex bj = indices[j];
                        const Real minXj = bodies.aabb.minX[bj];
                        const Real maxXj = bodies.aabb.maxX[bj];
                        const Real minYj = bodies.aabb.minY[bj];
                        const Real maxYj = bodies.aabb.maxY[bj];

                        if ((minXi < maxXj && maxXi > minXj) &&
                            (minYi < maxYj && maxYi > minYj))
                        {
                            broadPhaseCollisions.emplace_back(bi, bj);
                            bodies.collisionDebug[bi] = 1;
                            bodies.collisionDebug[bj] = 1;
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