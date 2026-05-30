#include "Simulation.h"

#include "Core/TracyProfiler.h"

#include <numeric>

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

    void Simulation::broadPhaseCollisionDetection(const size_t bodyCount)
    {
        TRACY_SCOPE_N("Broad phase");

        std::fill(bodies.collisionDebug.begin(), bodies.collisionDebug.end(), 0);
        broadPhaseCollisions.clear();

        if (bodyCount < 2) return; // No pairs to check.

        broadPhaseCollisions.reserve(bodyCount);

        //justAABB(bodyCount);
        //sweepAndPrune(bodyCount);
        kdTrees(bodyCount);
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

    void Simulation::justAABB(const size_t bodyCount)
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

    void Simulation::sweepAndPrune(size_t bodyCount)
    {
        TRACY_SCOPE_N("Sweet and prune");

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

    void Simulation::kdTrees(size_t bodyCount)
    {
        TRACY_SCOPE_N("KD trees");

        static std::vector<KDNode> kdNodes;
        static std::vector<BodyIndex> kdIndices;

        kdNodes.clear();
        kdNodes.reserve(2 * bodyCount); // A balanced binary tree needs at most 2n nodes.

        kdIndices.resize(bodyCount);
        std::iota(kdIndices.begin(), kdIndices.end(), 0);

        buildKDNode(kdNodes, kdIndices, 0, (int32_t)bodyCount);
        queryKDPairs(kdNodes, kdIndices, 0, 0); // Self-query the root finds all pairs.
    }

    int32_t Simulation::buildKDNode(std::vector<KDNode>& nodes, std::vector<BodyIndex>& indices, int32_t start, int32_t end)
    {
        // Compute merged bounding box for all bodies in [start, end).
        Real minX = std::numeric_limits<Real>::max();
        Real maxX = -std::numeric_limits<Real>::max();
        Real minY = std::numeric_limits<Real>::max();
        Real maxY = -std::numeric_limits<Real>::max();
        for (int32_t i = start; i < end; i++)
        {
            const BodyIndex b = indices[i];
            minX = std::min(minX, bodies.aabb.minX[b]);
            maxX = std::max(maxX, bodies.aabb.maxX[b]);
            minY = std::min(minY, bodies.aabb.minY[b]);
            maxY = std::max(maxY, bodies.aabb.maxY[b]);
        }

        const int32_t nodeIdx = (int32_t)nodes.size();
        nodes.emplace_back(minX, maxX, minY, maxY, -1, -1, start, end);

        if (end - start <= KDNode::KD_LEAF_SIZE)
            return nodeIdx;

        // Partition on the widest axis at the median centroid.
        const bool splitX = (maxX - minX) >= (maxY - minY);
        const int32_t mid = start + (end - start) / 2;// (start + end) / 2;

        auto centroid = [this, splitX](BodyIndex i) -> float
            {
                if (splitX)
                    return 0.5f * (bodies.aabb.minX[i] + bodies.aabb.maxX[i]);
                else
                    return 0.5f * (bodies.aabb.minY[i] + bodies.aabb.maxY[i]);
            };

        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [&](BodyIndex a, BodyIndex b)
            {
                return centroid(a) < centroid(b);
            });

        const int32_t left = buildKDNode(nodes, indices, start, mid);
        const int32_t right = buildKDNode(nodes, indices, mid, end);

        // Re-access by index: recursive calls may have reallocated nodes.
        nodes[nodeIdx].left = left;
        nodes[nodeIdx].right = right;
        return nodeIdx;
    }

    void Simulation::queryKDPairs(const std::vector<KDNode>& nodes, const std::vector<BodyIndex>& indices, int32_t nodeA, int32_t nodeB)
    {
        const KDNode& a = nodes[nodeA];
        const KDNode& b = nodes[nodeB];

        // Prune entire subtree pair if their bounding boxes don't overlap.
        if (a.minX >= b.maxX || a.maxX <= b.minX ||
            a.minY >= b.maxY || a.maxY <= b.minY)
            return;

        const bool aLeaf = (a.left == -1);
        const bool bLeaf = (b.left == -1);

        if (aLeaf && bLeaf)
        {
            if (nodeA == nodeB)
            {
                // Self-query leaf: unique pairs only.
                for (int32_t i = a.start; i < a.end; ++i)
                    for (int32_t j = i + 1; j < a.end; ++j)
                        checkAndRecord(indices[i], indices[j]);
            }
            else
            {
                // Cross-query: all pairs between two distinct leaves.
                for (int32_t i = a.start; i < a.end; ++i)
                    for (int32_t j = b.start; j < b.end; ++j)
                        checkAndRecord(indices[i], indices[j]);
            }
            return;
        }

        if (nodeA == nodeB)
        {
            // Self-query internal: left-left, left-right, right-right.
            // Copy child indices before recursing - a is a reference into nodes.
            const int32_t L = a.left, R = a.right;
            queryKDPairs(nodes, indices, L, L);
            queryKDPairs(nodes, indices, L, R);
            queryKDPairs(nodes, indices, R, R);
        }
        else if (aLeaf || (!bLeaf && (a.end - a.start) < (b.end - b.start)))
        {
            // Split the larger node B.
            queryKDPairs(nodes, indices, nodeA, b.left);
            queryKDPairs(nodes, indices, nodeA, b.right);
        }
        else
        {
            // Split A.
            queryKDPairs(nodes, indices, a.left, nodeB);
            queryKDPairs(nodes, indices, a.right, nodeB);
        }
    }

    void Simulation::checkAndRecord(BodyIndex i, BodyIndex j)
    {
        if ((bodies.aabb.minX[i] < bodies.aabb.maxX[j] && bodies.aabb.maxX[i] > bodies.aabb.minX[j]) &&
            (bodies.aabb.minY[i] < bodies.aabb.maxY[j] && bodies.aabb.maxY[i] > bodies.aabb.minY[j]))
        {
            broadPhaseCollisions.emplace_back(i, j);
            bodies.collisionDebug[i] = 1;
            bodies.collisionDebug[j] = 1;
        }
    }
}