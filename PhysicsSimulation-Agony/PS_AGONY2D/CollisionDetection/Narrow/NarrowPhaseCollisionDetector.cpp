#include "NarrowPhaseCollisionDetector.h"
#include "HelperMethods.h"
#include "Constants.h"

#include "PS_AGONY2D/Threading.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <tuple>
#include <algorithm>
#include <xmmintrin.h>
#include <iostream>

/*
 * NARROW PHASE: DESIGN NOTES
 *
 * Pipeline
 *   processPairs partitions pairs by (typeA <= typeB) into per-type vectors (same type: higher collider index
 *   first), then dispatches one collision method per non-empty group via 'collisionFuncs'. Every method
 *   therefore sees one shape-type combination: no per-pair type branching. Methods only append to
 *   'outCollisionData'; warm-start data is attached afterwards by contact id.
 *   Contact reuse (CONTACT_REUSE_ENABLED): a pair with a manifold from the previous frame, whose colliders stayed
 *   within the tolerances of the transforms that manifold was computed at, skips the collision methods and takes the
 *   previous manifold (impulses included) as it is. Transform snapshots live in arrays index-aligned with the
 *   manifold arrays.
 *
 * Conventions (all methods)
 *   - Normal points from A to B. Depth > 0 means penetration. Contact points lie on the surface of the shape
 *     that is the "reference" for the contact (circle/capsule surface, or the clipped incident edge).
 *   - Max 2 contacts. Ids must be stable between frames (warm starting):
 *       polygon/box face = REFERENCE_BODY_TAG | (faceIndex << 1) | clippedEndpoint, vertex contact = vertex tag | index,
 *       circle-box = BOX_FACE_* region flags, capsule-capsule = overlap end (0/1) or 2 for the closest point.
 *   - Rotation is stored as (cos, sin): right = (cos, sin), up = (-sin, cos). No trig at runtime.
 *   - Epsilons: ZERO_DIVISION_BOUNDARY(_SQUARED) guards divisions and degenerate features, SAT_EPSILON biases
 *     towards faces so the feature (and its id) does not flicker between frames.
 *   - Compiled with /fp:fast: NaN is not handled. Use std::fmin/std::fmax for clamps (not std::min/max or
 *     ternaries). Never rely on NaN comparisons.
 *   - Polygons are validated at build time (>= 3 vertices, no degenerate edges, unit normals cached in
 *     PolygonGeometry). Narrow phase does not re-check; do not add per-pair guards.
 *
 * Method style
 *   - One flat loop per method. Raw ECSTASY_RESTRICT pointers into the SoA arrays, hoisted out of the loop.
 *     No virtual calls, no helper calls in hot paths: small helpers are __forceinline, one-off logic is a lambda
 *     or a single template function. Early-out as soon as one separating test fails (cheapest tests first).
 *   - Work in the local space of the cheaper side: rotate the small shape (box, circle, capsule core: O(1)) into
 *     the polygon's frame instead of transforming all polygon vertices. Output is rotated back to world space.
 *   - Use reciprocal multiply (v * (1 / len)) instead of vector / scalar. Use squared distances until a root is needed.
 *   - Do not add more per-shape cached data (memory is already heavy with cached normals).
 *
 * How to add or change a method
 *   1. Keep the conventions above (normal A->B, stable ids, max 2 contacts).
 *   2. Pick the frame that avoids O(N) transforms. Prefer analytic projections for boxes and capsules.
 *   3. Reject early with bounding circles or the first separating axis. Avoid sqrt and division before the reject.
 *   4. Pure-math methods with a rare branch: use the SIMD batch pattern of circle-circle. Branch-heavy ones
 *      (clipping, feature selection) stay scalar.
 *   5. Register in 'collisionFuncs'. Argument order is (lower BodyType, higher BodyType).
 */

namespace PS_AGONY2D
{
    const SymmetricMatrix<NarrowPhaseCollisionDetector::CollisionFunc, NarrowPhaseCollisionDetector::BODY_TYPE_COUNT>
        NarrowPhaseCollisionDetector::collisionFuncs = [] {
        SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> mat;

        auto add = [&](ColliderShapeType typeA, ColliderShapeType typeB, CollisionFunc func)
            {
                mat((size_t)typeA, (size_t)typeB) = func;
            };

        add(ColliderShapeType::Circle, ColliderShapeType::Circle, &NarrowPhaseCollisionDetector::collisionCircleCircle);
        add(ColliderShapeType::Circle, ColliderShapeType::Box, &NarrowPhaseCollisionDetector::collisionCircleBox);
        add(ColliderShapeType::Circle, ColliderShapeType::Polygon, &NarrowPhaseCollisionDetector::collisionCirclePolygon);
        add(ColliderShapeType::Circle, ColliderShapeType::Capsule, &NarrowPhaseCollisionDetector::collisionCircleCapsule);

        add(ColliderShapeType::Box, ColliderShapeType::Box, &NarrowPhaseCollisionDetector::collisionBoxBox);
        add(ColliderShapeType::Box, ColliderShapeType::Polygon, &NarrowPhaseCollisionDetector::collisionBoxPolygon);
        add(ColliderShapeType::Box, ColliderShapeType::Capsule, &NarrowPhaseCollisionDetector::collisionBoxCapsule);

        add(ColliderShapeType::Polygon, ColliderShapeType::Polygon, &NarrowPhaseCollisionDetector::collisionPolygonPolygon);
        add(ColliderShapeType::Polygon, ColliderShapeType::Capsule, &NarrowPhaseCollisionDetector::collisionPolygonCapsule);

        add(ColliderShapeType::Capsule, ColliderShapeType::Capsule, &NarrowPhaseCollisionDetector::collisionCapsuleCapsule);
        return mat;
        }();


    void NarrowPhaseCollisionDetector::setDataViewers(
        const BodySoAViewer& bodies,
        const ColliderSoAViewer& colliders,
        const CircleSoAViewer& circles,
        const BoxSoAViewer& boxes,
        const PolygonSoAViewer& polygons,
        const CapsuleSoAViewer& capsules
    )
    {
        this->bodies = bodies;
        this->colliders = colliders;
        this->circles = circles;
        this->boxes = boxes;
        this->polygons = polygons;
        this->capsules = capsules;
    }

    std::span<BodyCollisionData> NarrowPhaseCollisionDetector::findCollisions(
        std::span<const ObjectPair> colliderPairs,
        ExecutionPolicy executionPolicy
    )
    {
        TRACY_SCOPE_N("Narrow phase");

        currentCollisionArray.clear();
        if constexpr (CONTACT_REUSE_ENABLED)
        {
            currentReuseArray.clear();
            reuseStats = {};
        }

        if (colliderPairs.empty())
        {
            previousCollisionArray.clear();
            if constexpr (CONTACT_REUSE_ENABLED) previousReuseArray.clear();
            contactCacheDirty = true;
            return {};
        }

        // Sleep/wake changed 'previousCollisionArray' after the cache was built.
        if (contactCacheDirty) rebuildContactCache();

        currentCollisionArray.reserve(colliderPairs.size());
        if constexpr (CONTACT_REUSE_ENABLED) currentReuseArray.reserve(colliderPairs.size());

        // Determine to use threading or not.
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
            useThreading = colliderPairs.size() > MULTITHREAD_PAIR_THRESHOLD;
        }

        // Find collisions.
        if (useThreading)
        {
            findCollisionsMultiThreaded(colliderPairs, colliderPairs.size() / PAIRS_PER_WORKER);
        }
        else
        {
            findCollisionsSingleThreaded(colliderPairs);
        }

        std::swap(currentCollisionArray, previousCollisionArray);
        currentCollisionArray.clear();
        if constexpr (CONTACT_REUSE_ENABLED)
        {
            std::swap(currentReuseArray, previousReuseArray);
            currentReuseArray.clear();
        }

        if constexpr (CONTACT_REUSE_ENABLED && CONTACT_REUSE_PRINT_STATS)
        {
            const size_t total = previousCollisionArray.size();
            const size_t tenthsOfPercent = total != 0 ? size_t(reuseStats.reused) * 1000 / total : 0;
            std::cout << "Contact reuse: " << reuseStats.reused << '/' << total << " manifolds ("
                << tenthsOfPercent / 10 << '.' << tenthsOfPercent % 10 << "%), not cached " << reuseStats.notCached
                << ", handle mismatch " << reuseStats.handleMismatch
                << ", moved " << reuseStats.movedLinear << " linear / " << reuseStats.movedAngular << " angular\n";
        }

        return previousCollisionArray;
    }

    void NarrowPhaseCollisionDetector::findCollisionsInCircle(
        std::span<const ObjectDenseIndex> collidersToCheck,
        Vec2 pos, Real radius,
        std::vector<std::pair<ObjectDenseIndex, Real>>& outColliding
    ) const
    {
        for (ObjectDenseIndex colliderIndex : collidersToCheck)
        {
            const ColliderShapeType type = colliders.shapeType[colliderIndex];
            const ObjectDenseIndex shapeIdx = colliders.shapeIndex[colliderIndex];
            const Vec2 colliderPos = { colliders.worldPosX[colliderIndex], colliders.worldPosY[colliderIndex] };

            Real sdf = std::numeric_limits<Real>::max();

            if (type == ColliderShapeType::Circle)
            {
                const Real otherRadius = circles.radius[shapeIdx];
                const Vec2 delta = colliderPos - pos;
                const Real distSquared = glm::dot(delta, delta);

                // Colliding iff dist < otherRadius + radius: decide on squared values, sqrt only on a hit.
                const Real radiusSum = otherRadius + radius;
                if (radiusSum <= Real(0) || distSquared >= radiusSum * radiusSum) continue;

                sdf = std::sqrt(distSquared) - otherRadius;
            }
            else if (type == ColliderShapeType::Box)
            {
                const Real cosB = colliders.worldRotationCos[colliderIndex];
                const Real sinB = colliders.worldRotationSin[colliderIndex];
                const Real halfWidthB = boxes.halfWidth[shapeIdx];
                const Real halfHeightB = boxes.halfHeight[shapeIdx];

                const Vec2 right = { cosB, sinB };
                const Vec2 up = { -sinB, cosB };

                const Vec2 d = pos - colliderPos;
                const Vec2 circleLocalPosition = {
                    glm::dot(d, right),
                    glm::dot(d, up)
                };

                // Exact 2D Box Signed Distance Field (SDF) representation
                const Real dx = std::fabs(circleLocalPosition.x) - halfWidthB;
                const Real dy = std::fabs(circleLocalPosition.y) - halfHeightB;

                // Outside along one axis by at least 'radius': the distance cannot be smaller. Skips the sqrt.
                const Real maxD = std::fmax(dx, dy);
                if (maxD > Real(0) && maxD >= radius) continue;

                const Real extX = std::fmax(Real(0), dx);
                const Real extY = std::fmax(Real(0), dy);
                const Real distOutside = std::sqrt(extX * extX + extY * extY);
                const Real distInside = std::fmin(Real(0), maxD);

                sdf = distInside + distOutside;
            }
            else if (type == ColliderShapeType::Polygon)
            {
                const Real cosB = colliders.worldRotationCos[colliderIndex];
                const Real sinB = colliders.worldRotationSin[colliderIndex];
                const PolygonGeometry& localPolygonVertices = polygons.geometry[shapeIdx];
                const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.vertices();
                const Vec2* ECSTASY_RESTRICT localNormals = localPolygonVertices.normals();
                const size_t vertexCount = localPolygonVertices.size();

                const Vec2 rightB = { cosB,  sinB };
                const Vec2 upB = { -sinB, cosB };

                const Vec2 d = pos - colliderPos;
                const Vec2 circleLocal = {
                    glm::dot(d, rightB),
                    glm::dot(d, upB)
                };

                uint32_t supportEdge = 0;
                Real maxSeparation = 0;
                if (!findSupportEdge(localVerts, localNormals, vertexCount, circleLocal, radius, supportEdge, maxSeparation)) continue;

                if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
                {
                    const Vec2 a = localVerts[supportEdge];
                    const Vec2 b = localVerts[nextIndex(supportEdge, uint32_t(vertexCount))];
                    const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                    sdf = glm::length(circleLocal - q);
                }
                else
                {
                    // Point is inside the polygon; maxSeparation represents negative depth
                    sdf = maxSeparation;
                }
            }
            else if (type == ColliderShapeType::Capsule)
            {
                const Real halfLengthB = capsules.halfLength[shapeIdx];
                const Real capsuleRadius = capsules.radius[shapeIdx];
                const Vec2 axis = { colliders.worldRotationCos[colliderIndex], colliders.worldRotationSin[colliderIndex] };

                // Closest point of the capsule core to the query center.
                const Vec2 d = pos - colliderPos;
                const Real t = clampValue(glm::dot(d, axis), -halfLengthB, halfLengthB);
                const Vec2 delta = d - axis * t;
                const Real distSquared = glm::dot(delta, delta);

                // Colliding iff dist < capsuleRadius + radius: decide on squared values, sqrt only on a hit.
                const Real radiusSum = capsuleRadius + radius;
                if (radiusSum <= Real(0) || distSquared >= radiusSum * radiusSum) continue;

                sdf = std::sqrt(distSquared) - capsuleRadius;
            }

            // Check if the collider collides with the query circle
            if (sdf < radius)
            {
                outColliding.emplace_back(colliderIndex, sdf);
            }
        }
    }

    void NarrowPhaseCollisionDetector::rebuildContactCache()
    {
        TRACY_SCOPE_N("Rebuild contact cache");

        contactCacheDirty = false;

        const size_t prevCount = previousCollisionArray.size();
        contactCache.reset(prevCount);

        const BodyCollisionData* ECSTASY_RESTRICT prevPtr = previousCollisionArray.data();
        for (size_t i = 0; i < prevCount; i++)
        {
            const BodyCollisionData& prevData = prevPtr[i];

            // Zero accumulators are identical to a lookup miss: don't store them, unless every manifold is needed for reuse.
            if constexpr (!CONTACT_REUSE_ENABLED)
            {
                using Real4Simd = Ecstasy::Core::Simd<Real, sizeof(Real) * 32>;

                const Real4Simd impulsesV = Real4Simd::loadu(reinterpret_cast<const Real*>(prevData.persistentContactData.data()));
                uint32_t impulseMask = (impulsesV != Real4Simd::fillLanesWithZero()).movemask();
                impulseMask &= (1u << (2u * prevData.contactCount)) - 1;
                const bool any = impulseMask != 0;
                if (!any) continue;
            }

            contactCache.insert(makePairKey(prevData.colliderA, prevData.colliderB), uint32_t(i));
        }
    }

    void NarrowPhaseCollisionDetector::sleepContactData(IndexRange sleepingBodies)
    {
        TRACY_SCOPE_N("Sleep contact data");

        if (sleepingBodies.empty() || previousCollisionArray.empty()) return;

        const HandleTable& handles = *colliders.handles;
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderBody = colliders.bodyIndex;
        const ObjectDenseIndex* ECSTASY_RESTRICT sleepIslandIdPtr = bodies.sleepIslandId;
        const uint32_t sleepBegin = static_cast<uint32_t>(sleepingBodies.begin);
        const uint32_t sleepSize = static_cast<uint32_t>(sleepingBodies.size());

        // Single stable compaction pass: preserves order. Manifolds of deleted colliders are dropped.
        BodyCollisionData* ECSTASY_RESTRICT data = previousCollisionArray.data();
        [[maybe_unused]] ContactReuseData* ECSTASY_RESTRICT reuse = previousReuseArray.data();
        const size_t count = previousCollisionArray.size();
        size_t write = 0;
        for (size_t r = 0; r < count; r++)
        {
            const BodyCollisionData& d = data[r];

            const uint32_t indexA = handles.indexOf(d.colliderA);
            const uint32_t indexB = handles.indexOf(d.colliderB);
            if ((indexA == Handle::NULL_DENSE_INDEX) || (indexB == Handle::NULL_DENSE_INDEX)) continue;

            // Unsigned wrap rejects bodies below the range as well.
            const uint32_t bodyA = colliderBody[indexA];
            const uint32_t bodyB = colliderBody[indexB];
            uint32_t id = 0;
            if (bodyA - sleepBegin < sleepSize) id = sleepIslandIdPtr[bodyA];
            else if (bodyB - sleepBegin < sleepSize) id = sleepIslandIdPtr[bodyB];

            if (id != 0)
            {
                const size_t slot = size_t(id) - FIRST_SLEEP_ISLAND_ID;
                if (slot >= islandManifolds.size()) [[unlikely]]
                {
                    islandManifolds.resize(slot + 1);
                    if constexpr (CONTACT_REUSE_ENABLED) islandReuse.resize(slot + 1);
                }
                islandManifolds[slot].push_back(d);
                if constexpr (CONTACT_REUSE_ENABLED) islandReuse[slot].push_back(reuse[r]);
                continue;
            }
            if (write != r)
            {
                data[write] = d;
                if constexpr (CONTACT_REUSE_ENABLED) reuse[write] = reuse[r];
            }
            write++;
        }

        if (write != count)
        {
            previousCollisionArray.resize(write);
            if constexpr (CONTACT_REUSE_ENABLED) previousReuseArray.resize(write);
            contactCacheDirty = true;
        }
    }

    void NarrowPhaseCollisionDetector::wakeContactData(std::span<const uint32_t> islandIds)
    {
        TRACY_SCOPE_N("Wake contact data");

        // Stage the woken islands' manifolds (in the given island order: deterministic). Deleted colliders are dropped.
        wakeStaging.clear();
        if constexpr (CONTACT_REUSE_ENABLED) wakeStagingReuse.clear();
        for (const uint32_t id : islandIds)
        {
            const size_t slot = size_t(id) - FIRST_SLEEP_ISLAND_ID; // id < FIRST wraps and fails the bound check.
            if (slot >= islandManifolds.size()) [[unlikely]] continue;

            std::vector<BodyCollisionData>& stored = islandManifolds[slot];
            for (size_t j = 0; j < stored.size(); j++)
            {
                if (!isAlive(stored[j])) continue;
                wakeStaging.push_back(stored[j]);
                if constexpr (CONTACT_REUSE_ENABLED) wakeStagingReuse.push_back(islandReuse[slot][j]);
            }
            stored.clear();
            if constexpr (CONTACT_REUSE_ENABLED) islandReuse[slot].clear();
        }

        const size_t stagedCount = wakeStaging.size();
        if (stagedCount == 0) return;

        // Small table over the staged pair keys. Duplicate pairs: the first one wins, the rest are dropped.
        wakeConsumed.assign(stagedCount, uint8_t(0));
        wakeCache.reset(stagedCount);
        for (size_t i = 0; i < stagedCount; i++)
        {
            const uint64_t key = makePairKey(wakeStaging[i].colliderA, wakeStaging[i].colliderB);
            if (wakeCache.find(key) == ContactCache::NONE) wakeCache.insert(key, uint32_t(i));
            else wakeConsumed[i] = 1;
        }

        // One read-only pass over this frame's manifolds: pairs generated this frame keep their fresh geometry and
        // take the stored impulses. A flipped collider order means different normal and ids: nothing to take.
        BodyCollisionData* ECSTASY_RESTRICT cur = previousCollisionArray.data();
        const size_t currentCount = previousCollisionArray.size();
        for (size_t i = 0; i < currentCount; i++)
        {
            BodyCollisionData& c = cur[i];
            const uint32_t idx = wakeCache.find(makePairKey(c.colliderA, c.colliderB));
            if (idx == ContactCache::NONE) continue;

            wakeConsumed[idx] = 1;
            const BodyCollisionData& s = wakeStaging[idx];
            if (c.colliderA != s.colliderA || c.colliderB != s.colliderB) continue;

            for (uint32_t a = 0; a < c.contactCount; a++)
            {
                for (uint32_t b = 0; b < s.contactCount; b++)
                {
                    if (c.contactIds[a] == s.contactIds[b])
                    {
                        c.persistentContactData[a] = s.persistentContactData[b];
                        break;
                    }
                }
            }
        }

        // Not generated this frame (partner static, or broad phase skipped the pair): restore the whole manifold.
        for (size_t i = 0; i < stagedCount; i++)
        {
            if (wakeConsumed[i]) continue;
            previousCollisionArray.push_back(wakeStaging[i]);
            if constexpr (CONTACT_REUSE_ENABLED) previousReuseArray.push_back(wakeStagingReuse[i]);
            contactCacheDirty = true;
        }
    }

    void NarrowPhaseCollisionDetector::takeContactsOfColliders(
        std::span<const Handle> colliderHandles,
        std::vector<uint32_t>& outSleepIslandIds,
        std::vector<Handle>& outAwakePartners)
    {
        if (colliderHandles.empty()) return;

        size_t maxSlot = 0;
        for (const Handle h : colliderHandles) maxSlot = std::max(maxSlot, size_t(h.slot));
        if (editedColliderMask.size() <= maxSlot) editedColliderMask.resize(maxSlot + 1, uint8_t(0));

        uint8_t* const mask = editedColliderMask.data();
        const size_t maskSize = editedColliderMask.size();
        for (const Handle h : colliderHandles) mask[h.slot] = 1;

        const auto edited = [mask, maskSize](Handle h) noexcept { return size_t(h.slot) < maskSize && mask[h.slot] != 0; };

        for (size_t slot = 0; slot < islandManifolds.size(); slot++)
        {
            std::vector<BodyCollisionData>& stored = islandManifolds[slot];
            [[maybe_unused]] ContactReuseData* storedReuse = nullptr;
            if constexpr (CONTACT_REUSE_ENABLED) storedReuse = islandReuse[slot].data();
            size_t write = 0;
            const size_t count = stored.size();
            for (size_t r = 0; r < count; r++)
            {
                if (edited(stored[r].colliderA) || edited(stored[r].colliderB)) continue;
                if (write != r)
                {
                    stored[write] = stored[r];
                    if constexpr (CONTACT_REUSE_ENABLED) storedReuse[write] = storedReuse[r];
                }
                write++;
            }
            if (write == count) continue;

            stored.erase(stored.begin() + write, stored.end());
            if constexpr (CONTACT_REUSE_ENABLED) islandReuse[slot].resize(write);
            outSleepIslandIds.push_back(static_cast<uint32_t>(slot + FIRST_SLEEP_ISLAND_ID));
        }

        for (size_t i = 0; i < previousCollisionArray.size(); i++)
        {
            const BodyCollisionData& d = previousCollisionArray[i];
            const bool editedA = edited(d.colliderA);
            const bool editedB = edited(d.colliderB);
            if constexpr (CONTACT_REUSE_ENABLED)
            {
                if (editedA || editedB) previousReuseArray[i].posA.x = CONTACT_REUSE_INVALID_POSITION;
            }
            if (editedA == editedB) continue;
            outAwakePartners.push_back(editedA ? d.colliderB : d.colliderA);
        }

        for (const Handle h : colliderHandles) mask[h.slot] = 0;
    }

    void NarrowPhaseCollisionDetector::resetCollisionData()
    {
        previousCollisionArray.clear();
        if constexpr (CONTACT_REUSE_ENABLED) previousReuseArray.clear();
        contactCacheDirty = true; // The cache indexes the array that was just cleared.
    }

    void NarrowPhaseCollisionDetector::collectMemoryUsage(MemoryUsageTracking& memoryTracking, uint32_t parentNodeIndex) const
    {
        const uint32_t root = memoryTracking.createNode("Narrow phase", 0, parentNodeIndex);

        memoryTracking.createNode("Current collision array", getVectorMemoryUsage(currentCollisionArray), root);
        memoryTracking.createNode("Previous collision array", getVectorMemoryUsage(previousCollisionArray), root);

        size_t chunkBytes = getVectorMemoryUsage(chunks);
        for (const auto& chunkData : chunks)
        {
            for (const auto& vec : chunkData.pairs.getDirectAccess())
            {
                chunkBytes += getVectorMemoryUsage(vec);
            }
            chunkBytes += getVectorMemoryUsage(chunkData.results);
        }
        memoryTracking.createNode("Chunks", chunkBytes, root);

        size_t islandManifoldBytes = getVectorMemoryUsage(islandManifolds);
        for (const auto& stored : islandManifolds)
        {
            islandManifoldBytes += getVectorMemoryUsage(stored);
        }
        memoryTracking.createNode("Island manifolds", islandManifoldBytes, root);

        if constexpr (CONTACT_REUSE_ENABLED)
        {
            size_t reuseBytes = getVectorMemoryUsage(currentReuseArray) + getVectorMemoryUsage(previousReuseArray)
                + getVectorMemoryUsage(wakeStagingReuse) + getVectorMemoryUsage(islandReuse);
            for (const auto& stored : islandReuse) reuseBytes += getVectorMemoryUsage(stored);
            for (const auto& chunkData : chunks) reuseBytes += getVectorMemoryUsage(chunkData.reuse);
            memoryTracking.createNode("Contact reuse", reuseBytes, root);
        }

        memoryTracking.createNode("Edited collider mask", getVectorMemoryUsage(editedColliderMask), root);

        memoryTracking.createNode("Contact cache", contactCache.memoryUsage(), root);

        memoryTracking.createNode("Wake cache", wakeCache.memoryUsage(), root);

        memoryTracking.createNode("Wake", getVectorMemoryUsage(wakeStaging) + getVectorMemoryUsage(wakeConsumed), root);
    }

    void NarrowPhaseCollisionDetector::findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs)
    {
        if (chunks.empty()) chunks.resize(1);
        ChunkData& cd = chunks[0];

        // Same chunk partitioning as the multi-threaded path (when DETERMINISTIC), but results
        // are appended directly to 'allCollisionData': no intermediate buffer, no merge copy.
        const size_t total = colliderPairs.size();
        const size_t step = DETERMINISTIC ? DETERMINISTIC_CHUNK_SIZE : total;
        for (size_t start = 0; start < total; start += step)
        {
            cd.start = start;
            cd.end = std::min(start + step, total);
            cd.clear();
            processPairs(colliderPairs, cd, currentCollisionArray, currentReuseArray);
            if constexpr (CONTACT_REUSE_ENABLED) reuseStats.add(cd.reuseStats);
        }
    }

    void NarrowPhaseCollisionDetector::findCollisionsMultiThreaded(std::span<const ObjectPair> colliderPairs, size_t maxWorkerCount)
    {
        if (colliderPairs.empty()) [[unlikely]] return;

        auto& threadingBackend = Threading::getThreadingBackend();
        const size_t poolCount = threadingBackend.getThreadCount(); // Main thread not included.
        const size_t participants = std::max<size_t>(std::min<size_t>(maxWorkerCount, poolCount + 1), 1); // Includes main.

        if (participants <= 1)
        {
            findCollisionsSingleThreaded(colliderPairs);
            return;
        }

        size_t chunkCount, chunkSize;
        if constexpr (DETERMINISTIC)
        {
            chunkSize = DETERMINISTIC_CHUNK_SIZE;
            chunkCount = (colliderPairs.size() + chunkSize - 1) / chunkSize;
        }
        else
        {
            std::tie(chunkCount, chunkSize) = Threading::getChunkCountAndSize(
                participants, colliderPairs.size(), LOAD_BALANCING_FACTOR);
        }

        if (chunks.size() < chunkCount) chunks.resize(chunkCount);

        for (size_t i = 0; i < chunkCount; i++)
        {
            ChunkData& cd = chunks[i];
            cd.start = i * chunkSize;
            cd.end = std::min(cd.start + chunkSize, colliderPairs.size());
            cd.finished = false;
        }

        // Main takes chunks too, so never launch more workers than chunkCount - 1.
        const size_t launchCount = std::min<size_t>(participants - 1, chunkCount - 1);

        alignas(std::hardware_destructive_interference_size) std::atomic<uint32_t> workerChunkIndex{ 0 };
        alignas(std::hardware_destructive_interference_size) std::atomic<uint32_t> destroyedWorkerCount{ 0 };

        size_t nextToMerge = 0;
        auto mergeReady = [&](bool blocking)
            {
                while (nextToMerge < chunkCount)
                {
                    const ChunkData& cd = chunks[nextToMerge];
                    if (blocking) cd.finished.wait(false, std::memory_order_acquire);
                    else if (!cd.finished.load(std::memory_order_acquire)) return;

                    currentCollisionArray.insert(currentCollisionArray.end(), cd.results.begin(), cd.results.end());
                    if constexpr (CONTACT_REUSE_ENABLED)
                    {
                        currentReuseArray.insert(currentReuseArray.end(), cd.reuse.begin(), cd.reuse.end());
                        reuseStats.add(cd.reuseStats);
                    }
                    nextToMerge++;
                }
            };

        auto workerFunc = [&](bool isMainThread)
            {
                while (true)
                {
                    const uint32_t chunkIndex = workerChunkIndex.fetch_add(1, std::memory_order_relaxed);
                    if (chunkIndex >= chunkCount) break;

                    ChunkData& cd = chunks[chunkIndex];
                    cd.clear();
                    processPairs(colliderPairs, cd, cd.results, cd.reuse);

                    cd.finished.store(true, std::memory_order_release);
                    if (isMainThread)
                    {
                        mergeReady(false);
                    }
                    else
                    {
                        cd.finished.notify_one();
                    }
                }

                // Only pool workers signal; main never signals itself.
                if (!isMainThread)
                {
                    destroyedWorkerCount.fetch_add(1, std::memory_order_release);
                    destroyedWorkerCount.notify_one();
                }
            };

        if (launchCount > 0)
        {
            std::vector<uint32_t> threadIndices(launchCount);
            for (uint32_t i = 0; i < launchCount; i++)
            {
                threadIndices[i] = i;
            }

            threadingBackend.launch(
                launchCount,
                [](void* ctx, size_t) { (*static_cast<decltype(workerFunc)*>(ctx))(false); },
                &workerFunc,
                threadIndices
            );
        }

        workerFunc(true);
        {
            TRACY_SCOPE_N("Wait for workers to finish and combine data");
            mergeReady(true);
        }
        if (launchCount > 0)
        {
            TRACY_SCOPE_N("Wait for workers to get destroyed");
            while (true)
            {
                const auto value = destroyedWorkerCount.load(std::memory_order_acquire);
                if (value == launchCount) break;
                destroyedWorkerCount.wait(value, std::memory_order_acquire);
            }
        }
    }

    std::span<const ObjectPair> NarrowPhaseCollisionDetector::reuseContacts(
        std::span<const ObjectPair> pairs,
        ChunkData& chunkData,
        CacheLineAlignedVector<BodyCollisionData>& out,
        CacheLineAlignedVector<ContactReuseData>& outReuse
    )
    {
        TRACY_SCOPE_N("Reuse contacts");

        const size_t begin = chunkData.start;
        const size_t end = chunkData.end;
        ContactReuseStats& stats = chunkData.reuseStats;

        if (contactCache.empty())
        {
            stats.notCached += uint32_t(end - begin);
            return pairs.subspan(begin, end - begin);
        }

        CacheLineAlignedVector<ObjectPair>& remaining = chunkData.remainingPairs;
        remaining.reserve(end - begin);

        const HandleTable& handles = *colliders.handles;
        const Real* ECSTASY_RESTRICT posX = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT posY = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotCos = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotSin = colliders.worldRotationSin;
        const BodyCollisionData* ECSTASY_RESTRICT prevDataPtr = previousCollisionArray.data();
        const ContactReuseData* ECSTASY_RESTRICT prevReusePtr = previousReuseArray.data();

        struct RingEntry
        {
            uint64_t key;
            Handle handleA, handleB;
        };

        const auto makeEntry = [&](size_t i) noexcept
            {
                const ObjectPair pair = pairs[i];
                const Handle handleA = handles.handleAt(pair.a);
                const Handle handleB = handles.handleAt(pair.b);
                return RingEntry{ makePairKey(handleA, handleB), handleA, handleB };
            };

        const auto defer = [&remaining](uint32_t& counter, const ObjectPair& pair)
            {
                counter++;
                remaining.push_back(pair);
            };

        constexpr size_t LOOKAHEAD = 8; // Power of two.
        constexpr size_t RING_MASK = LOOKAHEAD - 1;
        RingEntry ring[LOOKAHEAD];

        const size_t warmupEnd = std::min(begin + LOOKAHEAD, end);
        for (size_t i = begin; i < warmupEnd; i++)
        {
            const RingEntry entry = makeEntry(i);
            ring[(i - begin) & RING_MASK] = entry;
            contactCache.prefetch(entry.key);
        }

        for (size_t i = begin; i < end; i++)
        {
            const size_t ringSlot = (i - begin) & RING_MASK;
            const RingEntry entry = ring[ringSlot];

            if (i + LOOKAHEAD < end)
            {
                const RingEntry next = makeEntry(i + LOOKAHEAD);
                ring[ringSlot] = next;
                contactCache.prefetch(next.key);
            }

            const ObjectPair pair = pairs[i];
            const ObjectDenseIndex indexA = pair.a;
            const ObjectDenseIndex indexB = pair.b;

            const uint32_t idx = contactCache.find(entry.key);
            if (idx == ContactCache::NONE) { defer(stats.notCached, pair); continue; }

            // The previous manifold keeps its own collider order, which may differ from the current canonical order.
            const BodyCollisionData& prevData = prevDataPtr[idx];
            const bool sameOrder    = prevData.colliderA == entry.handleA && prevData.colliderB == entry.handleB;
            const bool flippedOrder = prevData.colliderA == entry.handleB && prevData.colliderB == entry.handleA;
            if (!(sameOrder || flippedOrder)) { defer(stats.handleMismatch, pair); continue; }

            const ObjectDenseIndex prevIndexA = sameOrder ? indexA : indexB;
            const ObjectDenseIndex prevIndexB = sameOrder ? indexB : indexA;

            const ContactReuseData& snapshot = prevReusePtr[idx];
            const Real dxA = posX[prevIndexA] - snapshot.posA.x;
            const Real dyA = posY[prevIndexA] - snapshot.posA.y;
            const Real dxB = posX[prevIndexB] - snapshot.posB.x;
            const Real dyB = posY[prevIndexB] - snapshot.posB.y;

            const bool linearOk =
                dxA * dxA + dyA * dyA < CONTACT_REUSE_LINEAR_TOLERANCE_SQUARED &&
                dxB * dxB + dyB * dyB < CONTACT_REUSE_LINEAR_TOLERANCE_SQUARED;
            if (!linearOk) { defer(stats.movedLinear, pair); continue; }

            const bool rotationOkA = rotCos[prevIndexA] * snapshot.rotA.x + rotSin[prevIndexA] * snapshot.rotA.y >= CONTACT_REUSE_ANGULAR_COS_TOLERANCE;
            const bool rotationOkB = rotCos[prevIndexB] * snapshot.rotB.x + rotSin[prevIndexB] * snapshot.rotB.y >= CONTACT_REUSE_ANGULAR_COS_TOLERANCE;
            if (!(rotationOkA && rotationOkB)) { defer(stats.movedAngular, pair); continue; }

            out.push_back(prevData);
            BodyCollisionData& reused = out.back();

            reused.penetration = snapshot.penetration - ((dxB - dxA) * reused.normal.x + (dyB - dyA) * reused.normal.y);
            const Vec2 shift = { (dxA + dxB) * HALF, (dyA + dyB) * HALF };
            reused.contactPoints[0] = snapshot.contactPoints[0] + shift;
            reused.contactPoints[1] = snapshot.contactPoints[1] + shift;
            outReuse.push_back(snapshot);

            stats.reused++;
        }
        return remaining;
    }

    void NarrowPhaseCollisionDetector::processPairs(
        std::span<const ObjectPair> pairs,
        ChunkData& chunkData,
        CacheLineAlignedVector<BodyCollisionData>& out,
        CacheLineAlignedVector<ContactReuseData>& outReuse
    )
    {
        TRACY_SCOPE_N("Process pair range");

        std::span<const ObjectPair> pairsToCompute;
        if constexpr (CONTACT_REUSE_ENABLED)
        {
            pairsToCompute = reuseContacts(pairs, chunkData, out, outReuse);
        }
        else
        {
            pairsToCompute = pairs.subspan(chunkData.start, chunkData.end - chunkData.start);
        }

        const ColliderShapeType* ECSTASY_RESTRICT shapeTypePtr = colliders.shapeType;

        // Partition into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");

            for (const ObjectPair pair : pairsToCompute)
            {
                auto [colliderIndexA, colliderIndexB] = pair;

                ColliderShapeType typeA = shapeTypePtr[colliderIndexA];
                ColliderShapeType typeB = shapeTypePtr[colliderIndexB];

                if (typeA > typeB)
                {
                    std::swap(colliderIndexA, colliderIndexB);
                    std::swap(typeA, typeB);
                }
                else if (typeA == typeB && colliderIndexB > colliderIndexA)
                {
                    // TODO: Maybe do it with handle slots, not dense indices.
                    std::swap(colliderIndexA, colliderIndexB);
                }

                chunkData.pairs((size_t)typeA, (size_t)typeB).emplace_back(colliderIndexA, colliderIndexB);
            }
        }

        // Results from this index on are computed by the collision methods.
        const size_t firstResult = out.size();

        // Dispatch each non-empty type group.
        const auto pairsDA = chunkData.pairs.getDirectAccess();
        const auto collisionFuncsDA = collisionFuncs.getDirectAccess();

        constexpr size_t count = chunkData.pairs.STORED_COUNT;
        for (size_t i = 0; i < count; i++)
        {
            auto& vec = pairsDA[i];
            if (!vec.empty())
            {
                CollisionFunc func = collisionFuncsDA[i];
                if (func) [[likely]]
                {
                    (this->*func)(vec, out);
                }
            }
        }

        finalizeResults(firstResult, out, outReuse);
    }

    void NarrowPhaseCollisionDetector::finalizeResults(size_t firstResult, std::span<BodyCollisionData> results, CacheLineAlignedVector<ContactReuseData>& reuse)
    {
        TRACY_SCOPE_N("Finalize results");

        const HandleTable& handles = *colliders.handles;
        BodyCollisionData* ECSTASY_RESTRICT resultPtr = results.data();
        const size_t end = results.size();

        [[maybe_unused]] ContactReuseData* ECSTASY_RESTRICT reusePtr = nullptr;
        if constexpr (CONTACT_REUSE_ENABLED)
        {
            reuse.resize(end);
            reusePtr = reuse.data();
        }

        // Snapshots the collider transforms and overwrites the index view of the union with handles.
        auto toHandles = [&](size_t r)
            {
                BodyCollisionData& d = resultPtr[r];
                const ObjectDenseIndex indexA = d.indexA;
                const ObjectDenseIndex indexB = d.indexB;
                if constexpr (CONTACT_REUSE_ENABLED)
                {
                    ContactReuseData& snapshot = reusePtr[r];
                    snapshot.posA = { colliders.worldPosX[indexA], colliders.worldPosY[indexA] };
                    snapshot.posB = { colliders.worldPosX[indexB], colliders.worldPosY[indexB] };
                    snapshot.rotA = { colliders.worldRotationCos[indexA], colliders.worldRotationSin[indexA] };
                    snapshot.rotB = { colliders.worldRotationCos[indexB], colliders.worldRotationSin[indexB] };
                    snapshot.penetration = d.penetration;
                    snapshot.contactPoints[0] = d.contactPoints[0];
                    snapshot.contactPoints[1] = d.contactPoints[1];
                }
                d.colliderA = handles.handleAt(indexA);
                d.colliderB = handles.handleAt(indexB);
            };

        if (contactCache.empty())
        {
            for (size_t r = firstResult; r < end; r++) toHandles(r);
            return;
        }

        const BodyCollisionData* ECSTASY_RESTRICT prevDataPtr = previousCollisionArray.data();

        // Pair keys computed LOOKAHEAD results ahead (right after their handle conversion) and kept in a ring,
        // so each key is computed once: it is used for the prefetch now and for the lookup later.
        constexpr size_t LOOKAHEAD = 8; // Power of two.
        constexpr size_t RING_MASK = LOOKAHEAD - 1;
        uint64_t keys[LOOKAHEAD];

        const size_t warmupEnd = std::min(firstResult + LOOKAHEAD, end);
        for (size_t r = firstResult; r < warmupEnd; r++)
        {
            toHandles(r);
            const BodyCollisionData& d = resultPtr[r];
            const uint64_t key = makePairKey(d.colliderA, d.colliderB);
            keys[(r - firstResult) & RING_MASK] = key;
            contactCache.prefetch(key);
        }

        for (size_t r = firstResult; r < end; r++)
        {
            // Slot of 'r' equals the slot of 'r + LOOKAHEAD': read the key, then reuse the slot.
            const size_t slot = (r - firstResult) & RING_MASK;
            const uint64_t key = keys[slot];

            const size_t ahead = r + LOOKAHEAD;
            if (ahead < end)
            {
                toHandles(ahead);
                const BodyCollisionData& n = resultPtr[ahead];
                const uint64_t nextKey = makePairKey(n.colliderA, n.colliderB);
                keys[slot] = nextKey;
                contactCache.prefetch(nextKey);
            }

            BodyCollisionData& currData = resultPtr[r];
            const uint32_t idx = contactCache.find(key);
            if (idx == ContactCache::NONE) continue;

            const BodyCollisionData& prevData = prevDataPtr[idx];
            if (prevData.colliderA != currData.colliderA || prevData.colliderB != currData.colliderB) continue;
            const uint32_t prevContactCount = prevData.contactCount;

            // Match current contacts with previous frame contacts using contactIds.
            for (uint32_t i = 0; i < currData.contactCount; i++)
            {
                const uint32_t id = currData.contactIds[i];
                if (prevContactCount > 0 && id == prevData.contactIds[0])
                    currData.persistentContactData[i] = prevData.persistentContactData[0];
                else if (prevContactCount > 1 && id == prevData.contactIds[1])
                    currData.persistentContactData[i] = prevData.persistentContactData[1];
            }
        }
    }
}