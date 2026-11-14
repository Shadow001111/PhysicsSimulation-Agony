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

        if (colliderPairs.empty())
        {
            previousCollisionArray.clear();
            contactCacheDirty = true;
            return {};
        }

        // Sleep/wake changed 'previousCollisionArray' after the cache was built.
        if (contactCacheDirty) rebuildContactCache();

        currentCollisionArray.reserve(colliderPairs.size());

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

        using Real4Simd = Ecstasy::Core::Simd<Real, sizeof(Real) * 32>;

        contactCacheDirty = false;

        const size_t prevCount = previousCollisionArray.size();
        contactCache.reset(prevCount);

        const BodyCollisionData* ECSTASY_RESTRICT prevPtr = previousCollisionArray.data();
        for (size_t i = 0; i < prevCount; i++)
        {
            const BodyCollisionData& prevData = prevPtr[i];

            // Zero accumulators are identical to a lookup miss: don't store them.
            const Real4Simd impulsesV = Real4Simd::loadu(reinterpret_cast<const Real*>(prevData.persistentContactData.data()));
            uint32_t impulseMask = (impulsesV != Real4Simd::fillLanesWithZero()).movemask();
            impulseMask &= (1u << (2u * prevData.contactCount)) - 1;
            const bool any = impulseMask != 0;
            if (!any) continue;

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
                if (slot >= islandManifolds.size()) [[unlikely]] islandManifolds.resize(slot + 1);
                islandManifolds[slot].push_back(d);
                continue;
            }
            if (write != r) data[write] = d;
            write++;
        }

        if (write != count)
        {
            previousCollisionArray.resize(write);
            contactCacheDirty = true;
        }
    }

    void NarrowPhaseCollisionDetector::wakeContactData(std::span<const uint32_t> islandIds)
    {
        TRACY_SCOPE_N("Wake contact data");

        // Stage the woken islands' manifolds (in the given island order: deterministic). Deleted colliders are dropped.
        wakeStaging.clear();
        for (const uint32_t id : islandIds)
        {
            const size_t slot = size_t(id) - FIRST_SLEEP_ISLAND_ID; // id < FIRST wraps and fails the bound check.
            if (slot >= islandManifolds.size()) [[unlikely]] continue;

            std::vector<BodyCollisionData>& stored = islandManifolds[slot];
            for (const BodyCollisionData& s : stored)
            {
                if (isAlive(s)) wakeStaging.push_back(s);
            }
            stored.clear();
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
            contactCacheDirty = true;
        }
    }

    void NarrowPhaseCollisionDetector::resetCollisionData()
    {
        previousCollisionArray.clear();
        contactCacheDirty = true; // The cache indexes the array that was just cleared.
    }

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = 0;

        // Collision arrays.
        total += getVectorMemoryUsage(currentCollisionArray);
        total += getVectorMemoryUsage(previousCollisionArray);
        for (const auto& stored : islandManifolds) total += getVectorMemoryUsage(stored);
        total += getVectorMemoryUsage(islandManifolds);

        // Chunks.
        total += getVectorMemoryUsage(chunks);
        for (const auto& chunkData : chunks)
        {
            for (const auto& vec : chunkData.pairs.getDirectAccess())
            {
                total += getVectorMemoryUsage(vec);
            }
            total += getVectorMemoryUsage(chunkData.results);
        }

        // The rest.
        total += contactCache.memoryUsage();
        total += wakeCache.memoryUsage();
        total += getVectorMemoryUsage(wakeStaging) + getVectorMemoryUsage(wakeConsumed);
        return total;
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
            processPairs(colliderPairs, cd, currentCollisionArray);
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

        alignas(CACHE_LINE_ALIGNMENT) std::atomic<uint32_t> workerChunkIndex{ 0 };
        alignas(CACHE_LINE_ALIGNMENT) std::atomic<uint32_t> destroyedWorkerCount{ 0 };

        size_t nextToMerge = 0;
        auto mergeReady = [&](bool blocking)
            {
                while (nextToMerge < chunkCount)
                {
                    const ChunkData& cd = chunks[nextToMerge];
                    if (blocking) cd.finished.wait(false, std::memory_order_acquire);
                    else if (!cd.finished.load(std::memory_order_acquire)) return;

                    currentCollisionArray.insert(currentCollisionArray.end(), cd.results.begin(), cd.results.end());
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
                    processPairs(colliderPairs, cd, cd.results);

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

    void NarrowPhaseCollisionDetector::processPairs(std::span<const ObjectPair> pairs, ChunkData& chunkData, CacheLineAlignedVector<BodyCollisionData>& out)
    {
        TRACY_SCOPE_N("Process pair range");

        const size_t firstResult = out.size();

        const ColliderShapeType* ECSTASY_RESTRICT shapeTypePtr = colliders.shapeType;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = chunkData.start; i < chunkData.end; i++)
            {
                auto [colliderIndexA, colliderIndexB] = pairs[i];

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

        finalizeResults(firstResult, out);
    }

    void NarrowPhaseCollisionDetector::finalizeResults(size_t firstResult, std::span<BodyCollisionData> results)
    {
        TRACY_SCOPE_N("Finalize results");

        const HandleTable& handles = *colliders.handles;
        BodyCollisionData* ECSTASY_RESTRICT resultPtr = results.data();
        const size_t end = results.size();

        // Overwrites the index view of the union with handles.
        auto toHandles = [&handles](BodyCollisionData& d)
            {
                const ObjectDenseIndex indexA = d.indexA;
                const ObjectDenseIndex indexB = d.indexB;
                d.colliderA = handles.handleAt(indexA);
                d.colliderB = handles.handleAt(indexB);
            };

        if (contactCache.empty())
        {
            for (size_t r = firstResult; r < end; r++) toHandles(resultPtr[r]);
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
            BodyCollisionData& d = resultPtr[r];
            toHandles(d);
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
                BodyCollisionData& n = resultPtr[ahead];
                toHandles(n);
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