#include "NarrowPhaseCollisionDetector.h"
#include "HelperMethods.h"
#include "Constants.h"

#include "PS_AGONY2D/Threading.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <tuple>
#include <algorithm>

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

        auto add = [&](BodyType typeA, BodyType typeB, CollisionFunc func)
            {
                mat((size_t)typeA, (size_t)typeB) = func;
            };

        add(BodyType::Circle , BodyType::Circle , &NarrowPhaseCollisionDetector::collisionCircleCircle  );
        add(BodyType::Circle , BodyType::Box    , &NarrowPhaseCollisionDetector::collisionCircleBox     );
        add(BodyType::Circle , BodyType::Polygon, &NarrowPhaseCollisionDetector::collisionCirclePolygon );
        add(BodyType::Circle , BodyType::Capsule, &NarrowPhaseCollisionDetector::collisionCircleCapsule );

        add(BodyType::Box    , BodyType::Box    , &NarrowPhaseCollisionDetector::collisionBoxBox        );
        add(BodyType::Box    , BodyType::Polygon, &NarrowPhaseCollisionDetector::collisionBoxPolygon    );
        add(BodyType::Box    , BodyType::Capsule, &NarrowPhaseCollisionDetector::collisionBoxCapsule    );

        add(BodyType::Polygon, BodyType::Polygon, &NarrowPhaseCollisionDetector::collisionPolygonPolygon);
        add(BodyType::Polygon, BodyType::Capsule, &NarrowPhaseCollisionDetector::collisionPolygonCapsule);

        add(BodyType::Capsule, BodyType::Capsule, &NarrowPhaseCollisionDetector::collisionCapsuleCapsule);
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

        allCollisionData.clear();

        if (colliderPairs.empty())
        {
            return allCollisionData;
        }

        allCollisionData.reserve(colliderPairs.size());

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

        return allCollisionData;
    }

    void NarrowPhaseCollisionDetector::findCollisionsInCircle(
        std::span<const ObjectIndex> collidersToCheck,
        Vec2 pos, Real radius,
        std::vector<std::pair<ObjectIndex, Real>>& outColliding
    ) const
    {
        for (ObjectIndex colliderIndex : collidersToCheck)
        {
            const BodyType type = colliders.shapeType[colliderIndex];
            const ObjectIndex shapeIdx = colliders.shapeIndex[colliderIndex];
            const Vec2 colliderPos = { colliders.worldPosX[colliderIndex], colliders.worldPosY[colliderIndex] };

            Real sdf = std::numeric_limits<Real>::max();

            if (type == BodyType::Circle)
            {
                const Real otherRadius = circles.radius[shapeIdx];
                const Vec2 delta = colliderPos - pos;
                const Real distSquared = glm::dot(delta, delta);

                // Colliding iff dist < otherRadius + radius: decide on squared values, sqrt only on a hit.
                const Real radiusSum = otherRadius + radius;
                if (radiusSum <= Real(0) || distSquared >= radiusSum * radiusSum) continue;

                sdf = std::sqrt(distSquared) - otherRadius;
            }
            else if (type == BodyType::Box)
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
            else if (type == BodyType::Polygon)
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
            else if (type == BodyType::Capsule)
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

    void NarrowPhaseCollisionDetector::updatePersistentContactData()
    {
        TRACY_SCOPE_N("Update persistent contact data");

        previousContactDataContainer.clear();
        previousContactDataContainer.reserve(allCollisionData.size());
        for (BodyCollisionData& collData : allCollisionData)
        {
            ColliderPairKey key{ collData.colliderA , collData.colliderB };

            // Default CachedContactPair already has invalid ids; reassign in case of a duplicate key.
            CachedContactPair& data = previousContactDataContainer.try_emplace(key).first->second;
            data = CachedContactPair{};

            // Only cache valid active contacts.
            for (uint32_t i = 0; i < collData.contactCount; i++)
            {
                data.contactIds[i] = collData.contactIds[i];
                data.contactData[i] = collData.persistentContactData[i];
            }
        }
    }

    void NarrowPhaseCollisionDetector::remapPersistentContactData(std::span<const ObjectDeletion> deletedColliders)
    {
        if (deletedColliders.empty() || previousContactDataContainer.empty())
        {
            return;
        }

        TRACY_SCOPE_N("Remap persistent contact data");

        // Compose the whole deletion log once, tracking only touched indices.
        // slotToEntity: slot -> ORIGINAL index of the collider currently in that slot,
        // or INVALID_COLLIDER_INDEX if the slot holds a collider created after the cache was built
        // (or was vacated). Slots never touched are identity.
        robin_hood::unordered_flat_map<ColliderIndex, ColliderIndex> slotToEntity;
        robin_hood::unordered_flat_set<ColliderIndex> dead;
        slotToEntity.reserve(deletedColliders.size() * 2);
        dead.reserve(deletedColliders.size());

        auto entityAt = [&](ColliderIndex slot) -> ColliderIndex
            {
                const auto it = slotToEntity.find(slot);
                return it == slotToEntity.end() ? slot : it->second;
            };

        for (const auto& deletion : deletedColliders)
        {
            const ColliderIndex d = deletion.deletedIndex;
            const ColliderIndex s = deletion.swappedFromIndex;

            const ColliderIndex victim = entityAt(d);
            if (victim != INVALID_COLLIDER_INDEX) dead.insert(victim);

            if (s != INVALID_COLLIDER_INDEX && s != d)
            {
                const ColliderIndex moved = entityAt(s);
                slotToEntity[d] = moved;
                slotToEntity[s] = INVALID_COLLIDER_INDEX; // Vacated; a later create may reuse it.
            }
            else
            {
                slotToEntity[d] = INVALID_COLLIDER_INDEX;
            }
        }

        // original index -> final index (INVALID_COLLIDER_INDEX if deleted). Untouched indices are absent.
        robin_hood::unordered_flat_map<ColliderIndex, ColliderIndex> remap;
        remap.reserve(dead.size() + slotToEntity.size());
        for (const ColliderIndex e : dead) remap[e] = INVALID_COLLIDER_INDEX;
        for (const auto& [slot, entity] : slotToEntity)
        {
            if (entity != INVALID_COLLIDER_INDEX && entity != slot) remap[entity] = slot;
        }

        // Update in place: only affected entries are extracted and re-inserted.
        std::vector<std::pair<ColliderPairKey, CachedContactPair>> reinsert;

        for (auto it = previousContactDataContainer.begin(); it != previousContactDataContainer.end();)
        {
            ColliderIndex a = it->first.colliderA;
            ColliderIndex b = it->first.colliderB;

            const auto ra = remap.find(a);
            const auto rb = remap.find(b);
            if (ra == remap.end() && rb == remap.end())
            {
                ++it;
                continue;
            }

            if (ra != remap.end()) a = ra->second;
            if (rb != remap.end()) b = rb->second;

            if (a != INVALID_COLLIDER_INDEX && b != INVALID_COLLIDER_INDEX)
            {
                // Re-sort the indices to preserve narrow-phase invariants.
                const BodyType typeA = colliders.shapeType[a];
                const BodyType typeB = colliders.shapeType[b];

                if (typeA > typeB || (typeA == typeB && b > a))
                {
                    std::swap(a, b);
                }

                reinsert.emplace_back(ColliderPairKey{ a, b }, it->second);
            }

            it = previousContactDataContainer.erase(it);
        }

        for (auto& [key, data] : reinsert)
        {
            previousContactDataContainer.insert_or_assign(key, data);
        }
    }

    void NarrowPhaseCollisionDetector::clearData()
    {
        allCollisionData.clear();
    }

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = sizeof(NarrowPhaseCollisionDetector);
        total += PS_AGONY2D::getVectorMemoryUsage(allCollisionData);

        total += PS_AGONY2D::getVectorMemoryUsage(chunks);

        for (const auto& chunkData : chunks)
        {
            for (const auto& vec : chunkData.pairs.getDirectAccess())
            {
                total += PS_AGONY2D::getVectorMemoryUsage(vec);
            }
            total += PS_AGONY2D::getVectorMemoryUsage(chunkData.results);
        }
        {
            auto const numElementsWithBuffer = previousContactDataContainer.calcNumElementsWithBuffer(previousContactDataContainer.mask() + 1);
            total += previousContactDataContainer.calcNumBytesTotal(numElementsWithBuffer);
        }
        return total;
    }

    void NarrowPhaseCollisionDetector::findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs)
    {
        TRACY_SCOPE_N("Single-threaded narrow phase");

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
            processPairs(colliderPairs, cd, allCollisionData);
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

        TRACY_SCOPE_N("Multi-threaded narrow phase");

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

                    allCollisionData.insert(allCollisionData.end(), cd.results.begin(), cd.results.end());
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

        {
            TRACY_SCOPE_N("Participate in work");
            workerFunc(true);
        }
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

        const BodyType* ECSTASY_RESTRICT shapeTypePtr = colliders.shapeType;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = chunkData.start; i < chunkData.end; i++)
            {
                auto [colliderIndexA, colliderIndexB] = pairs[i];

                BodyType typeA = shapeTypePtr[colliderIndexA];
                BodyType typeB = shapeTypePtr[colliderIndexB];

                if (typeA > typeB)
                {
                    std::swap(colliderIndexA, colliderIndexB);
                    std::swap(typeA, typeB);
                }
                else if (typeA == typeB && colliderIndexB > colliderIndexA)
                {
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

        // Move persistent contact data from previous frame.
        {
            TRACY_SCOPE_N("Move previous contact data");
            for (size_t r = firstResult; r < out.size(); r++)
            {
                BodyCollisionData& collData = out[r];
                ColliderPairKey key{ collData.colliderA , collData.colliderB };

                // Check if pair existed in previous frame.
                const auto it = previousContactDataContainer.find(key);
                if (it == previousContactDataContainer.end()) continue;

                const CachedContactPair& data = it->second;

                // Match current contacts with previous frame contacts using contactIds.
                for (uint32_t i = 0; i < collData.contactCount; i++)
                {
                    if (data.contactIds[0] != INVALID_CONTACT_ID && collData.contactIds[i] == data.contactIds[0])
                    {
                        collData.persistentContactData[i] = data.contactData[0];
                    }
                    else if (data.contactIds[1] != INVALID_CONTACT_ID && collData.contactIds[i] == data.contactIds[1])
                    {
                        collData.persistentContactData[i] = data.contactData[1];
                    }
                }
            }
        }
    }
}