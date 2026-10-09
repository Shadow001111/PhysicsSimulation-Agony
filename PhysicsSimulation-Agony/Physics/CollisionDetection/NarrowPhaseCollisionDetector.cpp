#include "NarrowPhaseCollisionDetector.h"

#include "Physics/Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <bit>
#include <tuple>
#include <algorithm>
#include <iostream>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;


    template <typename Map>
    size_t robinHoodMapMemoryUsage(const Map& map)
    {
        // mask() + 1 == current bucket count (always a power of two).
        auto const numElementsWithBuffer = map.calcNumElementsWithBuffer(map.mask() + 1);
        return map.calcNumBytesTotal(numElementsWithBuffer);
    }


    const SymmetricMatrix<NarrowPhaseCollisionDetector::CollisionFunc, NarrowPhaseCollisionDetector::BODY_TYPE_COUNT>
        NarrowPhaseCollisionDetector::collisionFuncs = [] {
        SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> mat;

        mat((size_t)BodyType::Circle, (size_t)BodyType::Circle) = &NarrowPhaseCollisionDetector::collisionCircleCircle;
        mat((size_t)BodyType::Circle, (size_t)BodyType::Box) = &NarrowPhaseCollisionDetector::collisionCircleBox;
        mat((size_t)BodyType::Circle, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionCirclePolygon;

        mat((size_t)BodyType::Box, (size_t)BodyType::Box) = &NarrowPhaseCollisionDetector::collisionBoxBox;
        mat((size_t)BodyType::Box, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionBoxPolygon;

        mat((size_t)BodyType::Polygon, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionPolygonPolygon;
        return mat;
        }();

    /* HELPER METHODS */

    constexpr Real ZERO_DIVISION_BOUNDARY = 1e-4;
    constexpr Real ZERO_DIVISION_BOUNDARY_SQUARED = ZERO_DIVISION_BOUNDARY * ZERO_DIVISION_BOUNDARY;
    constexpr Real SAT_EPSILON = 1e-4;

    static __forceinline void flipSignIfNegative(Vec2& v, const Real& sign)
    {
        using Int = std::conditional_t<sizeof(Real) == 8, uint64_t, uint32_t>;
        constexpr Int signBit = Int(1) << (sizeof(Real) * 8 - 1);

        const Int signMask = reinterpret_cast<const Int&>(sign) & signBit;
        reinterpret_cast<Int&>(v.x) ^= signMask;
        reinterpret_cast<Int&>(v.y) ^= signMask;

        //const Int signMask = std::bit_cast<Int>(sign) & signBit;
        //v.x = std::bit_cast<Real>(std::bit_cast<Int>(v.x) ^ signMask);
        //v.y = std::bit_cast<Real>(std::bit_cast<Int>(v.y) ^ signMask);

        //if (sign < Real(0))
        //{
        //    v = -v;
        //}
    }

    [[nodiscard]] static bool clipSegment(Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal)
    {
        const Real d1 = glm::dot(p1 - planePoint, planeNormal);
        const Real d2 = glm::dot(p2 - planePoint, planeNormal);

        if (d1 >= 0 && d2 >= 0) return false; // Both inside.
        if (d1 < 0 && d2 < 0) return true;  // Both outside.

        const Real t = d1 / (d1 - d2);
        const Vec2 dir = p2 - p1;
        const Vec2 intersect = p1 + dir * t;

        if (d1 >= 0) p2 = intersect;
        else         p1 = intersect;

        return false;
    };

    static void projectVerticesOnAxis(
        const Vec2* ECSTASY_RESTRICT verticesPtr,
        size_t vertexCount,
        const Vec2& axis,
        Real& ECSTASY_RESTRICT minOut,
        Real& ECSTASY_RESTRICT maxOut
    )
    {
        minOut = std::numeric_limits<Real>::max();
        maxOut = -std::numeric_limits<Real>::max();

        for (size_t i = 0; i < vertexCount; i++)
        {
            const Real proj = glm::dot(verticesPtr[i], axis);
            minOut = std::fmin(minOut, proj);
            maxOut = std::fmax(maxOut, proj);
        }
    };

    // Returns normalized or zero vector.
    [[nodiscard]] static Vec2 edgeOutwardNormal(const Vec2* vertsPtr, size_t verticesCount, uint32_t edgeIndex)
    {
        const Vec2 p0 = vertsPtr[edgeIndex];
        const size_t next = (size_t(edgeIndex) + 1 == verticesCount) ? 0 : size_t(edgeIndex) + 1;
        const Vec2 p1 = vertsPtr[next];
        const Vec2 edge = p1 - p0;

        Vec2 n = Vec2{ edge.y, -edge.x };
        const Real lenSquared = glm::dot(n, n);
        if (lenSquared >= ZERO_DIVISION_BOUNDARY_SQUARED) [[likely]]
        {
            n *= Real(1) / std::sqrt(lenSquared);
            return n;
        }
        return Vec2(0, 0);
    };

    /*               */
    NarrowPhaseCollisionDetector::NarrowPhaseCollisionDetector()
    {
    }

    void NarrowPhaseCollisionDetector::setDataViewers(
        const BodySoAViewer& bodies,
        const ColliderSoAViewer& colliders,
        const CircleSoAViewer& circles,
        const BoxSoAViewer& boxes,
        const PolygonSoAViewer& polygons
    )
    {
        this->bodies = bodies;
        this->colliders = colliders;
        this->circles = circles;
        this->boxes = boxes;
        this->polygons = polygons;
    }

    const std::vector<BodyCollisionData>& NarrowPhaseCollisionDetector::findCollisions(
        const std::vector<ObjectPair>& colliderPairs,
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
            useThreading = colliderPairs.size() > 6000;
        }

        // Find collisions.
        if (useThreading)
        {
            findCollisionsMultiThreaded(colliderPairs, colliderPairs.size() / 3000);
        }
        else
        {
            findCollisionsSingleThreaded(colliderPairs);
        }

        return allCollisionData;
    }

    void NarrowPhaseCollisionDetector::findCollisionsInCircle(
        const std::vector<ObjectIndex>& collidersToCheck,
        Vec2 pos, Real radius,
        std::vector<std::pair<ObjectIndex, Real>>& outColliding
    ) const
    {
        // Lambda to find the closest point on an edge segment
        auto closestPointOnSegment = [](const Vec2& p, const Vec2& a, const Vec2& b) -> Vec2
            {
                const Vec2 ab = b - a;
                const Real denom = glm::dot(ab, ab);
                if (denom < ZERO_DIVISION_BOUNDARY) return a;
                const Real t = glm::dot(p - a, ab) / denom;
                return a + ab * glm::clamp(t, Real(0), Real(1));
            };

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
                const Real dist = std::sqrt(glm::dot(delta, delta));
                sdf = dist - otherRadius;
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

                const Real extX = std::max(Real(0), dx);
                const Real extY = std::max(Real(0), dy);
                const Real distOutside = std::sqrt(extX * extX + extY * extY);
                const Real distInside = std::min(Real(0), std::max(dx, dy));

                sdf = distInside + distOutside;
            }
            else if (type == BodyType::Polygon)
            {
                const Real cosB = colliders.worldRotationCos[colliderIndex];
                const Real sinB = colliders.worldRotationSin[colliderIndex];
                const VerticesContainer& localPolygonVertices = polygons.localVertices[shapeIdx];
                const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.data();
                const size_t vertexCount = localPolygonVertices.size();
                if (vertexCount < 3) [[unlikely]] continue;

                const Vec2 rightB = { cosB,  sinB };
                const Vec2 upB = { -sinB, cosB };

                const Vec2 circleLocal = {
                    glm::dot(pos - colliderPos, rightB),
                    glm::dot(pos - colliderPos, upB)
                };

                Real maxSeparation = -std::numeric_limits<Real>::max();
                uint32_t supportEdge = 0;
                for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
                {
                    const Vec2 normal = edgeOutwardNormal(localVerts, vertexCount, i);
                    const Real separation = glm::dot(normal, circleLocal - localVerts[i]);
                    if (separation > maxSeparation)
                    {
                        maxSeparation = separation;
                        supportEdge = i;
                    }
                }

                if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
                {
                    const Vec2 a = localVerts[supportEdge];
                    const Vec2 b = localVerts[(supportEdge + 1) % uint32_t(vertexCount)];
                    const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                    sdf = glm::length(circleLocal - q);
                }
                else
                {
                    // Point is inside the polygon; maxSeparation represents negative depth
                    sdf = maxSeparation;
                }
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
        if constexpr (!ENABLE_WARM_STARTING)
        {
            previousContactDataContainer.clear();
            return;
        }

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

    void NarrowPhaseCollisionDetector::remapPersistentContactData(const std::vector<ObjectDeletion>& deletedColliders)
    {
        if (deletedColliders.empty() || previousContactDataContainer.empty())
        {
            return;
        }

        TRACY_SCOPE_N("Remap persistent contact data");

        constexpr ColliderIndex INVALID_INDEX = std::numeric_limits<ColliderIndex>::max();

        // Compose the whole deletion log once, tracking only touched indices.
        // slotToEntity: slot -> ORIGINAL index of the collider currently in that slot,
        // or INVALID_INDEX if the slot holds a collider created after the cache was built
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
            if (victim != INVALID_INDEX) dead.insert(victim);

            if (s != INVALID_INDEX && s != d)
            {
                const ColliderIndex moved = entityAt(s);
                slotToEntity[d] = moved;
                slotToEntity[s] = INVALID_INDEX; // Vacated; a later create may reuse it.
            }
            else
            {
                slotToEntity[d] = INVALID_INDEX;
            }
        }

        // original index -> final index (INVALID_INDEX if deleted). Untouched indices are absent.
        robin_hood::unordered_flat_map<ColliderIndex, ColliderIndex> remap;
        remap.reserve(dead.size() + slotToEntity.size());
        for (const ColliderIndex e : dead) remap[e] = INVALID_INDEX;
        for (const auto& [slot, entity] : slotToEntity)
        {
            if (entity != INVALID_INDEX && entity != slot) remap[entity] = slot;
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

            if (a != INVALID_INDEX && b != INVALID_INDEX)
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
        total += PS_AGONY::getVectorMemoryUsage(allCollisionData);

        total += PS_AGONY::getVectorMemoryUsage(chunks);

        for (const auto& chunkData : chunks)
        {
            for (const auto& vec : chunkData.pairs.getDirectAccess())
            {
                total += PS_AGONY::getVectorMemoryUsage(vec);
            }
            total += PS_AGONY::getVectorMemoryUsage(chunkData.results);
        }

        total += robinHoodMapMemoryUsage(previousContactDataContainer);

        return total;
    }

    void NarrowPhaseCollisionDetector::findCollisionsSingleThreaded(const std::vector<ObjectPair>& colliderPairs)
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

    void NarrowPhaseCollisionDetector::findCollisionsMultiThreaded(const std::vector<ObjectPair>& colliderPairs, size_t workerCount)
    {
        TRACY_SCOPE_N("Multi-threaded narrow phase");

        constexpr size_t LOAD_BALANCING_FACTOR = 4;

        auto& threadPool = Threading::getGlobalThreadPool();
        workerCount = std::min<size_t>(workerCount, threadPool.getThreadCount());

        size_t chunkCount, chunkSize;
        if constexpr (DETERMINISTIC)
        {
            // Fixed chunk size: partitioning (and thus output order) is independent of thread count.
            chunkSize = DETERMINISTIC_CHUNK_SIZE;
            chunkCount = (colliderPairs.size() + chunkSize - 1) / chunkSize;
        }
        else
        {
            std::tie(chunkCount, chunkSize) = Ecstasy::Core::Threading::ParallelForRangeExecutor::getChunkCountAndSize(
                threadPool, colliderPairs.size(), LOAD_BALANCING_FACTOR);
        }

        if (chunks.size() < chunkCount) chunks.resize(chunkCount); // Grow-only: keeps chunk buffers alive.

        for (size_t i = 0; i < chunkCount; i++)
        {
            ChunkData& cd = chunks[i];
            cd.start = i * chunkSize;
            cd.end = std::min(cd.start + chunkSize, colliderPairs.size());
            cd.finished = false;
        }

        std::vector<Ecstasy::Core::Threading::Task> tasks;
        tasks.reserve(workerCount);

        alignas(64) std::atomic<uint32_t> workerChunkIndex{ 0 };
        alignas(64) std::atomic<uint32_t> destroyedWorkerCount{ 0 };

        // Only the calling thread merges, strictly in chunk order. It merges whatever is
        // already finished between its own chunks, so most copying overlaps with the workers.
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
                destroyedWorkerCount.fetch_add(1, std::memory_order_release);
                destroyedWorkerCount.notify_one();
            };

        // One less task submitted to the pool: the calling thread takes that slot itself.
        for (size_t i = 0; i + 1 < workerCount; i++)
        {
            tasks.emplace_back([&workerFunc]() { workerFunc(false); });
        }
        threadPool.enqueueBulk(tasks);

        {
            TRACY_SCOPE_N("Participate in work");
            workerFunc(true);
        }

        {
            TRACY_SCOPE_N("Wait for workers to finish and combine data");
            mergeReady(true);
        }
        {
            TRACY_SCOPE_N("Wait for workers to get destroyed");
            while (true)
            {
                const auto value = destroyedWorkerCount.load(std::memory_order_acquire);
                if (value == workerCount) break;
                destroyedWorkerCount.wait(value, std::memory_order_acquire);
            }
        }
    }

    void NarrowPhaseCollisionDetector::processPairs(const std::vector<ObjectPair>& pairs, ChunkData& chunkData, std::vector<BodyCollisionData>& out)
    {
        TRACY_SCOPE_N("Process pair range");

        const size_t firstResult = out.size();

        const BodyType* ECSTASY_RESTRICT shapeTypePtr = colliders.shapeType;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;
        const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies.isStatic;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = chunkData.start; i < chunkData.end; i++)
            {
                auto [colliderIndexA, colliderIndexB] = pairs[i];

                const ObjectIndex bodyIndexA = colliderOwnerPtr[colliderIndexA];
                const ObjectIndex bodyIndexB = colliderOwnerPtr[colliderIndexB];

                if (bodyIndexA == bodyIndexB)
                {
                    // Colliders on the same body never collide. Broad phase already
                    // filters this; kept here as a cheap safety net.
                    continue;
                }

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
        const auto& pairsDA = chunkData.pairs.getDirectAccess();
        const auto& collisionFuncsDA = collisionFuncs.getDirectAccess();

        constexpr size_t count = chunkData.pairs.STORED_COUNT;
        for (size_t i = 0; i < count; i++)
        {
            auto& vec = pairsDA[i];
            if (!vec.empty())
            {
                CollisionFunc func = collisionFuncsDA[i];
                (this->*func)(vec, out);
            }
        }

        // Move persistent contact data from previous frame.
        if constexpr (ENABLE_WARM_STARTING)
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

                // Match current contacts with previous frame contacts using contactIds
                for (uint32_t i = 0; i < collData.contactCount; i++)
                {
                    if (data.contactIds[0] != uint32_t(-1) && collData.contactIds[i] == data.contactIds[0])
                    {
                        collData.persistentContactData[i] = data.contactData[0];
                    }
                    else if (data.contactIds[1] != uint32_t(-1) && collData.contactIds[i] == data.contactIds[1])
                    {
                        collData.persistentContactData[i] = data.contactData[1];
                    }
                }
            }
        }
    }


    /*===COLLISION METHODS===*/

    void NarrowPhaseCollisionDetector::collisionCircleCircle(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        const size_t pairCount = pairs.size();
        size_t i = 0;
        if constexpr (true)
        {
            ObjectPair colliderPairsBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionAXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionAYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionBXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionBYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real radiusABatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real radiusBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real normalXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real normalYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real depthBatch[RealSimd::lanes];

            const RealSimd normalBX(1);
            const RealSimd normalBY(0);

            for (; i + RealSimd::lanes <= pairCount; i += RealSimd::lanes)
            {
                for (size_t j = 0; j < RealSimd::lanes; j++)
                {
                    const auto pair = pairs[i + j];
                    colliderPairsBatch[j] = pair;

                    const ObjectIndex indexA = pair.a;
                    const ObjectIndex indexB = pair.b;

                    positionAXBatch[j] = positionXPtr[indexA];
                    positionAYBatch[j] = positionYPtr[indexA];
                    positionBXBatch[j] = positionXPtr[indexB];
                    positionBYBatch[j] = positionYPtr[indexB];

                    const ObjectIndex shapeA = shapeIndexPtr[indexA];
                    const ObjectIndex shapeB = shapeIndexPtr[indexB];

                    radiusABatch[j] = radiusPtr[shapeA];
                    radiusBBatch[j] = radiusPtr[shapeB];
                }

                const RealSimd positionAX = RealSimd::load(positionAXBatch);
                const RealSimd positionAY = RealSimd::load(positionAYBatch);
                const RealSimd positionBX = RealSimd::load(positionBXBatch);
                const RealSimd positionBY = RealSimd::load(positionBYBatch);

                const RealSimd radiusA = RealSimd::load(radiusABatch);
                const RealSimd radiusB = RealSimd::load(radiusBBatch);

                const RealSimd deltaPositionX = positionBX - positionAX;
                const RealSimd deltaPositionY = positionBY - positionAY;

                const RealSimd radiusSum = radiusA + radiusB;
                const RealSimd radiusSumSquared = radiusSum * radiusSum;

                const RealSimd squaredDistance = RealSimd::mulAdd(deltaPositionX, deltaPositionX, deltaPositionY * deltaPositionY);

                const RealSimd isCollidingMask = squaredDistance < radiusSumSquared;

                const RealSimd distance = RealSimd::sqrt(squaredDistance);
                const RealSimd depth = radiusSum - distance;

                const RealSimd isCloseToZeroMask = distance < RealSimd(ZERO_DIVISION_BOUNDARY);

                const RealSimd invDistance = RealSimd(1) / distance;

                const RealSimd normalAX = deltaPositionX * invDistance;
                const RealSimd normalAY = deltaPositionY * invDistance;

                const RealSimd normalX = RealSimd::blendv(normalAX, normalBX, isCloseToZeroMask);
                const RealSimd normalY = RealSimd::blendv(normalAY, normalBY, isCloseToZeroMask);

                normalX.store(normalXBatch);
                normalY.store(normalYBatch);
                depth.store(depthBatch);

                {
                    unsigned int collisionMask = isCollidingMask.movemask();
                    while (collisionMask != 0)
                    {
                        const int lane = std::countr_zero(collisionMask);
                        collisionMask &= collisionMask - 1;

                        const ObjectPair pair = colliderPairsBatch[lane];
                        const Vec2 normal{ normalXBatch[lane], normalYBatch[lane] };
                        const Real depth = depthBatch[lane];
                        const Vec2 positionA{ positionAXBatch[lane], positionAYBatch[lane] };
                        const Real radiusA = radiusABatch[lane];

                        outCollisionData.emplace_back(
                            pair.a, pair.b,
                            colliderOwnerPtr[pair.a], colliderOwnerPtr[pair.b],
                            normal,
                            depth,
                            1,
                            positionA + normal * radiusA,
                            Vec2(),
                            0,
                            0
                        );
                    }
                }
            }
        }
        for (; i < pairCount; i++)
        {
            auto [indexA, indexB] = pairs[i];

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real radiusB = radiusPtr[shapeB];

            const Vec2 deltaPosition = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);

            if (squaredDistance >= radiusSum * radiusSum) continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance < ZERO_DIVISION_BOUNDARY) [[unlikely]]
            {
                normal = { 1.0, 0.0 };
            }
            else
            {
                const Real invDistance = Real(1.0) / distance;
                normal = deltaPosition * invDistance;
            }

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                positionA + normal * radiusA,
                Vec2(),
                0,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-box collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Vec2 right = { cosB, sinB };
            const Vec2 up = { -sinB, cosB };

            const Vec2 d = positionA - positionB;
            const Vec2 circleLocalPosition = {
                glm::dot(d, right),
                glm::dot(d, up)
            };

            const Vec2 closestLocal = {
                glm::clamp(circleLocalPosition.x, -halfWidthB,  halfWidthB),
                glm::clamp(circleLocalPosition.y, -halfHeightB, halfHeightB)
            };

            const Vec2 deltaLocal = closestLocal - circleLocalPosition;
            const Real squaredDistance = glm::dot(deltaLocal, deltaLocal);

            if (squaredDistance >= radiusA * radiusA) continue;

            if (squaredDistance >= ZERO_DIVISION_BOUNDARY_SQUARED)
            {
                const Real distance = std::sqrt(squaredDistance);
                const Real invDistance = Real(1) / distance;

                const Vec2 worldDelta = {
                    cosB * deltaLocal.x - sinB * deltaLocal.y,
                    sinB * deltaLocal.x + cosB * deltaLocal.y
                };
                const Vec2 normal = worldDelta * invDistance;
                const Real depth = radiusA - distance;
                const Vec2 contactOnCircle = positionA + normal * radiusA;

                uint32_t contactId = 0;
                if (circleLocalPosition.x > halfWidthB)  contactId |= 1;
                else if (circleLocalPosition.x < -halfWidthB)  contactId |= 2;
                if (circleLocalPosition.y > halfHeightB) contactId |= 4;
                else if (circleLocalPosition.y < -halfHeightB) contactId |= 8;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normal,
                    depth,
                    1,
                    contactOnCircle,
                    Vec2(),
                    contactId,
                    0
                );
                continue;
            }

            // Circle center inside box: choose nearest face.
            const Real dx = halfWidthB - std::fabs(circleLocalPosition.x);
            const Real dy = halfHeightB - std::fabs(circleLocalPosition.y);
            const bool useX = dx < dy;
            const Real minPen = useX ? dx : dy;

            const Real sx = std::copysign(Real(1), circleLocalPosition.x);
            const Real sy = std::copysign(Real(1), circleLocalPosition.y);

            Vec2 normalLocal;
            uint32_t contactId;
            if (useX)
            {
                normalLocal = Vec2(sx, Real(0));
                contactId = sx > 0 ? 0 : 1;
            }
            else
            {
                normalLocal = Vec2(Real(0), sy);
                contactId = sy > 0 ? 2 : 3;
            }

            const Vec2 normal = -Vec2{
                cosB * normalLocal.x - sinB * normalLocal.y,
                sinB * normalLocal.x + cosB * normalLocal.y
            };
            const Real depth = radiusA + minPen;
            const Vec2 contactOnCircle = positionA + normal * radiusA;


            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                contactOnCircle,
                Vec2(),
                contactId,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-polygon collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        auto closestPointOnSegment = [](const Vec2& p, const Vec2& a, const Vec2& b) -> Vec2
            {
                const Vec2 ab = b - a;
                const Real denom = glm::dot(ab, ab);
                if (denom < ZERO_DIVISION_BOUNDARY) return a;
                const Real t = glm::dot(p - a, ab) / denom;
                return a + ab * glm::clamp(t, Real(0), Real(1));
            };

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];

            const VerticesContainer& localPolygonVertices = polyLocalVerticesPtr[shapeB];
            const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.data();
            const size_t vertexCount = localPolygonVertices.size();
            if (vertexCount < 3) [[unlikely]] continue;

            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            const Vec2 abPosDelta = positionA - positionB;
            const Vec2 circleLocal = {
                glm::dot(abPosDelta, rightB),
                glm::dot(abPosDelta, upB)
            };

            Real maxSeparation = -std::numeric_limits<Real>::max();
            uint32_t supportEdge = 0;
            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                const Vec2 normal = edgeOutwardNormal(localVerts, vertexCount, i);
                const Real separation = glm::dot(normal, circleLocal - localVerts[i]);
                if (separation > maxSeparation)
                {
                    maxSeparation = separation;
                    supportEdge = i;
                }
            }

            if (maxSeparation > radiusA) continue;

            Vec2 normalLocal;
            Real separation;
            if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
            {
                // Center is outside the polygon but within radiusA of it.
                const Vec2 a = localVerts[supportEdge];
                const Vec2 b = localVerts[(supportEdge + 1) % uint32_t(vertexCount)];
                const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                const Vec2 d = circleLocal - q;
                const Real distSquared = glm::dot(d, d);

                if (distSquared < ZERO_DIVISION_BOUNDARY_SQUARED)
                {
                    normalLocal = edgeOutwardNormal(localVerts, vertexCount, supportEdge);
                    separation = 0;
                }
                else
                {
                    const Real dist = std::sqrt(distSquared);
                    normalLocal = d / dist;
                    separation = dist;
                }

                if (separation > radiusA) continue;
            }
            else
            {
                // Circle center is inside the polygon.
                normalLocal = edgeOutwardNormal(localVerts, vertexCount, supportEdge);
                separation = maxSeparation;
            }

            Vec2 normal = -Vec2{
                rightB.x * normalLocal.x + upB.x * normalLocal.y,
                rightB.y * normalLocal.x + upB.y * normalLocal.y
            };

            const Real depth = radiusA - separation;
            const Vec2 contactOnCircle = positionA + normal * radiusA;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                contactOnCircle,
                Vec2(),
                supportEdge,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-box collision");

        enum class SATAxis : uint32_t
        {
            A_RIGHT,
            A_UP,
            B_RIGHT,
            B_UP
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Vec2 rightA = { cosA,  sinA };
            const Vec2 upA = { -sinA, cosA };
            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            const Real absRelativeCos = std::fabs(cosA * cosB + sinA * sinB);
            const Real absRelativeSin = std::fabs(cosA * sinB - sinA * cosB);

            const Vec2 centerDelta = positionB - positionA;
            const Real centerDeltaOnRightA = glm::dot(centerDelta, rightA);
            const Real centerDeltaOnUpA = glm::dot(centerDelta, upA);
            const Real centerDeltaOnRightB = glm::dot(centerDelta, rightB);
            const Real centerDeltaOnUpB = glm::dot(centerDelta, upB);

            const Real projectedRadiusBOnRightA = halfWidthB * absRelativeCos + halfHeightB * absRelativeSin;
            const Real projectedRadiusBOnUpA = halfWidthB * absRelativeSin + halfHeightB * absRelativeCos;
            const Real projectedRadiusAOnRightB = halfWidthA * absRelativeCos + halfHeightA * absRelativeSin;
            const Real projectedRadiusAOnUpB = halfWidthA * absRelativeSin + halfHeightA * absRelativeCos;

            Vec2 normal{};
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxis = SATAxis::A_RIGHT;

            auto sat = [&](Real radiusSum, Real centerDeltaOnAxis, Vec2 axis, SATAxis axisType) -> bool
                {
                    const Real overlap = radiusSum - std::fabs(centerDeltaOnAxis);
                    if (overlap < Real(0)) return false;
                    if (overlap < depth)
                    {
                        depth = overlap;
                        normal = axis;
                        flipSignIfNegative(normal, centerDeltaOnAxis);
                        bestAxis = axisType;
                    }
                    return true;
                };

            if (!sat(projectedRadiusBOnRightA + halfWidthA, centerDeltaOnRightA, rightA, SATAxis::A_RIGHT)) continue;
            if (!sat(projectedRadiusBOnUpA + halfHeightA, centerDeltaOnUpA, upA, SATAxis::A_UP))    continue;
            if (!sat(projectedRadiusAOnRightB + halfWidthB, centerDeltaOnRightB, rightB, SATAxis::B_RIGHT)) continue;
            if (!sat(projectedRadiusAOnUpB + halfHeightB, centerDeltaOnUpB, upB, SATAxis::B_UP))    continue;

            const bool refIsA = bestAxis < SATAxis::B_RIGHT;

            Vec2 positionRef, positionInc;
            Real halfWidthRef, halfHeightRef, halfWidthInc, halfHeightInc;
            Vec2 rightRef, upRef, rightInc, upInc;
            Vec2 refNormal;

            if (refIsA)
            {
                positionRef = positionA;
                halfWidthRef = halfWidthA;
                halfHeightRef = halfHeightA;
                rightRef = rightA;
                upRef = upA;

                positionInc = positionB;
                halfWidthInc = halfWidthB;
                halfHeightInc = halfHeightB;
                rightInc = rightB;
                upInc = upB;

                refNormal = normal;
            }
            else
            {
                positionRef = positionB;
                halfWidthRef = halfWidthB;
                halfHeightRef = halfHeightB;
                rightRef = rightB;
                upRef = upB;

                positionInc = positionA;
                halfWidthInc = halfWidthA;
                halfHeightInc = halfHeightA;
                rightInc = rightA;
                upInc = upA;

                refNormal = -normal;
            }

            Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;
            {
                const Real dotX = glm::dot(refNormal, rightRef);
                const Real dotY = glm::dot(refNormal, upRef);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real sign = std::copysign(Real(1), useX ? dotX : dotY);

                if (useX)
                {
                    refFaceCenter = positionRef + rightRef * (sign * halfWidthRef);
                    sideDir = upRef;
                }
                else
                {
                    refFaceCenter = positionRef + upRef * (sign * halfHeightRef);
                    sideDir = rightRef;
                }
                const Vec2 edgeOffset = sideDir * (useX ? halfHeightRef : halfWidthRef);
                refEdgeStart = refFaceCenter + edgeOffset;
                refEdgeEnd = refFaceCenter - edgeOffset;
            }

            Vec2 incEdgeStart, incEdgeEnd;
            {
                const Real dotX = glm::dot(refNormal, rightInc);
                const Real dotY = glm::dot(refNormal, upInc);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                Vec2 faceCenter, edgeOffset;
                if (useX)
                {
                    faceCenter = positionInc + rightInc * (sign * halfWidthInc);
                    edgeOffset = upInc * halfHeightInc;
                }
                else
                {
                    faceCenter = positionInc + upInc * (sign * halfHeightInc);
                    edgeOffset = rightInc * halfWidthInc;
                }
                incEdgeStart = faceCenter + edgeOffset;
                incEdgeEnd = faceCenter - edgeOffset;
            }

            Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };
            if (clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir)) continue;
            if (clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir)) continue;

            Vec2 contactPoints[2];
            uint32_t contactIds[2];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);
            const uint32_t contactIdBase = (uint32_t)bestAxis << 1;
            for (uint32_t i = 0; i < 2; i++)
            {
                if (glm::dot(clipped[i], refNormal) <= refPlaneDist + SAT_EPSILON)
                {
                    contactPoints[contactCount] = clipped[i];
                    contactIds[contactCount] = contactIdBase | i;
                    contactCount++;
                }
            }
            if (contactCount == 0) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                contactCount,
                contactPoints[0],
                contactPoints[1],
                contactIds[0],
                contactIds[1]
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-polygon collision");

        // NOTE: This routine works in Polygon B's local space rather than Box A's.
        // The box is described by only a center point and two axes, so transforming
        // it into B's frame is O(1); transforming B's (potentially large) vertex
        // list into A's frame, as the box-box/polygon-polygon routines do, would be
        // O(N). Polygon vertices are therefore read directly from localVertsB with
        // no per-pair transform or scratch buffer.

        enum class SATAxis : uint32_t
        {
            BOX_RIGHT,
            BOX_UP,
            POLY_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];

            const VerticesContainer& localPolygonVertices = polyLocalVerticesPtr[shapeB];
            const Vec2* localVertsB = localPolygonVertices.data();
            const size_t vertexCount = localPolygonVertices.size();
            if (vertexCount < 3) [[unlikely]] continue;

            const Vec2 rightB = { cosB, sinB };
            const Vec2 upB = { -sinB, cosB };

            // Rotation taking a vector from Box A's local space into Polygon B's
            // local space (the inverse of the A<-B rotation used elsewhere in this
            // file), plus Box A's center expressed in B's local space.
            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2 rightA_inB = { relCos, -relSin };
            const Vec2 upA_inB = { relSin, relCos };

            const Vec2 centerDeltaBA = positionA - positionB;
            const Vec2 boxCenterInB = {
                glm::dot(centerDeltaBA, rightB),
                glm::dot(centerDeltaBA, upB)
            };

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::BOX_RIGHT;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            Real polyMinX, polyMaxX, polyMinY, polyMaxY;

            // Test Box X axis (rightA_inB), values kept relative to the box center
            // to match the box-local-space convention the rest of the routine uses.
            {
                Real polyMinXRaw, polyMaxXRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, rightA_inB, polyMinXRaw, polyMaxXRaw);

                const Real boxCenterProjX = glm::dot(boxCenterInB, rightA_inB);
                polyMinX = polyMinXRaw - boxCenterProjX;
                polyMaxX = polyMaxXRaw - boxCenterProjX;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfWidthA - polyMinX, polyMaxX + halfWidthA);
                if (overlap < Real(0)) continue;

                const Real polyCenterX = Real(0.5) * (polyMinX + polyMaxX);
                const Real axisFacing = polyCenterX;
                depth = overlap;
                normalLocal = rightA_inB;
                flipSignIfNegative(normalLocal, polyCenterX);
                bestAxisType = SATAxis::BOX_RIGHT;
                bestAxisFacing = axisFacing;
            }

            // Test Box Y axis (upA_inB).
            {
                Real polyMinYRaw, polyMaxYRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, upA_inB, polyMinYRaw, polyMaxYRaw);

                const Real boxCenterProjY = glm::dot(boxCenterInB, upA_inB);
                polyMinY = polyMinYRaw - boxCenterProjY;
                polyMaxY = polyMaxYRaw - boxCenterProjY;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfHeightA - polyMinY, polyMaxY + halfHeightA);

                const Real polyCenterY = Real(0.5) * (polyMinY + polyMaxY);
                const Real axisFacing = polyCenterY;
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;
                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    normalLocal = upA_inB;
                    flipSignIfNegative(normalLocal, polyCenterY);
                    bestAxisType = SATAxis::BOX_UP;
                    bestAxisFacing = axisFacing;
                }
            }

            // Test Polygon edge outward normals. These are already expressed in
            // Polygon B's own local space, so no per-edge rotation is needed here
            // (unlike the box-local-space version of this routine).
            const Real polyCenterX = Real(0.5) * (polyMinX + polyMaxX);
            const Real polyCenterY = Real(0.5) * (polyMinY + polyMaxY);
            const Vec2 polyCenterRelVec = polyCenterX * rightA_inB + polyCenterY * upA_inB;
            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                const Vec2 nB = edgeOutwardNormal(localVertsB, vertexCount, i);
                Vec2 axisLocal = nB;
                const Real axisLenSquared = glm::dot(axisLocal, axisLocal);
                if (axisLenSquared < ZERO_DIVISION_BOUNDARY_SQUARED) [[unlikely]] continue;

                // Either normalized or zero. No need to normalize 'axisLocal'.

                const Real boxRadius = halfWidthA * std::fabs(glm::dot(axisLocal, rightA_inB)) + halfHeightA * std::fabs(glm::dot(axisLocal, upA_inB));

                Real polyMin, polyMax;
                projectVerticesOnAxis(localVertsB, vertexCount, axisLocal, polyMin, polyMax);

                const Real boxCenterProj = glm::dot(boxCenterInB, axisLocal);
                const Real polyMinRel = polyMin - boxCenterProj;
                const Real polyMaxRel = polyMax - boxCenterProj;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(boxRadius - polyMinRel, polyMaxRel + boxRadius);
                if (overlap < Real(0)) goto nextPair;

                const Real axisFacing = glm::dot(axisLocal, polyCenterRelVec);
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;

                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    // axisLocal is B's own outward edge normal (unambiguous from local winding);
                    // negate to match the "A to B" normal convention, no position heuristic needed.
                    normalLocal = -axisLocal;
                    bestAxisType = SATAxis::POLY_EDGE;
                    bestAxisIndex = i;
                    bestAxisFacing = axisFacing;
                }
            }

            {
                const bool refIsBox = bestAxisType != SATAxis::POLY_EDGE;

                Vec2 refNormalLocal;
                Vec2 refFaceCenterLocal;
                Vec2 sideDirLocal;
                Vec2 refEdgeStartLocal;
                Vec2 refEdgeEndLocal;

                if (refIsBox)
                {
                    refNormalLocal = normalLocal;

                    // normalLocal is exactly +-rightA_inB (BOX_RIGHT) or +-upA_inB (BOX_UP),
                    // so its box-local "x"/"y" components collapse to a known best-axis
                    // check plus a dot product against the corresponding box axis.
                    const bool useX = bestAxisType == SATAxis::BOX_RIGHT;
                    const Real sign = useX ? glm::dot(refNormalLocal, rightA_inB) : glm::dot(refNormalLocal, upA_inB);

                    if (useX)
                    {
                        refFaceCenterLocal = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        sideDirLocal = upA_inB;
                    }
                    else
                    {
                        refFaceCenterLocal = boxCenterInB + upA_inB * (sign * halfHeightA);
                        sideDirLocal = rightA_inB;
                    }

                    const Vec2 edgeOffset = sideDirLocal * (useX ? halfHeightA : halfWidthA);
                    refEdgeStartLocal = refFaceCenterLocal + edgeOffset;
                    refEdgeEndLocal = refFaceCenterLocal - edgeOffset;
                }
                else
                {
                    refNormalLocal = -normalLocal;

                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = (i0 + 1) % uint32_t(vertexCount);

                    refEdgeStartLocal = localVertsB[i0];
                    refEdgeEndLocal = localVertsB[i1];
                    refFaceCenterLocal = (refEdgeStartLocal + refEdgeEndLocal) * Real(0.5);

                    const Vec2 edge = refEdgeEndLocal - refEdgeStartLocal;
                    const Real edgeLen = std::sqrt(glm::dot(edge, edge));
                    if (edgeLen < ZERO_DIVISION_BOUNDARY) continue;

                    sideDirLocal = edge / -edgeLen;
                }

                Vec2 incEdgeStartLocal, incEdgeEndLocal;
                uint32_t incidentEdge, incidentEdge2;

                if (refIsBox)
                {
                    incidentEdge = 0;
                    Real minDot = std::numeric_limits<Real>::max();

                    for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
                    {
                        // Already in Polygon B's local space, same frame as refNormalLocal:
                        // no per-edge rotation needed here (unlike the box-local-space version).
                        const Vec2 nB = edgeOutwardNormal(localVertsB, vertexCount, i);
                        const Real d = glm::dot(refNormalLocal, nB);
                        if (d < minDot)
                        {
                            minDot = d;
                            incidentEdge = i;
                        }
                    }

                    incidentEdge2 = (incidentEdge + 1) % uint32_t(vertexCount);
                    incEdgeStartLocal = localVertsB[incidentEdge];
                    incEdgeEndLocal = localVertsB[incidentEdge2];
                }
                else
                {
                    // refNormalLocal is B's polygon-edge normal expressed in B's local
                    // space already; project it onto the box's axes-in-B to recover the
                    // same box-local x/y components the box-local-space version reads
                    // directly, then pick the box's incident edge exactly as before.
                    const Real dotX = glm::dot(refNormalLocal, rightA_inB);
                    const Real dotY = glm::dot(refNormalLocal, upA_inB);
                    const bool useX = std::fabs(dotX) > std::fabs(dotY);
                    const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                    const int signBool = sign > 0;

                    Vec2 faceCenter, edgeOffset;
                    if (useX)
                    {
                        faceCenter = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        edgeOffset = upA_inB * halfHeightA;

                        incidentEdge = (1 - signBool) * 3;
                        incidentEdge2 = 2 - signBool;
                    }
                    else
                    {
                        faceCenter = boxCenterInB + upA_inB * (sign * halfHeightA);
                        edgeOffset = rightA_inB * halfWidthA;

                        incidentEdge = 1 - signBool;
                        incidentEdge2 = 2 + signBool;
                    }

                    incEdgeStartLocal = faceCenter + edgeOffset;
                    incEdgeEndLocal = faceCenter - edgeOffset;
                }

                Vec2 clipped[2] = { incEdgeStartLocal, incEdgeEndLocal };
                if (clipSegment(clipped[0], clipped[1], refEdgeStartLocal, -sideDirLocal)) continue;
                if (clipSegment(clipped[0], clipped[1], refEdgeEndLocal, sideDirLocal)) continue;

                const uint32_t bodyTag = refIsBox ? 0x80000000u : 0u;
                uint32_t id1 = bodyTag | incidentEdge;
                uint32_t id2 = bodyTag | incidentEdge2;

                Vec2 contactPoints[2];
                uint32_t contactIds[2];
                uint32_t contactCount = 0;
                const Real refPlaneDist = glm::dot(refFaceCenterLocal, refNormalLocal);

                const Vec2 normalWorld = rightB * normalLocal.x + upB * normalLocal.y;

                for (uint32_t i = 0; i < 2; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionB + rightB * clipped[i].x + upB * clipped[i].y;
                        contactIds[contactCount] = (i == 0) ? id1 : id2;
                        contactCount++;
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normalWorld,
                    depth,
                    contactCount,
                    contactPoints[0],
                    contactPoints[1],
                    contactIds[0],
                    contactIds[1]
                );
            }

        nextPair:
            continue;
        }
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-polygon collision");

        enum class SATAxis : uint32_t
        {
            A_EDGE,
            B_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        static thread_local std::vector<Vec2> vertsB_inA;
        static thread_local std::vector<Vec2> normalsA;      // A edge normals, A-local.
        static thread_local std::vector<Vec2> normalsB_inA;  // B edge normals rotated into A-local.

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const VerticesContainer& localVertsAContainer = polyLocalVerticesPtr[shapeA];
            const VerticesContainer& localVertsBContainer = polyLocalVerticesPtr[shapeB];

            const size_t countA = localVertsAContainer.size();
            const size_t countB = localVertsBContainer.size();
            if (countA < 3 || countB < 3) [[unlikely]] continue;

            const Vec2 rightA = { cosA, sinA };
            const Vec2 upA = { -sinA, cosA };

            const Vec2 centerDelta = positionB - positionA;
            const Vec2 bodyDeltaLocal = {
                glm::dot(centerDelta, rightA),
                glm::dot(centerDelta, upA)
            };

            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2* localVertsA = localVertsAContainer.data();
            const Vec2* localVertsB = localVertsBContainer.data();

            vertsB_inA.resize(countB);
            normalsA.resize(countA);
            normalsB_inA.resize(countB);
            Real polyMinX_B = std::numeric_limits<Real>::max();
            Real polyMaxX_B = -std::numeric_limits<Real>::max();
            Real polyMinY_B = std::numeric_limits<Real>::max();
            Real polyMaxY_B = -std::numeric_limits<Real>::max();

            for (size_t i = 0; i < countB; i++)
            {
                const Vec2 v = localVertsB[i];
                const Vec2 vInA = {
                    bodyDeltaLocal.x + relCos * v.x - relSin * v.y,
                    bodyDeltaLocal.y + relSin * v.x + relCos * v.y
                };
                vertsB_inA[i] = vInA;

                polyMinX_B = std::fmin(polyMinX_B, vInA.x);
                polyMaxX_B = std::fmax(polyMaxX_B, vInA.x);
                polyMinY_B = std::fmin(polyMinY_B, vInA.y);
                polyMaxY_B = std::fmax(polyMaxY_B, vInA.y);
            }

            Real polyMinX_A = localVertsA[0].x;
            Real polyMaxX_A = polyMinX_A;
            Real polyMinY_A = localVertsA[0].y;
            Real polyMaxY_A = polyMinY_A;

            for (size_t i = 1; i < countA; i++)
            {
                polyMinX_A = std::fmin(polyMinX_A, localVertsA[i].x);
                polyMaxX_A = std::fmax(polyMaxX_A, localVertsA[i].x);
                polyMinY_A = std::fmin(polyMinY_A, localVertsA[i].y);
                polyMaxY_A = std::fmax(polyMaxY_A, localVertsA[i].y);
            }

            const Vec2 polyCenterA_inA = {
                Real(0.5) * (polyMinX_A + polyMaxX_A),
                Real(0.5) * (polyMinY_A + polyMaxY_A)
            };
            const Vec2 polyCenterB_inA = {
                Real(0.5) * (polyMinX_B + polyMaxX_B),
                Real(0.5) * (polyMinY_B + polyMaxY_B)
            };

            const Vec2 centerDeltaLocal = polyCenterB_inA - polyCenterA_inA;

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::A_EDGE;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            // Face-separation SAT. For a face with outward unit normal n, the owning polygon's
            // max extent along n is just dot(n, faceVertex), so only the OTHER polygon is projected
            // (O(nA*nB) per direction instead of re-projecting both shapes on every axis).
            // overlap = faceDist - min_other(dot(n, v)); negative => separating axis.
            auto consider = [&](Real overlap, Vec2 axisLocal, SATAxis axisType, uint32_t axisIndex)
                {
                    if (overlap > depth) return;

                    const Real facingSign = Real(axisType == SATAxis::B_EDGE) * Real(2) - Real(1);
                    const Real axisFacing = facingSign * glm::dot(axisLocal, centerDeltaLocal);

                    if (overlap < depth || axisFacing < bestAxisFacing)
                    {
                        depth = overlap;
                        // axisLocal is that shape's own outward edge normal: A's points from A to B
                        // already; B's must be negated to match.
                        normalLocal = (axisType == SATAxis::A_EDGE) ? axisLocal : -axisLocal;

                        bestAxisType = axisType;
                        bestAxisIndex = axisIndex;
                        bestAxisFacing = axisFacing;
                    }
                };

            for (uint32_t i = 0; i < uint32_t(countA); i++)
            {
                const Vec2 axisLocal = edgeOutwardNormal(localVertsA, countA, i);
                normalsA[i] = axisLocal;
                if (glm::dot(axisLocal, axisLocal) < ZERO_DIVISION_BOUNDARY_SQUARED) continue;

                Real minB = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countB; j++)
                {
                    const Real p = glm::dot(vertsB_inA[j], axisLocal);
                    minB = p < minB ? p : minB;
                }

                const Real overlap = glm::dot(localVertsA[i], axisLocal) - minB;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::A_EDGE, i);
            }

            for (uint32_t i = 0; i < uint32_t(countB); i++)
            {
                const Vec2 nB = edgeOutwardNormal(localVertsB, countB, i);
                const Vec2 axisLocal = { relCos * nB.x - relSin * nB.y, relSin * nB.x + relCos * nB.y };
                normalsB_inA[i] = axisLocal;
                if (glm::dot(axisLocal, axisLocal) < ZERO_DIVISION_BOUNDARY_SQUARED) continue;

                Real minA = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countA; j++)
                {
                    const Real p = glm::dot(localVertsA[j], axisLocal);
                    minA = p < minA ? p : minA;
                }

                const Real overlap = glm::dot(vertsB_inA[i], axisLocal) - minA;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::B_EDGE, i);
            }

            {
                const bool refIsA = bestAxisType == SATAxis::A_EDGE;

                const Vec2* refVertsPtr = refIsA ? localVertsA : vertsB_inA.data();
                const size_t refCount = refIsA ? countA : countB;

                const Vec2* incVertsPtr = refIsA ? vertsB_inA.data() : localVertsA;
                const size_t incCount = refIsA ? countB : countA;

                Vec2 refNormalLocal = refIsA ? normalLocal : -normalLocal;
                Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;

                {
                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = (i0 + 1) % uint32_t(refCount);

                    refEdgeStart = refVertsPtr[i0];
                    refEdgeEnd = refVertsPtr[i1];
                    refFaceCenter = (refEdgeStart + refEdgeEnd) * Real(0.5);

                    const Vec2 edge = refEdgeEnd - refEdgeStart;
                    const Real edgeLen = std::sqrt(glm::dot(edge, edge));
                    if (edgeLen < ZERO_DIVISION_BOUNDARY) goto nextPair;

                    sideDir = edge / edgeLen;

                    const Vec2 expectedOutward = Vec2{ edge.y, -edge.x };
                    if (glm::dot(expectedOutward, refNormalLocal) < Real(0))
                    {
                        std::swap(refEdgeStart, refEdgeEnd);
                        sideDir = -sideDir;
                    }
                }

                uint32_t incidentEdgeIndex = 0;
                Real minDot = std::numeric_limits<Real>::max();

                for (uint32_t i = 0; i < uint32_t(incCount); i++)
                {
                    const Vec2 nLocal = refIsA ? normalsB_inA[i] : normalsA[i];

                    const Real d = glm::dot(refNormalLocal, nLocal);
                    if (d < minDot)
                    {
                        minDot = d;
                        incidentEdgeIndex = i;
                    }
                }

                Vec2 incEdgeStart = incVertsPtr[incidentEdgeIndex];
                Vec2 incEdgeEnd = incVertsPtr[(incidentEdgeIndex + 1) % uint32_t(incCount)];

                Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };
                if (clipSegment(clipped[0], clipped[1], refEdgeStart, sideDir)) goto nextPair;
                if (clipSegment(clipped[0], clipped[1], refEdgeEnd, -sideDir)) goto nextPair;

                const Real refPlaneDist = glm::dot(refFaceCenter, refNormalLocal);

                const uint32_t bodyTag = refIsA ? 0x80000000u : 0u;
                const uint32_t id1 = bodyTag | incidentEdgeIndex;
                const uint32_t id2 = bodyTag | ((incidentEdgeIndex + 1) % uint32_t(incCount));

                Vec2 contactPoints[2];
                uint32_t contactIds[2];
                uint32_t contactCount = 0;

                const Vec2 normalWorld = rightA * normalLocal.x + upA * normalLocal.y;

                for (uint32_t i = 0; i < 2; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionA + rightA * clipped[i].x + upA * clipped[i].y;
                        contactIds[contactCount] = (i == 0) ? id1 : id2;
                        contactCount++;
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normalWorld,
                    depth,
                    contactCount,
                    contactPoints[0],
                    contactPoints[1],
                    contactIds[0],
                    contactIds[1]
                );
            }
        nextPair:
            continue;
        }
    }
}