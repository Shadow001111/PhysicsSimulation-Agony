#include "NarrowPhaseCollisionDetector.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

namespace PS_AGONY
{
    struct Vector2AndSqDistance
    {
        Vec2 vector;
        Real squaredDistance;
    };

    static __forceinline void flipSignIfNegative(Vec2& v, const Real& sign)
    {
        using Int = std::conditional_t<sizeof(Real) == 8,
            uint64_t,
            uint32_t>;

        constexpr Int signBit = 1ull << (sizeof(Real) * 8 - 1);

        const Int signMask = reinterpret_cast<const Int&>(sign) & signBit;
        reinterpret_cast<Int&>(v.x) ^= signMask;
        reinterpret_cast<Int&>(v.y) ^= signMask;
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

    NarrowPhaseCollisionDetector::NarrowPhaseCollisionDetector()
    {
    }

    void NarrowPhaseCollisionDetector::setDataViewers(
        const BodySoAViewer& bodies,
        const CircleSoAViewer& circles,
        const BoxSoAViewer& boxes,
        const PolygonSoAViewer& polygons
    )
    {
        this->bodies = bodies;
        this->circles = circles;
        this->boxes = boxes;
        this->polygons = polygons;
    }

    const std::vector<BodyCollisionData>& NarrowPhaseCollisionDetector::findCollisions(
        const std::vector<BodyPair>& bodyPairs,
        ExecutionPolicy executionPolicy
    )
    {
        TRACY_SCOPE_N("Narrow phase");

        allCollisionData.clear();

        if (bodyPairs.empty())
        {
            return allCollisionData;
        }

        allCollisionData.reserve(bodyPairs.size());

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
            useThreading = bodyPairs.size() > 635;
        }
        
        if (useThreading)
        {
            findCollisionsMultiThreaded(bodyPairs);
        }
        else
        {
            findCollisionsSingleThreaded(bodyPairs);
        }

        return allCollisionData;
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

        return total;
    }

    void NarrowPhaseCollisionDetector::findCollisionsSingleThreaded(const std::vector<BodyPair>& bodyPairs)
    {
        TRACY_SCOPE_N("Single‑threaded narrow phase");

        chunks.resize(1);
        ChunkData& cd = chunks[0];
        cd.start = 0;
        cd.end = bodyPairs.size();
        cd.clear();
        processPairs(bodyPairs, cd);
        allCollisionData.swap(cd.results);
    }

    void NarrowPhaseCollisionDetector::findCollisionsMultiThreaded(const std::vector<BodyPair>& bodyPairs)
    {
        TRACY_SCOPE_N("Multi-threaded narrow phase");

        constexpr size_t LOAD_BALANCING_FACTOR = 4;

        auto& threadPool = Threading::getGlobalThreadPool();
        const size_t workerCount = threadPool.getThreadCount();

        auto [chunkCount, chunkSize] = Ecstasy::Threading::ParallelForRangeExecutor::getChunkCountAndSize(
            threadPool, bodyPairs.size(), LOAD_BALANCING_FACTOR);

        chunks.resize(chunkCount);

        for (size_t i = 0; i < chunkCount; i++)
        {
            ChunkData& cd = chunks[i];
            cd.start = i * chunkSize;
            cd.end = std::min(chunks[i].start + chunkSize, bodyPairs.size());
            cd.finished = false;
        }

        std::vector<Ecstasy::Threading::Task> tasks;
        tasks.reserve(workerCount);

        alignas(64) std::atomic<uint32_t> workerChunkIndex{ 0 };
        alignas(64) std::atomic<uint32_t> destroyedWorkerCount{ 0 };

        auto workerFunc = [&]()
            {
                while (true)
                {
                    const uint32_t chunkIndex = workerChunkIndex.fetch_add(1, std::memory_order_relaxed);
                    if (chunkIndex >= chunkCount) break;

                    ChunkData& cd = chunks[chunkIndex];
                    cd.clear();
                    processPairs(bodyPairs, cd);

                    cd.finished.store(true, std::memory_order_release);
                    cd.finished.notify_one();
                }
                destroyedWorkerCount.fetch_add(1, std::memory_order_release);
                destroyedWorkerCount.notify_one();
            };

        for (size_t i = 0; i < workerCount; i++)
        {
            tasks.emplace_back(workerFunc);
        }
        threadPool.enqueueBulk(tasks);

        {
            TRACY_SCOPE_N("Wait for workers to finish and combine data");
            for (size_t i = 0; i < chunkCount; i++)
            {
                const auto& cd = chunks[i];
                cd.finished.wait(false, std::memory_order_acquire);
                {
                    TRACY_SCOPE_N("Combine");
                    allCollisionData.insert(
                        allCollisionData.end(),
                        cd.results.begin(),
                        cd.results.end()
                    );
                }
            }
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

    void NarrowPhaseCollisionDetector::processPairs(const std::vector<BodyPair>& pairs, ChunkData& chunkData)
    {
        TRACY_SCOPE_N("Process pair range");

        const BodyType* ECSTASY_RESTRICT bodyTypePtr = bodies.bodyType;
        const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies.isStatic;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = chunkData.start; i < chunkData.end; i++)
            {
                auto [bodyIndexA, bodyIndexB] = pairs[i];
                if (isStaticPtr[bodyIndexA] && isStaticPtr[bodyIndexB]) [[unlikely]]
                {
                    continue;
                }

                BodyType typeA = bodyTypePtr[bodyIndexA];
                BodyType typeB = bodyTypePtr[bodyIndexB];

                if (typeA > typeB)
                {
                    std::swap(bodyIndexA, bodyIndexB);
                    std::swap(typeA, typeB);
                }

                chunkData.pairs((size_t)typeA, (size_t)typeB).emplace_back(bodyIndexA, bodyIndexB);
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
                (this->*func)(vec, chunkData.results);
            }
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleCircle(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real radiusB = radiusPtr[shapeB];

            const Vec2 deltaPosition = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);

            if (squaredDistance >= radiusSum * radiusSum)
                continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance == Real(0)) [[unlikely]]
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
                normal,
                depth,
                positionA + normal * radiusA,
                Vec2(),
                1
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-box collision");

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

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

            if (squaredDistance > Real(1e-8))
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

                outCollisionData.emplace_back(
                    indexA, indexB,
                    normal,
                    depth,
                    contactOnCircle,
                    Vec2(),
                    1
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
            if (useX)
                normalLocal = Vec2(sx, Real(0));
            else
                normalLocal = Vec2(Real(0), sy);

            const Vec2 normal = -Vec2{
                cosB * normalLocal.x - sinB * normalLocal.y,
                sinB * normalLocal.x + cosB * normalLocal.y
            };
            const Real depth = radiusA + minPen;
            const Vec2 contactOnCircle = positionA + normal * radiusA;

            outCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contactOnCircle,
                Vec2(),
                1
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-polygon collision");

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];

            const VerticesContainer& localPolygonVertices = polyLocalVerticesPtr[shapeB];
            const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.data();
            const size_t vertexCount = localPolygonVertices.size();
            if (vertexCount < 3) [[unlikely]] continue;

            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            const Vec2 circleLocal = {
                glm::dot(positionA - positionB, rightB),
                glm::dot(positionA - positionB, upB)
            };

            auto edgeOutwardNormal = [&](uint32_t edgeIndex) -> Vec2
                {
                    const Vec2 v0 = localVerts[edgeIndex];
                    const Vec2 v1 = localVerts[(edgeIndex + 1) % uint32_t(vertexCount)];
                    const Vec2 edge = v1 - v0;

                    Vec2 n = Vec2{ edge.y, -edge.x };
                    const Real len2 = glm::dot(n, n);
                    if (len2 > Real(1e-12))
                        n *= Real(1) / std::sqrt(len2);
                    return n;
                };

            auto closestPointOnSegment = [](const Vec2& p, const Vec2& a, const Vec2& b) -> Vec2
                {
                    const Vec2 ab = b - a;
                    const Real denom = glm::dot(ab, ab);
                    if (denom <= Real(1e-12)) return a;
                    const Real t = glm::dot(p - a, ab) / denom;
                    return a + ab * glm::clamp(t, Real(0), Real(1));
                };

            Real bestDist2 = std::numeric_limits<Real>::max();
            Vec2 bestPointLocal;
            uint32_t bestEdgeIndex = 0;

            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                const Vec2 a = localVerts[i];
                const Vec2 b = localVerts[(i + 1) % uint32_t(vertexCount)];
                const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                const Vec2 d = q - circleLocal;
                const Real dist2 = glm::dot(d, d);

                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    bestPointLocal = q;
                    bestEdgeIndex = i;
                }
            }

            if (bestDist2 >= radiusA * radiusA) continue;

            Vec2 normalLocal;
            if (bestDist2 > Real(1e-8))
            {
                const Real invDist = Real(1) / std::sqrt(bestDist2);
                normalLocal = (bestPointLocal - circleLocal) * invDist;
            }
            else
            {
                normalLocal = edgeOutwardNormal(bestEdgeIndex);
                flipSignIfNegative(normalLocal, glm::dot(circleLocal - bestPointLocal, normalLocal));
            }

            Vec2 normal = {
                rightB.x * normalLocal.x + upB.x * normalLocal.y,
                rightB.y * normalLocal.x + upB.y * normalLocal.y
            };

            flipSignIfNegative(normal, glm::dot(positionB - positionA, normal));

            const Real depth = radiusA - std::sqrt(bestDist2);
            const Vec2 contactOnCircle = positionA + normal * radiusA;

            outCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contactOnCircle,
                Vec2(),
                1
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(
        const std::vector<BodyPair>& pairs,
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

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

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

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

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

            Vec2 normal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxis;

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

            auto clipSegment = [](Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal) -> bool
                {
                    const Real d1 = glm::dot(p1 - planePoint, planeNormal);
                    const Real d2 = glm::dot(p2 - planePoint, planeNormal);
                    if (d1 >= 0 && d2 >= 0) return false; // both inside
                    if (d1 < 0 && d2 < 0) return true;    // both outside
                    const Vec2 dir = p2 - p1;
                    const Real t = d1 / (d1 - d2);
                    const Vec2 intersect = p1 + dir * t;
                    if (d1 >= 0) p2 = intersect;
                    else         p1 = intersect;
                    return false;
                };

            Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };
            if (clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir)) continue;
            if (clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir)) continue;

            Vec2 contacts[2];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);
            for (uint32_t i = 0; i < 2; i++)
            {
                if (glm::dot(clipped[i], refNormal) <= refPlaneDist + Real(1e-5))
                    contacts[contactCount++] = clipped[i];
            }
            if (contactCount == 0) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contacts[0],
                contacts[1],
                contactCount
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-polygon collision");

        enum class SATAxis : uint32_t
        {
            BOX_RIGHT,
            BOX_UP,
            POLY_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        auto projectVerticesOnAxis = [](const std::vector<Vec2>& vertices, const Vec2 axis, Real& minOut, Real& maxOut)
            {
                minOut =  std::numeric_limits<Real>::max();
                maxOut = -std::numeric_limits<Real>::max();

                for (const Vec2& v : vertices)
                {
                    const Real p = glm::dot(v, axis);
                    minOut = std::fmin(minOut, p);
                    maxOut = std::fmax(maxOut, p);
                }
            };

        auto clipSegment = [](Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal) -> bool
            {
                const Real d1 = glm::dot(p1 - planePoint, planeNormal);
                const Real d2 = glm::dot(p2 - planePoint, planeNormal);

                if (d1 >= 0 && d2 >= 0) return false; // Both inside.
                if (d1 < 0 && d2 < 0) return true;    // Both outside.

                const Vec2 dir = p2 - p1;
                const Real t = d1 / (d1 - d2);
                const Vec2 intersect = p1 + dir * t;

                if (d1 >= 0) p2 = intersect;
                else         p1 = intersect;

                return false;
            };

        static thread_local std::vector<Vec2> polyWorldVerts;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];

            const VerticesContainer& localPolygonVertices = polyLocalVerticesPtr[shapeB];
            const Vec2* localVerts = localPolygonVertices.data();
            const size_t vertexCount = localPolygonVertices.size();
            if (vertexCount < 3) [[unlikely]] continue;

            const Vec2 rightA = {  cosA, sinA };
            const Vec2 upA    = { -sinA, cosA };
            const Vec2 rightB = {  cosB, sinB };
            const Vec2 upB    = { -sinB, cosB };

            polyWorldVerts.resize(vertexCount);

            for (size_t i = 0; i < vertexCount; i++)
            {
                const Vec2 v = localVerts[i];
                polyWorldVerts[i] = positionB + rightB * v.x + upB * v.y;
            }

            auto polygonEdgeNormal = [&](uint32_t edgeIndex) -> Vec2
                {
                    const Vec2 p0 = polyWorldVerts[edgeIndex];
                    const Vec2 p1 = polyWorldVerts[(edgeIndex + 1) % uint32_t(vertexCount)];
                    const Vec2 edge = p1 - p0;

                    Vec2 n = Vec2{ edge.y, -edge.x };
                    const Real len2 = glm::dot(n, n);
                    if (len2 > Real(1e-12))
                    {
                        n *= Real(1) / std::sqrt(len2);
                    }
                    return n;
                };

            const Vec2 centerDelta = positionB - positionA;

            Vec2 normal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxis = SATAxis::BOX_RIGHT;
            uint32_t bestAxisIndex = 0;

            auto testAxis = [&](Vec2 axis, SATAxis axisType, uint32_t axisIndex) -> bool
                {
                    const Real axisLenSquared = glm::dot(axis, axis);
                    if (axisLenSquared <= Real(1e-12)) return true;

                    axis *= Real(1) / std::sqrt(axisLenSquared);

                    const Real boxRadius =
                        halfWidthA * std::fabs(glm::dot(axis, rightA)) +
                        halfHeightA * std::fabs(glm::dot(axis, upA));

                    Real polyMin, polyMax;
                    projectVerticesOnAxis(polyWorldVerts, axis, polyMin, polyMax);

                    const Real boxCenterProj = glm::dot(positionA, axis);
                    const Real boxMin = boxCenterProj - boxRadius;
                    const Real boxMax = boxCenterProj + boxRadius;

                    const Real overlap = std::fmin(boxMax, polyMax) - std::fmax(boxMin, polyMin);
                    if (overlap < Real(0)) return false;

                    if (overlap < depth)
                    {
                        depth = overlap;
                        normal = axis;
                        flipSignIfNegative(normal, glm::dot(centerDelta, normal));

                        bestAxis = axisType;
                        bestAxisIndex = axisIndex;
                    }

                    return true;
                };

            if (!testAxis(rightA, SATAxis::BOX_RIGHT, 0)) continue;
            if (!testAxis(upA, SATAxis::BOX_UP, 0)) continue;

            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                Vec2 axis = polygonEdgeNormal(i);
                if (!testAxis(axis, SATAxis::POLY_EDGE, i)) goto nextPair;
            }

            const bool refIsBox = bestAxis != SATAxis::POLY_EDGE;

            Vec2 refNormal;
            Vec2 refFaceCenter;
            Vec2 sideDir;
            Vec2 refEdgeStart;
            Vec2 refEdgeEnd;

            if (refIsBox)
            {
                refNormal = normal;

                const Real dotX = glm::dot(refNormal, rightA);
                const Real dotY = glm::dot(refNormal, upA);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real sign = std::copysign(Real(1), useX ? dotX : dotY);

                if (useX)
                {
                    refFaceCenter = positionA + rightA * (sign * halfWidthA);
                    sideDir = upA;
                }
                else
                {
                    refFaceCenter = positionA + upA * (sign * halfHeightA);
                    sideDir = rightA;
                }

                const Vec2 edgeOffset = sideDir * (useX ? halfHeightA : halfWidthA);
                refEdgeStart = refFaceCenter + edgeOffset;
                refEdgeEnd = refFaceCenter - edgeOffset;
            }
            else
            {
                refNormal = -normal;

                const uint32_t i0 = bestAxisIndex;
                const uint32_t i1 = (i0 + 1) % uint32_t(vertexCount);

                refEdgeStart = polyWorldVerts[i0];
                refEdgeEnd = polyWorldVerts[i1];
                refFaceCenter = (refEdgeStart + refEdgeEnd) * Real(0.5);

                const Vec2 edge = refEdgeEnd - refEdgeStart;
                const Real edgeLen = std::sqrt(glm::dot(edge, edge));
                if (edgeLen <= Real(1e-12)) continue;

                sideDir = edge / -edgeLen;
            }

            Vec2 incEdgeStart, incEdgeEnd;

            if (refIsBox)
            {
                // Incident edge is the polygon edge whose normal is most opposite the reference normal.
                uint32_t incidentEdge = 0;
                Real minDot = std::numeric_limits<Real>::max();

                for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
                {
                    const Vec2 n = polygonEdgeNormal(i);
                    const Real d = glm::dot(refNormal, n);
                    if (d < minDot)
                    {
                        minDot = d;
                        incidentEdge = i;
                    }
                }

                incEdgeStart = polyWorldVerts[incidentEdge];
                incEdgeEnd = polyWorldVerts[(incidentEdge + 1) % uint32_t(vertexCount)];
            }
            else
            {
                // Incident edge is the box face opposite the reference normal.
                const Real dotX = glm::dot(refNormal, rightA);
                const Real dotY = glm::dot(refNormal, upA);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                Vec2 faceCenter, edgeOffset;
                if (useX)
                {
                    faceCenter = positionA + rightA * (sign * halfWidthA);
                    edgeOffset = upA * halfHeightA;
                }
                else
                {
                    faceCenter = positionA + upA * (sign * halfHeightA);
                    edgeOffset = rightA * halfWidthA;
                }

                incEdgeStart = faceCenter + edgeOffset;
                incEdgeEnd = faceCenter - edgeOffset;
            }

            Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };
            if (clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir)) continue;
            if (clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir)) continue;

            Vec2 contacts[2];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);

            for (uint32_t i = 0; i < 2; i++)
            {
                if (glm::dot(clipped[i], refNormal) <= refPlaneDist + Real(1e-5))
                {
                    contacts[contactCount++] = clipped[i];
                }
            }

            if (contactCount == 0) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contacts[0],
                contacts[1],
                contactCount
            );

        nextPair:
            continue;
        }
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-polygon collision");

        enum class SATAxis : uint32_t
        {
            A_EDGE,
            B_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* ECSTASY_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const VerticesContainer* ECSTASY_RESTRICT polyLocalVerticesPtr = polygons.localVertices;

        auto edgeOutwardNormal = [](const std::vector<Vec2>& verts, uint32_t edgeIndex) -> Vec2
            {
                const Vec2 p0 = verts[edgeIndex];
                const Vec2 p1 = verts[(edgeIndex + 1) % uint32_t(verts.size())];
                const Vec2 edge = p1 - p0;

                Vec2 n = Vec2{ edge.y, -edge.x };
                const Real len2 = glm::dot(n, n);
                if (len2 > Real(1e-12))
                    n *= Real(1) / std::sqrt(len2);
                return n;
            };

        auto projectVerticesOnAxis = [](const std::vector<Vec2>& vertices, const Vec2& axis, Real& minOut, Real& maxOut)
            {
                minOut = std::numeric_limits<Real>::max();
                maxOut = -std::numeric_limits<Real>::max();

                for (const Vec2& v : vertices)
                {
                    const Real p = glm::dot(v, axis);
                    minOut = std::fmin(minOut, p);
                    maxOut = std::fmax(maxOut, p);
                }
            };

        auto clipSegment = [](Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal) -> bool
            {
                const Real d1 = glm::dot(p1 - planePoint, planeNormal);
                const Real d2 = glm::dot(p2 - planePoint, planeNormal);

                if (d1 >= 0 && d2 >= 0) return false; // both inside
                if (d1 < 0 && d2 < 0) return true;    // both outside

                const Vec2 dir = p2 - p1;
                const Real t = d1 / (d1 - d2);
                const Vec2 intersect = p1 + dir * t;

                if (d1 >= 0) p2 = intersect;
                else         p1 = intersect;

                return false;
            };

        static thread_local std::vector<Vec2> worldVertsA;
        static thread_local std::vector<Vec2> worldVertsB;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const VerticesContainer& localVertsAContainer = polyLocalVerticesPtr[shapeA];
            const VerticesContainer& localVertsBContainer = polyLocalVerticesPtr[shapeB];

            const size_t countA = localVertsAContainer.size();
            const size_t countB = localVertsBContainer.size();
            if (countA < 3 || countB < 3) [[unlikely]] continue;

            const Vec2* localVertsA = localVertsAContainer.data();
            const Vec2* localVertsB = localVertsBContainer.data();

            const Vec2 rightA = { cosA,  sinA };
            const Vec2 upA = { -sinA, cosA };
            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            worldVertsA.resize(countA);
            worldVertsB.resize(countB);

            for (size_t i = 0; i < countA; i++)
            {
                const Vec2 v = localVertsA[i];
                worldVertsA[i] = positionA + rightA * v.x + upA * v.y;
            }

            for (size_t i = 0; i < countB; i++)
            {
                const Vec2 v = localVertsB[i];
                worldVertsB[i] = positionB + rightB * v.x + upB * v.y;
            }

            const Vec2 centerDelta = positionB - positionA;

            Vec2 normal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::A_EDGE;
            uint32_t bestAxisIndex = 0;

            auto sat = [&](
                const std::vector<Vec2>& vertsA, const std::vector<Vec2>& vertsB,
                Vec2 axis, SATAxis axisType, uint32_t axisIndex) -> bool
            {
                const Real axisLenSquared = glm::dot(axis, axis);
                if (axisLenSquared <= Real(1e-12)) return true;

                axis *= Real(1) / std::sqrt(axisLenSquared);

                Real minA, maxA;
                Real minB, maxB;
                projectVerticesOnAxis(vertsA, axis, minA, maxA);
                projectVerticesOnAxis(vertsB, axis, minB, maxB);

                const Real overlap = std::fmin(maxA, maxB) - std::fmax(minA, minB);
                if (overlap < Real(0)) return false;

                if (overlap < depth)
                {
                    depth = overlap;
                    normal = axis;
                    flipSignIfNegative(normal, glm::dot(centerDelta, normal));

                    bestAxisType = axisType;
                    bestAxisIndex = axisIndex;
                }

                return true;
            };

            for (uint32_t i = 0; i < uint32_t(countA); i++)
            {
                Vec2 axis = edgeOutwardNormal(worldVertsA, i);
                if (!sat(worldVertsA, worldVertsB, axis, SATAxis::A_EDGE, i))
                    goto nextPair;
            }

            for (uint32_t i = 0; i < uint32_t(countB); i++)
            {
                Vec2 axis = edgeOutwardNormal(worldVertsB, i);
                if (!sat(worldVertsA, worldVertsB, axis, SATAxis::B_EDGE, i))
                    goto nextPair;
            }

            {
                const bool refIsA = bestAxisType == SATAxis::A_EDGE;

                const std::vector<Vec2>& refVerts = refIsA ? worldVertsA : worldVertsB;
                const std::vector<Vec2>& incVerts = refIsA ? worldVertsB : worldVertsA;

                Vec2 refNormal = refIsA ? normal : -normal;
                Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;

                {
                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = (i0 + 1) % uint32_t(refVerts.size());

                    refEdgeStart = refVerts[i0];
                    refEdgeEnd = refVerts[i1];
                    refFaceCenter = (refEdgeStart + refEdgeEnd) * Real(0.5);

                    const Vec2 edge = refEdgeEnd - refEdgeStart;
                    const Real edgeLen = std::sqrt(glm::dot(edge, edge));
                    if (edgeLen <= Real(1e-12)) goto nextPair;

                    sideDir = edge / edgeLen;

                    const Vec2 expectedOutward = Vec2{ edge.y, -edge.x };
                    if (glm::dot(expectedOutward, refNormal) < Real(0))
                    {
                        std::swap(refEdgeStart, refEdgeEnd);
                        sideDir = -sideDir;
                    }
                }

                // Incident edge: pick the edge whose outward normal is most anti-parallel to the reference normal.
                uint32_t incidentEdgeIndex = 0;
                Real minDot = std::numeric_limits<Real>::max();

                for (uint32_t i = 0; i < uint32_t(incVerts.size()); i++)
                {
                    const Vec2 p0 = incVerts[i];
                    const Vec2 p1 = incVerts[(i + 1) % uint32_t(incVerts.size())];
                    const Vec2 edge = p1 - p0;
                    Vec2 n = Vec2{ edge.y, -edge.x };
                    const Real len2 = glm::dot(n, n);
                    if (len2 <= Real(1e-12)) continue;
                    n *= Real(1) / std::sqrt(len2);

                    const Real d = glm::dot(refNormal, n);
                    if (d < minDot)
                    {
                        minDot = d;
                        incidentEdgeIndex = i;
                    }
                }

                Vec2 incEdgeStart = incVerts[incidentEdgeIndex];
                Vec2 incEdgeEnd = incVerts[(incidentEdgeIndex + 1) % uint32_t(incVerts.size())];

                Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };
                if (clipSegment(clipped[0], clipped[1], refEdgeStart, sideDir)) goto nextPair;
                if (clipSegment(clipped[0], clipped[1], refEdgeEnd,  -sideDir)) goto nextPair;

                const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);
                const Real eps = Real(1e-5);

                std::array<Vec2, 2> contacts;
                uint32_t contactCount = 0;

                for (uint32_t i = 0; i < 2; i++)
                {
                    if (glm::dot(clipped[i], refNormal) <= refPlaneDist + eps)
                    {
                        contacts[contactCount++] = clipped[i];
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    normal,
                    depth,
                    contacts[0],
                    contacts[1],
                    contactCount
                );
            }
        nextPair:
            continue;
        }
    }
}