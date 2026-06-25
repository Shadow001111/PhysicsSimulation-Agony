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
        const BoxSoAViewer& boxes
    )
    {
        this->bodies = bodies;
        this->circles = circles;
        this->boxes = boxes;
    }

    const std::vector<BodyCollisionData>& NarrowPhaseCollisionDetector::findCollisions(const std::vector<BodyPair>& bodyPairs)
    {
        TRACY_SCOPE_N("Narrow phase");

        allCollisionData.clear();

        if (bodyPairs.empty())
        {
            return allCollisionData;
        }

        allCollisionData.reserve(bodyPairs.size());

        const bool useThreading = true;
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
        cd.clear();
        processPairs(bodyPairs, 0, bodyPairs.size(), cd);
        allCollisionData.swap(cd.results);
    }

    void NarrowPhaseCollisionDetector::findCollisionsMultiThreaded(const std::vector<BodyPair>& bodyPairs)
    {
        TRACY_SCOPE_N("Multi-threaded narrow phase");

        constexpr size_t LOAD_BALANCING_FACTOR = 2;
        auto& threadPool = Threading::getGlobalThreadPool();

        auto [chunkCount, chunkSize] = Ecstasy::Threading::ParallelForRangeExecutor::getChunkCountAndSize(
            threadPool, bodyPairs.size(), LOAD_BALANCING_FACTOR);

        chunks.resize(chunkCount);

        std::vector<std::future<void>> futures;
        futures.reserve(chunkCount);

        size_t chunkIndex = 0;
        for (size_t start = 0; start < bodyPairs.size(); start += chunkSize, ++chunkIndex)
        {
            size_t end = std::min(start + chunkSize, bodyPairs.size());
            ChunkData& cd = chunks[chunkIndex];
            futures.emplace_back(threadPool.enqueueFuture([this, &bodyPairs, start, end, &cd]()
                {
                    cd.clear();
                    processPairs(bodyPairs, start, end, cd);
                }));
        }

        {
            TRACY_SCOPE_N("Wait for workers and combine");

            for (size_t i = 0; i < chunkCount; i++)
            {
                futures[i].get();

                {
                    const auto& results = chunks[i].results;
                    TRACY_SCOPE_N("Combine");
                    allCollisionData.insert(
                        allCollisionData.end(),
                        results.begin(),
                        results.end()
                    );
                }
            }
        }
    }

    void NarrowPhaseCollisionDetector::processPairs(const std::vector<BodyPair>& pairs, size_t start, size_t end, ChunkData& chunkData)
    {
        TRACY_SCOPE_N("Process pair range");

        const BodyType* ECSTASY_RESTRICT bodyTypePtr = bodies.bodyType;
        const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies.isStatic;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = start; i < end; i++)
            {
                auto [bodyIndexA, bodyIndexB] = pairs[i];
                if (isStaticPtr[bodyIndexA] && isStaticPtr[bodyIndexB]) [[unlikely]]
                    continue;

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
        for (size_t i = 0; i < BODY_TYPE_COUNT; i++)
        {
            for (size_t j = i; j < BODY_TYPE_COUNT; j++)
            {
                auto& vec = chunkData.pairs(i, j);
                if (!vec.empty())
                {
                    CollisionFunc func = collisionFuncs(i, j);
                    (this->*func)(vec, chunkData.results);
                }
            }
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleCircle(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.truePositionX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.truePositionY;
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
                normal.x = 1.0;
                normal.y = 0.0;
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

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.truePositionX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.truePositionY;
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

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.truePositionX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.truePositionY;
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
            Real depth = FLT_MAX;
            SATAxis bestAxis;

            auto sat = [&](Real radiusSum, Real centerDeltaOnAxis, Vec2 axis, SATAxis axisType) -> bool
                {
                    const Real overlap = radiusSum - std::fabsf(centerDeltaOnAxis);
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
            for (uint32_t i = 0; i < 2; ++i)
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
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(
        const std::vector<BodyPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
    }
}