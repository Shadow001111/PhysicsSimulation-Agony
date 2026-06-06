#include "NarrowPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

namespace PS_AGONY
{
    struct Vector2AndSqDistance
    {
        Vec2 vector;
        Real squaredDistance;
    };

    /*static inline Vector2AndSqDistance findClosestPointOnSegment(const Vec2& start, const Vec2& end, const Vec2& point)
    {
        const Vec2 startToEnd = end - start;
        const Vec2 startToPoint = point - start;

        const Real d = glm::dot(startToEnd, startToPoint) / glm::dot(startToEnd, startToEnd);

        const Vec2 closest = start + startToEnd * std::clamp(d, Real(0), Real(1));

        const Vec2 deltaPosition = closest - point;

        Vector2AndSqDistance result;
        result.vector = closest;
        result.squaredDistance = glm::dot(deltaPosition, deltaPosition);

        return result;
    }*/

    static __forceinline void flip_sign_if_negative(glm::vec2& v, const float& sign)
    {
        constexpr uint32_t signBit = 1u << 31;
        const uint32_t sign_mask = reinterpret_cast<const uint32_t&>(sign) & signBit;
        reinterpret_cast<uint32_t&>(v.x) ^= sign_mask;
        reinterpret_cast<uint32_t&>(v.y) ^= sign_mask;
    }

    static __forceinline void flip_sign_if_negative(glm::dvec2& v, const double& sign)
    {
        constexpr uint64_t signBit = 1ull << 63;
        const uint64_t sign_mask = reinterpret_cast<const uint64_t&>(sign) & signBit;
        reinterpret_cast<uint64_t&>(v.x) ^= sign_mask;
        reinterpret_cast<uint64_t&>(v.y) ^= sign_mask;
    }


    NarrowPhaseCollisionDetector::NarrowPhaseCollisionDetector()
    {
        // Circle.
        bodyPairVectorMatrix(0, 0) = &circleCirclePairs;
        bodyPairVectorMatrix(0, 1) = &circleBoxPairs;
        //bodyPairVectorMatrix(0, 2) = &circlePolygonPairs;

        // Box.
        bodyPairVectorMatrix(1, 1) = &boxBoxPairs;
        //bodyPairVectorMatrix(1, 2) = &boxPolygonPairs;

        // Polygon.
        //bodyPairVectorMatrix(2, 2) = &polygonPolygonPairs;
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

        // Prepare.
        allCollisionData.clear();
        circleCirclePairs.clear();
        circleBoxPairs.clear();
        boxBoxPairs.clear();

        const size_t bodyPairCount = bodyPairs.size();
        if (bodyPairCount == 0) return allCollisionData; // No pairs to check.

        // Reserve.
        allCollisionData.reserve(bodyPairCount);
        circleCirclePairs.reserve(bodyPairCount);
        circleBoxPairs.reserve(bodyPairCount);
        boxBoxPairs.reserve(bodyPairCount);

        // Sort body pairs by type.
        {
            TRACY_SCOPE_N("Sort pairs");

            const BodyType* CORE_RESTRICT bodyTypePtr = bodies.bodyType;

            for (auto [bodyIndexA, bodyIndexB] : bodyPairs)
            {
                BodyType bodyTypeA = bodyTypePtr[bodyIndexA];
                BodyType bodyTypeB = bodyTypePtr[bodyIndexB];

                // Ensure bodyTypeA <= bodyTypeB.
                if (bodyTypeA > bodyTypeB)
                {
                    std::swap(bodyIndexA, bodyIndexB);
                    std::swap(bodyTypeA, bodyTypeB);
                }

                bodyPairVectorMatrix(static_cast<size_t>(bodyTypeA), static_cast<size_t>(bodyTypeB))->emplace_back(bodyIndexA, bodyIndexB);
            }
        }

        // Process each pair type separately.
        collisionCircleCircle();
        collisionCircleBox();
        collisionBoxBox();

		return allCollisionData;
	}

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = 0;

        total += PS_AGONY::getVectorMemoryUsage(allCollisionData);

        total += PS_AGONY::getVectorMemoryUsage(circleCirclePairs);
        total += PS_AGONY::getVectorMemoryUsage(circleBoxPairs);
        total += PS_AGONY::getVectorMemoryUsage(boxBoxPairs);

        return total;
    }

    void NarrowPhaseCollisionDetector::collisionCircleCircle()
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        for (auto [indexA, indexB] : circleCirclePairs)
        {
            // Gather data.
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real radiusB = radiusPtr[shapeB];

            // Delta position.
            const Vec2 deltaPosition = positionB - positionA;

            // Radius sum.
            const Real radiusSum = radiusA + radiusB;

            // Distance.
            const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);
            if (squaredDistance >= radiusSum * radiusSum)
            {
                continue;
            }
            const Real distance = std::sqrt(squaredDistance);

            // Depth.
            const Real depth = radiusSum - distance;

            // Normal.
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

            // Result.
            allCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                positionA + normal * radiusA,
                Vec2(),
                1
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox()
    {
        TRACY_SCOPE_N("Circle-box collision");

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : circleBoxPairs)
        {
            // Gather data.
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const BodyIndex shapeA = shapeIndexPtr[indexA];
            const BodyIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];

            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];


            // Get axes.
            const Vec2 right = { cosB, sinB };
            const Vec2 up = { -sinB, cosB };

            // Circle center in box local space.
            const Vec2 d = positionA - positionB;
            const Vec2 circleLocalPosition = {
                glm::dot(d, right),
                glm::dot(d, up)
            };

            // Closest point on box to the circle, in local space.
            const Vec2 closestLocal = {
                glm::clamp(circleLocalPosition.x, -halfWidthB,  halfWidthB),
                glm::clamp(circleLocalPosition.y, -halfHeightB, halfHeightB)
            };

            // Distance.
            const Vec2 deltaLocal = closestLocal - circleLocalPosition;
            const Real squaredDistance = glm::dot(deltaLocal, deltaLocal);

            if (squaredDistance >= radiusA * radiusA) continue;

            //
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

                allCollisionData.emplace_back(
                    indexA, indexB,
                    normal,
                    depth,
                    contactOnCircle,
                    Vec2(),
                    1
                );
                continue;
            }

            // Circle center is inside the box (or extremely close to an edge/corner).
            // Choose the nearest face in local space.
            const Real dx = halfWidthB - std::abs(circleLocalPosition.x);
            const Real dy = halfHeightB - std::abs(circleLocalPosition.y);
            const bool useX = dx < dy;
            const Real minPen = useX ? dx : dy;

            const Real sx = std::copysign(Real(1), circleLocalPosition.x);
            const Real sy = std::copysign(Real(1), circleLocalPosition.y);

            Vec2 normalLocal;
            if (useX)
            {
                normalLocal = Vec2(sx, Real(0));
            }
            else
            {
                normalLocal = Vec2(Real(0), sy);
            }

            const Vec2 normal = -Vec2{
                cosB * normalLocal.x - sinB * normalLocal.y,
                sinB * normalLocal.x + cosB * normalLocal.y
            };
            const Real depth = radiusA + minPen;

            const Vec2 contactOnCircle = positionA + normal * radiusA;

            allCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contactOnCircle,
                Vec2(),
                1
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon()
    {
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox()
    {
        TRACY_SCOPE_N("Box-box collision");

        enum class SATAxis : uint32_t
        {
            A_RIGHT,
            A_UP,
            B_RIGHT,
            B_UP
        };

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : boxBoxPairs)
        {
            // Gather data.
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

            // Get axes.
            const Vec2 rightA = { cosA,  sinA };
            const Vec2 upA =    { -sinA, cosA };

            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB =    { -sinB, cosB };

            // Relative rotation.
            const Real absRelativeCos = std::abs(cosA * cosB + sinA * sinB);
            const Real absRelativeSin = std::abs(cosA * sinB - sinA * cosB);

            // Center delta.
            const Vec2 centerDelta = positionB - positionA;
            const Real centerDeltaOnRightA = glm::dot(centerDelta, rightA);
            const Real centerDeltaOnUpA = glm::dot(centerDelta, upA);
            const Real centerDeltaOnRightB = glm::dot(centerDelta, rightB);
            const Real centerDeltaOnUpB = glm::dot(centerDelta, upB);

            // Projection.
            const Real projectedRadiusBOnRightA = halfWidthB * absRelativeCos + halfHeightB * absRelativeSin;
            const Real projectedRadiusBOnUpA = halfWidthB * absRelativeSin + halfHeightB * absRelativeCos;
            const Real projectedRadiusAOnRightB = halfWidthA * absRelativeCos + halfHeightA * absRelativeSin;
            const Real projectedRadiusAOnUpB = halfWidthA * absRelativeSin + halfHeightA * absRelativeCos;

            // SAT: find axis of minimum penetration.
            Vec2 normal;
            Real depth = FLT_MAX;
            SATAxis bestAxis;

            auto sat = [&](Real radiusSum, Real centerDeltaOnAxis, Vec2 axis, SATAxis axisType) -> bool
                {
                    // radiusSum = radiusA + radiusB.
                    const Real overlap = radiusSum - std::fabsf(centerDeltaOnAxis);
                    if (overlap < Real(0)) return false;
                    if (overlap < depth)
                    {
                        depth = overlap;
                        normal = axis;
                        flip_sign_if_negative(normal, centerDeltaOnAxis);
                        bestAxis = axisType;
                    }
                    return true;
                };

            if (!sat(projectedRadiusBOnRightA + halfWidthA,  centerDeltaOnRightA, rightA, SATAxis::A_RIGHT)) continue;
            if (!sat(projectedRadiusBOnUpA    + halfHeightA, centerDeltaOnUpA,    upA,    SATAxis::A_UP))    continue;
            if (!sat(projectedRadiusAOnRightB + halfWidthB,  centerDeltaOnRightB, rightB, SATAxis::B_RIGHT)) continue;
            if (!sat(projectedRadiusAOnUpB    + halfHeightB, centerDeltaOnUpB,    upB,    SATAxis::B_UP))    continue;

            // Identify reference and incident boxes.
            const bool refIsA = bestAxis < SATAxis::B_RIGHT;

            Vec2 positionRef;
            Real halfWidthRef, halfHeightRef;
            Vec2 rightRef, upRef;

            Vec2 positionInc;
            Real halfWidthInc, halfHeightInc;
            Vec2 rightInc, upInc;

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

            //
            Vec2 refEdgeStart, refEdgeEnd, refFaceCenter;
            Vec2 sideDir;
            {
                // Project reference normal on reference's local axes.
                const Real dotX = glm::dot(refNormal, rightRef);
                const Real dotY = glm::dot(refNormal, upRef);

                // Choose the face with outward normal matching the collision direction.
                const bool useX = std::abs(dotX) > std::abs(dotY);

                const Real sign = std::copysign(Real(1), useX ? dotX : dotY);

                // Reference face edge endpoints (world).
                Vec2 edgeOffset;
                if (useX)
                {
                    refFaceCenter = positionRef + rightRef * (sign * halfWidthRef);
                    edgeOffset = upRef * halfHeightRef;
                    sideDir = upRef;
                }
                else
                {
                    refFaceCenter = positionRef + upRef * (sign * halfHeightRef);
                    edgeOffset = rightRef * halfWidthRef;
                    sideDir = rightRef;
                }
                refEdgeStart = refFaceCenter + edgeOffset;
                refEdgeEnd = refFaceCenter - edgeOffset;
            }
            Vec2 incEdgeStart, incEdgeEnd;
            {
                // Project reference normal on incidental's local axes.
                const Real dotX = glm::dot(refNormal, rightInc);
                const Real dotY = glm::dot(refNormal, upInc);

                // Choose the face with outward normal matching the collision direction.
                const bool useX = std::abs(dotX) > std::abs(dotY);

                const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                // Reference face edge endpoints (world).
                Vec2 faceCenter;
                Vec2 edgeOffset;
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

            // Clip incident edge against reference side planes.
            // Note: Function return either 0 or 2, so I made it use bool. Returns bool on fail.
            auto clipSegment = [](Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal) -> bool
                {
                    const Real d1 = glm::dot(p1 - planePoint, planeNormal);
                    const Real d2 = glm::dot(p2 - planePoint, planeNormal);

                    if (d1 >= 0 && d2 >= 0)
                    {
                        // out1 = p1; out2 = p2; 
                        return false; // 2
                    }
                    if (d1 < 0 && d2 < 0) return true; // 0

                    // One point inside, one outside -> compute intersection.
                    const Vec2 dir = p2 - p1;
                    const Real t = d1 / (d1 - d2); // d1 - d2 != 0
                    const Vec2 intersect = p1 + dir * t;

                    if (d1 >= 0)
                    {
                        //out1 = p1;
                        p2 = intersect;
                    }
                    else
                    {
                        p1 = intersect;
                        //out2 = p2;
                    }
                    return false; // 2
                };

            // Note: Can put 'clipped' instead of inc edge variables to set to array directly above. I tried, but it didn't give any results.
            Vec2 clipped[2] = { incEdgeStart, incEdgeEnd };

            // Clip against first side plane.
            bool clipFail = clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir);
            if (clipFail) continue;

            // Clip against second side plane.
            clipFail = clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir);
            if (clipFail) continue;

            // Keep points that lie behind the reference face plane.
            Vec2 contacts[2];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);

            for (uint32_t i = 0; i < 2; i++)
            {
                const Real pointDist = glm::dot(clipped[i], refNormal);
                if (pointDist <= refPlaneDist + Real(1e-5))
                {
                    contacts[contactCount++] = clipped[i];
                }
            }

            if (contactCount == 0) continue;

            // Store final collision data.
            allCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contacts[0],
                contacts[1],
                contactCount
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon()
    {
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon()
    {
    }
}
