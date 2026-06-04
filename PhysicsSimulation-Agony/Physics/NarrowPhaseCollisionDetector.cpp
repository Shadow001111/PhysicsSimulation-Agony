#include "NarrowPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
    const NarrowPhaseCollisionDetector::CollisionFunc
        NarrowPhaseCollisionDetector::collisionFunctions[static_cast<size_t>(BodyType::COUNT)][static_cast<size_t>(BodyType::COUNT)] =
    {
        /* Row for Circle (0) */
        {
            &NarrowPhaseCollisionDetector::collisionCircleCircle,
            &NarrowPhaseCollisionDetector::collisionCircleBox,
            &NarrowPhaseCollisionDetector::collisionCirclePolygon
        },
        /* Row for Box (1) */
        {
            nullptr, // Box-Circle
            &NarrowPhaseCollisionDetector::collisionBoxBox,
            &NarrowPhaseCollisionDetector::collisionBoxPolygon
        },
        /* Row for Polygon (2) */
        {
            nullptr, // Polygon-Circle
            nullptr, // Polygon-Box
            &NarrowPhaseCollisionDetector::collisionPolygonPolygon
        }
    };

    static inline Vec2 projectBox(
        const Vec2& center,
        const Vec2& right,
        const Vec2& up,
        const Real halfWidth,
        const Real halfHeight,
        const Vec2& axis)
    {
        const Real c = glm::dot(center, axis);
        const Real r =
            halfWidth  * std::abs(glm::dot(right, axis)) +
            halfHeight * std::abs(glm::dot(up,    axis));
        return { c - r, c + r };
    }

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
        narrowCollisionData.clear();

        if (bodyPairs.empty()) return narrowCollisionData; // No pairs to check.

        narrowCollisionData.reserve(bodyPairs.size());

        // Real search.
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

            // Retrieve the collision function from the matrix.
            auto func = collisionFunctions[static_cast<size_t>(bodyTypeA)][static_cast<size_t>(bodyTypeB)];
            
            // Call the func.
            (this->*func)(bodyIndexA, bodyIndexB);
        }

		return narrowCollisionData;
	}

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = 0;

        total += PS_AGONY::getVectorMemoryUsage(narrowCollisionData);

        return total;
    }

    void NarrowPhaseCollisionDetector::collisionCircleCircle(BodyIndex indexA, BodyIndex indexB)
    {
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

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
            return;
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
        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            positionA + normal * radiusA,   // Contact 1.
            Vec2(),                         // Contact 2.
            1                               // Single contact.
        );
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox(BodyIndex indexA, BodyIndex indexB)
    {
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight;

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
        const Vec2 right = {  cosB, sinB };
        const Vec2 up    = { -sinB, cosB };

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
        
        if (squaredDistance >= radiusA * radiusA)
        {
            return;
        }

        //
        auto toWorldRotation = [&](const Vec2& pLocal) -> Vec2 {
            return {
                cosB * pLocal.x - sinB * pLocal.y,
                sinB * pLocal.x + cosB * pLocal.y
            };
            };

        if (squaredDistance > Real(1e-8)) [[likely]]
        {
            const Real distance = std::sqrt(squaredDistance);
            const Real invDistance = Real(1) / distance;

            const Vec2 normalLocal = deltaLocal * invDistance;
            const Vec2 normal = toWorldRotation(normalLocal);
            
            const Real depth = radiusA - distance;

            const Vec2 contactOnCircle = positionA + normal * radiusA;

            narrowCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contactOnCircle,
                Vec2(),         
                1               
            );
            return;
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
        Vec2 pointLocal;
        if (useX)
        {
            normalLocal = Vec2(sx, Real(0));
            pointLocal  = Vec2(sx * halfWidthB, circleLocalPosition.y);
        }
        else
        {
            normalLocal = Vec2(Real(0), sy);
            pointLocal  = Vec2(circleLocalPosition.x, sy * halfHeightB);
        }

        const Vec2 normal = toWorldRotation(normalLocal);
        const Real depth = radiusA + minPen;

        const Vec2 contactOnCircle = positionA + normal * radiusA;

        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            contactOnCircle,
            Vec2(),         
            1               
        );
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(BodyIndex indexA, BodyIndex indexB)
    {
        constexpr Real secondContactThreshold = Real(1e-4);

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight;

        // Gather data.
        const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
        const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

        const Real cosA = rotationCosPtr[indexA];
        const Real sinA = rotationSinPtr[indexA];
        const Real cosB = rotationCosPtr[indexB];
        const Real sinB = rotationSinPtr[indexB];

        const BodyIndex shapeA = shapeIndexPtr[indexA];
        const BodyIndex shapeB = shapeIndexPtr[indexB];

        const Real halfWidthA  = halfWidthPtr [shapeA];
        const Real halfHeightA = halfHeightPtr[shapeA];
        const Real halfWidthB  = halfWidthPtr [shapeB];
        const Real halfHeightB = halfHeightPtr[shapeB];

        // Get axes.
        const Vec2 rightA = { cosA, sinA };
        const Vec2 upA = { -sinA, cosA };

        const Vec2 rightB = { cosB, sinB };
        const Vec2 upB = { -sinB, cosB };

        // Relative rotation.
        const Real absRelativeCos = std::abs(cosA * cosB + sinA * sinB);
        const Real absRelativeSin = std::abs(cosA * sinB - sinA * cosB);

        // Center delta.
        const Vec2 centerDelta = positionB - positionA;
        const Real centerDeltaOnRightA = glm::dot(centerDelta, rightA);
        const Real centerDeltaOnUpA    = glm::dot(centerDelta, upA);
        const Real centerDeltaOnRightB = glm::dot(centerDelta, rightB);
        const Real centerDeltaOnUpB    = glm::dot(centerDelta, upB);

        // Projection.
        const Real projectedRadiusBOnRightA = halfWidthB * absRelativeCos + halfHeightB * absRelativeSin;
        const Real projectedRadiusBOnUpA    = halfWidthB * absRelativeSin + halfHeightB * absRelativeCos;
        const Real projectedRadiusAOnRightB = halfWidthA * absRelativeCos + halfHeightA * absRelativeSin;
        const Real projectedRadiusAOnUpB    = halfWidthA * absRelativeSin + halfHeightA * absRelativeCos;

        // SAT.
        Vec2 normal = {};
        Real depth = FLT_MAX;

        auto sat = [&](Real radiusA, Real radiusB, Real centerDeltaOnAxis, Vec2 axis) -> bool
            {
                const Real overlap = radiusA + radiusB - std::fabsf(centerDeltaOnAxis);
                if (overlap < Real(0)) return false;
                if (overlap < depth)
                {
                    depth = overlap;

                    normal = axis;
                    flip_sign_if_negative(normal, centerDeltaOnAxis);

                }
                return true;
            };

        if (!sat(halfWidthA,  projectedRadiusBOnRightA, centerDeltaOnRightA, rightA)) return;
        if (!sat(halfHeightA, projectedRadiusBOnUpA, centerDeltaOnUpA, upA)) return;
        if (!sat(projectedRadiusAOnRightB, halfWidthB,  centerDeltaOnRightB, rightB)) return;
        if (!sat(projectedRadiusAOnUpB, halfHeightB, centerDeltaOnUpB, upB)) return;

        // Compute 4 vertices.
        Vec2 vertsA[4], vertsB[4];
        {
            const Vec2 a1 = rightA * halfWidthA;
            const Vec2 a2 = upA * halfHeightA;
            const Vec2 b1 = rightB * halfWidthB;
            const Vec2 b2 = upB * halfHeightB;

            vertsA[0] = positionA + a1 + a2;
            vertsA[1] = positionA - a1 + a2;
            vertsA[2] = positionA - a1 - a2;
            vertsA[3] = positionA + a1 - a2;

            vertsB[0] = positionB + b1 + b2;
            vertsB[1] = positionB - b1 + b2;
            vertsB[2] = positionB - b1 - b2;
            vertsB[3] = positionB + b1 - b2;
        }

        // Find up to 2 contact points: closest points from each box's vertices onto the other box's edges.
        Vec2 contact1, contact2;
        uint32_t contactCount = 1;
        Real minDistanceSquared = FLT_MAX;
        Real maxDistanceSquaredBetweenContacts = 0;

        auto testEdgeAgainstVertices = [&](const Vec2 edgeA, const Vec2 edgeB, const Vec2* vertices)
            {
                const Vec2 startToEnd = edgeB - edgeA;
                const Real startToEndSqDistance = glm::dot(startToEnd, startToEnd);
                //if (startToEndSqDistance < Real(1e-16)) [[unlikely]] return;

                const Real startToEndInvSqDistance = Real(1) / startToEndSqDistance;

                for (size_t i = 0; i < 4; i++)
                {
                    const Vec2 vertex = vertices[i];

                    const Vec2 startToPoint = vertex - edgeA;
                    const Real d = glm::dot(startToEnd, startToPoint) * startToEndInvSqDistance;
                    const Vec2 closest = edgeA + startToEnd * std::clamp(d, Real(0), Real(1));
                    const Vec2 deltaPosition = closest - vertex;
                    const Real pointToClosestSqDistance = glm::dot(deltaPosition, deltaPosition);

                    if (std::fabsf(pointToClosestSqDistance - minDistanceSquared) < secondContactThreshold)
                    {
                        // Value contact2 that's furthest away from contact1.
                        const Vec2 diff = closest - contact1;
                        const Real squaredDistance = glm::dot(diff, diff);
                        if (squaredDistance > maxDistanceSquaredBetweenContacts)
                        {
                            maxDistanceSquaredBetweenContacts = squaredDistance;
                            contact2 = closest;
                            contactCount = 2;
                        }
                    }
                    else if (pointToClosestSqDistance < minDistanceSquared)
                    {
                        minDistanceSquared = pointToClosestSqDistance;
                        maxDistanceSquaredBetweenContacts = 0;
                        contact1 = closest;
                        contactCount = 1;
                    }
                }
            };
        for (size_t i = 0; i < 4; i++)
        {
            testEdgeAgainstVertices(vertsA[i], vertsA[(i + 1) & 3], vertsB);
        }
        for (size_t i = 0; i < 4; i++)
        {
            testEdgeAgainstVertices(vertsB[i], vertsB[(i + 1) & 3], vertsA);
        }

        // Result.
        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            contact1,
            contact2,
            contactCount
        );
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }
}
