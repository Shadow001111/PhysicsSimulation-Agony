#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
    void NarrowPhaseCollisionDetector::collisionCapsuleCapsule(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Capsule-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfLengthA = halfLengthPtr[shapeA];
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radiusA = capsuleRadiusPtr[shapeA];
            const Real radiusB = capsuleRadiusPtr[shapeB];

            const Vec2 centerDelta = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;

            // Bounding circle rejection (skips the AABB corner cases from the broad phase).
            const Real reach = halfLengthA + halfLengthB + radiusSum;
            if (glm::dot(centerDelta, centerDelta) >= reach * reach) continue;

            const Vec2 axisA = { rotationCosPtr[indexA], rotationSinPtr[indexA] };
            const Vec2 axisB = { rotationCosPtr[indexB], rotationSinPtr[indexB] };

            // Closest points of the two core segments, parametrized from their centers.
            const Real b = glm::dot(axisA, axisB);
            const Real centerAlongA = glm::dot(centerDelta, axisA);
            const Real c = -centerAlongA;
            const Real f = -glm::dot(axisB, centerDelta);
            const Real sinSquared = Real(1) - b * b;
            const bool parallel = sinSquared < CAPSULE_PARALLEL_SIN_SQUARED;

            // Overlap of the segments along axisA (meaningful only when parallel).
            const Real extentB = std::fabs(b) * halfLengthB;
            const Real overlapLo = std::fmax(-halfLengthA, centerAlongA - extentB);
            const Real overlapHi = std::fmin(halfLengthA, centerAlongA + extentB);
            const bool hasOverlap = parallel && (overlapHi - overlapLo >= ZERO_DIVISION_BOUNDARY);

            Real s, t;
            if (hasOverlap)
            {
                s = HALF * (overlapLo + overlapHi);
                t = clampValue(b * s + f, -halfLengthB, halfLengthB);
            }
            else
            {
                s = parallel ? Real(0) : clampValue((b * f - c) / sinSquared, -halfLengthA, halfLengthA);
                t = b * s + f;
                t = clampValue(t, -halfLengthB, halfLengthB);
                s = clampValue(b * t - c, -halfLengthA, halfLengthA);
            }

            const Vec2 pointA = positionA + axisA * s;
            const Vec2 pointB = positionB + axisB * t;
            const Vec2 delta = pointB - pointA;
            const Real squaredDistance = glm::dot(delta, delta);
            if (squaredDistance >= radiusSum * radiusSum) continue;

            Vec2 normal;
            Real depth;
            if (squaredDistance >= ZERO_DIVISION_BOUNDARY_SQUARED)
            {
                const Real distance = std::sqrt(squaredDistance);
                normal = delta * (Real(1) / distance);
                depth = radiusSum - distance;
            }
            else [[unlikely]]
            {
                // Core segments intersect: separate along the cheaper of the two side axes.
                const Vec2 sideA = { -axisA.y, axisA.x };
                const Vec2 sideB = { -axisB.y, axisB.x };
                const Real offsetA = glm::dot(centerDelta, sideA);
                const Real offsetB = glm::dot(centerDelta, sideB);

                // Fake depths. I took out common addition of 'radiusSum'.
                const Real depthA = std::fabs(glm::dot(axisB, sideA)) * halfLengthB - std::fabs(offsetA);
                const Real depthB = std::fabs(glm::dot(axisA, sideB)) * halfLengthA - std::fabs(offsetB);

                if (depthA <= depthB)
                {
                    normal = sideA;
                    flipSignIfNegative(normal, offsetA);
                    depth = depthA;
                }
                else
                {
                    normal = sideB;
                    flipSignIfNegative(normal, offsetB);
                    depth = depthB;
                }
                depth += radiusSum;
            }

            Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD] = { Vec2(), Vec2() };
            uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD] = { 0, 0 };
            uint32_t contactCount = 0;

            if (hasOverlap)
            {
                // Side by side: one contact at each end of the overlap, if actually within reach.
                const Vec2 sideB = { -axisB.y, axisB.x };
                const Real ends[2] = { overlapLo, overlapHi };
                Real minGap = std::numeric_limits<Real>::max();

                for (uint32_t k = 0; k < MAX_CONTACTS_PER_MANIFOLD; k++)
                {
                    const Vec2 p = positionA + axisA * ends[k];
                    const Real gap = std::fabs(glm::dot(p - positionB, sideB));
                    if (gap >= radiusSum) continue;

                    contactPoints[contactCount] = p + normal * radiusA;
                    contactIds[contactCount] = k;
                    contactCount++;
                    minGap = std::fmin(minGap, gap);
                }
                if (contactCount != 0) depth = radiusSum - minGap;
            }

            if (contactCount == 0)
            {
                contactPoints[0] = pointA + normal * radiusA;
                contactIds[0] = 2;
                contactCount = 1;
            }

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
}