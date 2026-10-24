#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
    void NarrowPhaseCollisionDetector::collisionBoxCapsule(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            // Box A, capsule B. The normal points from the box towards the capsule.
            const Vec2 halfExtents = { halfWidthPtr[shapeA], halfHeightPtr[shapeA] };
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 rightA = { rotationCosPtr[indexA], rotationSinPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };
            const Vec2 axisB = { rotationCosPtr[indexB], rotationSinPtr[indexB] };
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radius = capsuleRadiusPtr[shapeB];

            const Vec2 upA = { -rightA.y, rightA.x };
            const Vec2 centerDelta = positionB - positionA;

            // Capsule pose in box-local space, folded into the +,+ quadrant.
            const Real localX = glm::dot(centerDelta, rightA);
            const Real localY = glm::dot(centerDelta, upA);
            const Real sx = std::copysign(Real(1), localX);
            const Real sy = std::copysign(Real(1), localY);
            const Vec2 center = { std::fabs(localX), std::fabs(localY) };
            const Vec2 axis = { (rightA.x * axisB.x + rightA.y * axisB.y) * sx, (rightA.x * axisB.y - rightA.y * axisB.x) * sy };
            const Real absAxisX = std::fabs(axis.x);
            const Real absAxisY = std::fabs(axis.y);

            // Faces: since center >= 0, only right (x) and top (y) can separate best.
            const Real sepX = center.x - halfLengthB * absAxisX - halfExtents.x;
            const Real sepY = center.y - halfLengthB * absAxisY - halfExtents.y;
            const Real faceSeparation = std::fmax(sepX, sepY);
            if (faceSeparation >= radius) continue;

            // Capsule side axis.
            const Vec2 sideAxis = { -axis.y, axis.x };
            const Real sideOffset = glm::dot(sideAxis, center);
            const Real sideSeparation = std::fabs(sideOffset) - (halfExtents.x * absAxisY + halfExtents.y * absAxisX);
            if (sideSeparation >= radius) continue;

            // Side axis wins (bias towards faces for stable ids): a box corner pokes the capsule side.
            const bool sideWins = sideSeparation > faceSeparation + SAT_EPSILON;
            const bool swap = !sideWins && sepY > sepX;

            // Working frame: reference face at x = ext.x.
            const Vec2 ext = swap ? Vec2(halfExtents.y, halfExtents.x) : halfExtents;
            const Vec2 dir = swap ? Vec2(axis.y, axis.x) : axis;
            const Vec2 mid = swap ? Vec2(center.y, center.x) : center;
            const Vec2 basisX = rightA * sx;
            const Vec2 basisY = upA * sy;
            const Vec2 worldX = swap ? basisY : basisX;
            const Vec2 worldY = swap ? basisX : basisY;

            const Vec2 halfCore = dir * halfLengthB;
            const Vec2 s0 = mid - halfCore;
            const Vec2 s1 = mid + halfCore;
            const Real length = halfLengthB * Real(2);

            // Squared distance from 'w' to the capsule core.
            const auto coreDistSquared = [&](const Vec2& w) -> Real
                {
                    const Vec2 d = s0 + dir * std::fmin(std::fmax(glm::dot(w - s0, dir), Real(0)), length) - w;
                    return glm::dot(d, d);
                };

            Vec2 normal = { 1, 0 };
            Real depth = Real(0);
            uint32_t contactCount = 0;
            Vec2 point0 = {};
            Vec2 point1 = {};
            uint32_t id0 = 0;
            uint32_t id1 = 0;

            bool faceContact = false;
            bool rejected = false;
            bool cornerPosX = true;
            bool cornerPosY = true;
            Vec2 vertexNormal = { 1, 0 };
            Real vertexSeparation = faceSeparation;

            if (sideWins)
            {
                vertexNormal = sideOffset >= 0 ? sideAxis : -sideAxis;
                vertexSeparation = sideSeparation;
                cornerPosX = vertexNormal.x >= 0;
                cornerPosY = vertexNormal.y >= 0;
            }
            else do
            {
                Vec2 c0 = s0;
                Vec2 c1 = s1;
                Real e0 = s0.x - ext.x;
                Real e1 = s1.x - ext.x;

                if (faceSeparation > 0)
                {
                    // Disjoint cores: only the part of the segment above the face sees it.
                    // The fold guarantees y-max >= 0, so the segment is never fully beyond the lower end.
                    const Real minY = std::fmin(s0.y, s1.y);
                    const Real maxY = std::fmax(s0.y, s1.y);

                    if (minY > ext.y) break; // Fully beyond the upper end: corner (+,+).

                    const bool beyondLow = minY < -ext.y;
                    const bool beyondHigh = maxY > ext.y;
                    if (beyondLow || beyondHigh)
                    {
                        // Here s0.y != s1.y. Clip the segment to |y| <= ext.y.
                        const Real invDy = Real(1) / (s1.y - s0.y);
                        const Real tA = (-ext.y - s0.y) * invDy;
                        const Real tB = (ext.y - s0.y) * invDy;
                        const Real tLow = std::fmax(Real(0), std::fmin(tA, tB));
                        const Real tHigh = std::fmin(Real(1), std::fmax(tA, tB));
                        const Vec2 span = s1 - s0;
                        c0 = s0 + span * tLow;
                        c1 = s0 + span * tHigh;
                        e0 = c0.x - ext.x;
                        e1 = c1.x - ext.x;

                        const Real minE = std::fmin(e0, e1);
                        const Real faceDistSquared = minE * minE;

                        if (beyondLow && coreDistSquared(Vec2(ext.x, -ext.y)) + SAT_EPSILON < faceDistSquared)
                        {
                            cornerPosY = false;
                            break;
                        }
                        if (beyondHigh && coreDistSquared(ext) + SAT_EPSILON < faceDistSquared) break;
                    }
                }

                const Vec2 clipped = c1 - c0;
                const bool hasSecond = glm::dot(clipped, clipped) >= ZERO_DIVISION_BOUNDARY_SQUARED;
                const Real contactLimit = radius + SAT_EPSILON;
                const uint32_t faceId = swap ? (sy < Real(0) ? 0u : 2u) : (sx < Real(0) ? 3u : 1u);
                const uint32_t idBase = REFERENCE_BODY_TAG | (faceId << 1);

                if (e0 <= contactLimit)
                {
                    point0 = Vec2(c0.x - radius, c0.y);
                    id0 = idBase;
                    contactCount = 1;
                }
                if (hasSecond && e1 <= contactLimit)
                {
                    const Vec2 p = Vec2(c1.x - radius, c1.y);
                    if (contactCount == 0)
                    {
                        point0 = p;
                        id0 = idBase | 1u;
                    }
                    else
                    {
                        point1 = p;
                        id1 = idBase | 1u;
                    }
                    contactCount++;
                }
                if (contactCount == 0) [[unlikely]]
                {
                    rejected = true;
                    break;
                }

                depth = radius - std::fmin(e0, e1);
                faceContact = true;
            } while (false);

            if (rejected) [[unlikely]] continue;

            if (!faceContact)
            {
                // Single contact between a box corner and the capsule core.
                // 'vertexSeparation' > 0 means the cores are disjoint: the exact corner distance decides.
                const Vec2 w = { cornerPosX ? ext.x : -ext.x, cornerPosY ? ext.y : -ext.y };
                const Vec2 q = s0 + dir * std::fmin(std::fmax(glm::dot(w - s0, dir), Real(0)), length);

                normal = vertexNormal;
                depth = radius - vertexSeparation;
                if (vertexSeparation > Real(0))
                {
                    const Vec2 d = q - w;
                    const Real distSquared = glm::dot(d, d);
                    if (distSquared >= radius * radius) continue;
                    if (distSquared >= ZERO_DIVISION_BOUNDARY_SQUARED)
                    {
                        const Real distance = std::sqrt(distSquared);
                        normal = d * (Real(1) / distance);
                        depth = radius - distance;
                    }
                }

                // Corner id in real box space (undo swap, then fold). Corner order: (-,-), (+,-), (+,+), (-,+).
                const bool foldedPosX = swap ? cornerPosY : cornerPosX;
                const bool foldedPosY = swap ? cornerPosX : cornerPosY;
                const bool realPosX = foldedPosX != (sx < Real(0));
                const bool realPosY = foldedPosY != (sy < Real(0));
                const uint32_t vertexId = (realPosY ? 2u : 0u) | ((realPosX != realPosY) ? 1u : 0u);

                contactCount = 1;
                point0 = q - normal * radius;
                id0 = CAPSULE_VERTEX_CONTACT_TAG | vertexId;
            }

            // Working frame -> world.
            normal = worldX * normal.x + worldY * normal.y;
            point0 = positionA + worldX * point0.x + worldY * point0.y;
            if (contactCount > 1) point1 = positionA + worldX * point1.x + worldY * point1.y;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                contactCount,
                point0,
                point1,
                id0,
                id1
            );
        }
    }
}