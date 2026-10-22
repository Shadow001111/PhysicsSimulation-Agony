#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
    void NarrowPhaseCollisionDetector::collisionPolygonCapsule(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            // Polygon A, capsule B. The normal points from the polygon towards the capsule.
            const PolygonGeometry& geometry = polyGeometryPtr[shapeA];
            const Vec2* ECSTASY_RESTRICT vertsPtr = geometry.vertices();
            const Vec2* ECSTASY_RESTRICT normalsPtr = geometry.normals();
            const uint32_t vertexCount = uint32_t(geometry.size());

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 rightA = { rotationCosPtr[indexA], rotationSinPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };
            const Vec2 axisB = { rotationCosPtr[indexB], rotationSinPtr[indexB] };
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radius = capsuleRadiusPtr[shapeB];

            const Vec2 upA = { -rightA.y, rightA.x };
            const Vec2 centerDelta = positionB - positionA;

            // Capsule pose in polygon-local space.
            const Vec2 centerLocal = { glm::dot(centerDelta, rightA), glm::dot(centerDelta, upA) };
            const Vec2 axis = { rightA.x * axisB.x + rightA.y * axisB.y, rightA.x * axisB.y - rightA.y * axisB.x };
            const Vec2 halfCore = axis * halfLengthB;
            const Vec2 s0 = centerLocal - halfCore;
            const Vec2 s1 = centerLocal + halfCore;
            const Real length = halfLengthB * Real(2);

            const Vec2 sideAxis = { -axis.y, axis.x };
            const Real sideOffset = glm::dot(sideAxis, centerLocal);

            // Best face and polygon extent along the capsule side axis, in one pass.
            Real faceSeparation = -std::numeric_limits<Real>::max();
            uint32_t faceIndex = 0;
            Real minSide = std::numeric_limits<Real>::max();
            Real maxSide = -std::numeric_limits<Real>::max();
            uint32_t minSideVertex = 0;
            uint32_t maxSideVertex = 0;
            bool rejected = false;

            for (uint32_t i = 0; i < vertexCount; i++)
            {
                const Vec2 v = vertsPtr[i];
                const Vec2 n = normalsPtr[i];

                const Real separation = std::fmin(glm::dot(n, s0 - v), glm::dot(n, s1 - v));
                if (separation >= radius)
                {
                    rejected = true;
                    break;
                }
                if (separation > faceSeparation)
                {
                    faceSeparation = separation;
                    faceIndex = i;
                }

                const Real side = glm::dot(v, sideAxis);
                if (side < minSide) { minSide = side; minSideVertex = i; }
                if (side > maxSide) { maxSide = side; maxSideVertex = i; }
            }
            if (rejected) continue;

            const Real sidePlus = sideOffset - maxSide;   // Capsule on the +side of the polygon.
            const Real sideMinus = minSide - sideOffset;  // Capsule on the -side of the polygon.
            const Real sideSeparation = std::fmax(sidePlus, sideMinus);
            if (sideSeparation >= radius) continue;

            // Squared distance from 'w' to the capsule core.
            const auto coreDistSquared = [&](const Vec2& w) -> Real
                {
                    const Vec2 d = s0 + axis * std::fmin(std::fmax(glm::dot(w - s0, axis), Real(0)), length) - w;
                    return glm::dot(d, d);
                };

            Vec2 normal = {};
            Real depth = Real(0);
            uint32_t contactCount = 0;
            Vec2 point0 = {};
            Vec2 point1 = {};
            uint32_t id0 = 0;
            uint32_t id1 = 0;

            bool faceContact = false;
            uint32_t vertexId = 0;
            Vec2 vertexNormal = {};
            Real vertexSeparation = Real(0);

            do
            {
                // Side axis wins (bias towards faces for stable ids): a vertex pokes the capsule side.
                if (sideSeparation > faceSeparation + SAT_EPSILON)
                {
                    const bool onPlusSide = sidePlus >= sideMinus;
                    vertexId = onPlusSide ? maxSideVertex : minSideVertex;
                    vertexNormal = onPlusSide ? sideAxis : -sideAxis;
                    vertexSeparation = sideSeparation;
                    break;
                }

                // Face contact: the polygon face is the reference, the capsule core the incident segment.
                const uint32_t i0 = faceIndex;
                const uint32_t i1 = nextIndex(i0, vertexCount);
                const Vec2 a = vertsPtr[i0];
                const Vec2 b = vertsPtr[i1];
                const Vec2 n = normalsPtr[i0];
                const Vec2 tangent = { -n.y, n.x };

                vertexNormal = n;
                vertexSeparation = faceSeparation;

                Vec2 c0 = s0;
                Vec2 c1 = s1;
                Real e0 = glm::dot(s0 - a, n);
                Real e1 = glm::dot(s1 - a, n);

                if (faceSeparation > Real(0))
                {
                    // Disjoint cores: only the part of the segment above the edge sees the face;
                    // an overhanging part may be closer to an end vertex of the edge.
                    const Real edgeLength = glm::dot(b - a, tangent);
                    const Real t0 = glm::dot(s0 - a, tangent);
                    const Real t1 = glm::dot(s1 - a, tangent);

                    const bool beyondStart = std::fmin(t0, t1) < Real(0);
                    const bool beyondEnd = std::fmax(t0, t1) > edgeLength;
                    if (beyondStart || beyondEnd)
                    {
                        if (clipSegment(c0, c1, a, tangent)) { vertexId = i0; break; }
                        if (clipSegment(c0, c1, b, -tangent)) { vertexId = i1; break; }

                        e0 = glm::dot(c0 - a, n);
                        e1 = glm::dot(c1 - a, n);

                        const Real minE = std::fmin(e0, e1);
                        const Real faceDistSquared = minE * minE;

                        if (beyondStart && coreDistSquared(a) + SAT_EPSILON < faceDistSquared) { vertexId = i0; break; }
                        if (beyondEnd && coreDistSquared(b) + SAT_EPSILON < faceDistSquared) { vertexId = i1; break; }
                    }
                }

                const Vec2 span = c1 - c0;
                const bool hasSecond = glm::dot(span, span) >= ZERO_DIVISION_BOUNDARY_SQUARED;
                const Real contactLimit = radius + SAT_EPSILON;
                const uint32_t idBase = REFERENCE_BODY_TAG | (i0 << 1);

                if (e0 <= contactLimit)
                {
                    point0 = c0 - n * radius;
                    id0 = idBase;
                    contactCount = 1;
                }
                if (hasSecond && e1 <= contactLimit)
                {
                    const Vec2 p = c1 - n * radius;
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

                normal = n;
                depth = radius - std::fmin(e0, e1);
                faceContact = true;
            } while (false);

            if (rejected) [[unlikely]] continue;

            if (!faceContact)
            {
                // Single contact between a polygon vertex and the capsule core.
                // 'vertexSeparation' > 0 means the cores are disjoint: the exact vertex distance decides.
                const Vec2 w = vertsPtr[vertexId];
                const Vec2 q = s0 + axis * std::fmin(std::fmax(glm::dot(w - s0, axis), Real(0)), length);

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

                contactCount = 1;
                point0 = q - normal * radius;
                id0 = CAPSULE_VERTEX_CONTACT_TAG | vertexId;
            }

            // Polygon-local -> world.
            normal = rightA * normal.x + upA * normal.y;
            point0 = positionA + rightA * point0.x + upA * point0.y;
            if (contactCount > 1) point1 = positionA + rightA * point1.x + upA * point1.y;
            // TODO: Maybe remove branch? Showed +1% of execution time though.

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