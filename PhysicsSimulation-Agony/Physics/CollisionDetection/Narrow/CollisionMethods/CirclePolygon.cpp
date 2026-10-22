#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
    void NarrowPhaseCollisionDetector::collisionCirclePolygon(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-polygon collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];

            const PolygonGeometry& localPolygonVertices = polyGeometryPtr[shapeB];
            const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.vertices();
            const Vec2* ECSTASY_RESTRICT localNormals = localPolygonVertices.normals();
            const size_t vertexCount = localPolygonVertices.size();

            const Vec2 deltaGlobal = positionA - positionB;
            const Vec2 circleLocal = {
                deltaGlobal.x * cosB + deltaGlobal.y * sinB,
                deltaGlobal.x * -sinB + deltaGlobal.y * cosB
            };

            uint32_t supportEdge = 0;
            Real maxSeparation = 0;
            if (!findSupportEdge(localVerts, localNormals, vertexCount, circleLocal, radiusA, supportEdge, maxSeparation)) continue;

            Vec2 normalLocal;
            Real separation;
            if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
            {
                // Center is outside the polygon but within radiusA of it.
                const Vec2 a = localVerts[supportEdge];
                const Vec2 b = localVerts[nextIndex(supportEdge, uint32_t(vertexCount))];
                const Vec2 closestPoint = closestPointOnSegment(circleLocal, a, b);
                const Vec2 closestPointDelta = circleLocal - closestPoint;
                const Real distSquared = glm::dot(closestPointDelta, closestPointDelta);

                if (distSquared > radiusA * radiusA) continue;

                if (distSquared < ZERO_DIVISION_BOUNDARY_SQUARED) [[unlikely]]
                {
                    separation = 0;
                    normalLocal = localNormals[supportEdge];
                }
                else
                {
                    separation = std::sqrt(distSquared);
                    normalLocal = closestPointDelta * (Real(1) / separation);
                }
            }
            else
            {
                // Circle center is inside the polygon.
                normalLocal = localNormals[supportEdge];
                separation = maxSeparation;
            }

            Vec2 normal = -Vec2{
                cosB * normalLocal.x - sinB * normalLocal.y,
                sinB * normalLocal.x + cosB * normalLocal.y
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
}