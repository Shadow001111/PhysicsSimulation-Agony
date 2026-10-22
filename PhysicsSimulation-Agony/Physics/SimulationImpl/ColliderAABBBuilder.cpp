#include "ColliderAABBBuilder.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
    void buildCircleAABBs(ColliderSoA& colliders, const CircleSoA& circles)
    {
        const size_t count = circles.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build circle AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = circles.colliderIndices.data();
        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabbMinX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabbMinY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabbMaxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabbMaxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];
            const Real radius = radiusPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];

            aabbMinXPtr[colliderIndex] = x - radius;
            aabbMinYPtr[colliderIndex] = y - radius;
            aabbMaxXPtr[colliderIndex] = x + radius;
            aabbMaxYPtr[colliderIndex] = y + radius;
        }
    }

    void buildBoxAABBs(ColliderSoA& colliders, const BoxSoA& boxes)
    {
        const size_t count = boxes.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build box AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = boxes.colliderIndices.data();
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth.data();
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabbMinX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabbMinY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabbMaxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabbMaxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];
            const Real halfWidth = halfWidthPtr[i];
            const Real halfHeight = halfHeightPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];
            const Real cos = worldRotationCosPtr[colliderIndex];
            const Real sin = worldRotationSinPtr[colliderIndex];

            const Real absCos = std::fabs(cos);
            const Real absSin = std::fabs(sin);

            const Real ex = absCos * halfWidth + absSin * halfHeight;
            const Real ey = absSin * halfWidth + absCos * halfHeight;

            aabbMinXPtr[colliderIndex] = x - ex;
            aabbMinYPtr[colliderIndex] = y - ey;
            aabbMaxXPtr[colliderIndex] = x + ex;
            aabbMaxYPtr[colliderIndex] = y + ey;
        }
    }

    void buildPolygonAABBs(ColliderSoA& colliders, const PolygonSoA& polygons)
    {
        const size_t count = polygons.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build polygon AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = polygons.colliderIndices.data();
        const PolygonGeometry* ECSTASY_RESTRICT geometryPtr = polygons.geometry.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabbMinX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabbMinY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabbMaxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabbMaxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];
            const Real cos = worldRotationCosPtr[colliderIndex];
            const Real sin = worldRotationSinPtr[colliderIndex];

            const PolygonGeometry& geometry = geometryPtr[i];
            const Vec2* verticesPtr = geometry.vertices();
            const size_t vertexCount = geometry.size();

            Real minX = std::numeric_limits<Real>::max();
            Real minY = std::numeric_limits<Real>::max();
            Real maxX = -std::numeric_limits<Real>::max();
            Real maxY = -std::numeric_limits<Real>::max();

            for (size_t v = 0; v < vertexCount; v++)
            {
                const Vec2 vertex = verticesPtr[v];
                const Real wx = cos * vertex.x - sin * vertex.y;
                const Real wy = sin * vertex.x + cos * vertex.y;
                minX = std::fmin(minX, wx);
                maxX = std::fmax(maxX, wx);
                minY = std::fmin(minY, wy);
                maxY = std::fmax(maxY, wy);
            }

            aabbMinXPtr[colliderIndex] = x + minX;
            aabbMinYPtr[colliderIndex] = y + minY;
            aabbMaxXPtr[colliderIndex] = x + maxX;
            aabbMaxYPtr[colliderIndex] = y + maxY;
        }
    }

    void buildCapsuleAABBs(ColliderSoA& colliders, const CapsuleSoA& capsules)
    {
        const size_t count = capsules.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build capsule AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = capsules.colliderIndices.data();
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength.data();
        const Real* ECSTASY_RESTRICT radiusPtr = capsules.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabbMinX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabbMinY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabbMaxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabbMaxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];
            const Real halfLength = halfLengthPtr[i];
            const Real radius = radiusPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];

            const Real ex = std::fabs(worldRotationCosPtr[colliderIndex]) * halfLength + radius;
            const Real ey = std::fabs(worldRotationSinPtr[colliderIndex]) * halfLength + radius;

            aabbMinXPtr[colliderIndex] = x - ex;
            aabbMinYPtr[colliderIndex] = y - ey;
            aabbMaxXPtr[colliderIndex] = x + ex;
            aabbMaxYPtr[colliderIndex] = y + ey;
        }
    }
}