#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
    void NarrowPhaseCollisionDetector::collisionCircleBox(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
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

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const Vec2 deltaGlobal = positionA - positionB;
            const Vec2 circleLocalPosition = {
                deltaGlobal.x * cosB + deltaGlobal.y * sinB,
                deltaGlobal.x * -sinB + deltaGlobal.y * cosB
            };

            const Vec2 closestLocal = {
                std::fmin(std::fmax(circleLocalPosition.x, -halfWidthB),  halfWidthB),
                std::fmin(std::fmax(circleLocalPosition.y, -halfHeightB), halfHeightB)
            };

            const Vec2 deltaLocal = closestLocal - circleLocalPosition;
            const Real squaredDistance = glm::dot(deltaLocal, deltaLocal);

            if (squaredDistance >= radiusA * radiusA) continue;

            if (squaredDistance >= ZERO_DIVISION_BOUNDARY_SQUARED) [[likely]]
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
                contactId |= (uint32_t)(circleLocalPosition.x > halfWidthB);       // Bit 0 (1): BOX_FACE_RIGHT
                contactId |= (uint32_t)(circleLocalPosition.x < -halfWidthB) << 1; // Bit 1 (2): BOX_FACE_LEFT
                contactId |= (uint32_t)(circleLocalPosition.y > halfHeightB) << 2; // Bit 2 (4): BOX_FACE_TOP
                contactId |= (uint32_t)(circleLocalPosition.y < -halfHeightB) << 3; // Bit 3 (8): BOX_FACE_BOTTOM

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
            const bool useY = dy <= dx;
            const Real minPen = useY ? dy : dx;

            const Real sign = std::copysign(Real(1), circleLocalPosition[useY]);

            Vec2 normalLocal{};
            const uint32_t elementIndex{ useY };
            normalLocal[elementIndex] = sign;

            const uint32_t contactId = 1 << (uint32_t(sign <= 0) + (elementIndex << 1));

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
}