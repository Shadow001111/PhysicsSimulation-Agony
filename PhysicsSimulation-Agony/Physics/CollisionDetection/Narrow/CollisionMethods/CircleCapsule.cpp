#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <bit>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    void NarrowPhaseCollisionDetector::collisionCircleCapsule(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT circleRadiusPtr = circles.radius;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        const size_t pairCount = pairs.size();
        size_t i = 0;
        {
            ObjectPair colliderPairsBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionAXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionAYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionBXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionBYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real cosBBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real sinBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real radiusABatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real halfLengthBBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real radiusBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real normalXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real normalYBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real depthBatch[RealSimd::lanes];

            const RealSimd zero(Real(0));
            const RealSimd one(Real(1));

            for (; i + RealSimd::lanes <= pairCount; i += RealSimd::lanes)
            {
                for (size_t j = 0; j < RealSimd::lanes; j++)
                {
                    const auto pair = pairs[i + j];
                    colliderPairsBatch[j] = pair;

                    const ObjectIndex indexA = pair.a;
                    const ObjectIndex indexB = pair.b;

                    positionAXBatch[j] = positionXPtr[indexA];
                    positionAYBatch[j] = positionYPtr[indexA];
                    positionBXBatch[j] = positionXPtr[indexB];
                    positionBYBatch[j] = positionYPtr[indexB];

                    cosBBatch[j] = rotationCosPtr[indexB];
                    sinBBatch[j] = rotationSinPtr[indexB];

                    const ObjectIndex shapeA = shapeIndexPtr[indexA];
                    const ObjectIndex shapeB = shapeIndexPtr[indexB];

                    radiusABatch[j] = circleRadiusPtr[shapeA];
                    halfLengthBBatch[j] = halfLengthPtr[shapeB];
                    radiusBBatch[j] = capsuleRadiusPtr[shapeB];
                }

                const RealSimd positionAX = RealSimd::load(positionAXBatch);
                const RealSimd positionAY = RealSimd::load(positionAYBatch);
                const RealSimd positionBX = RealSimd::load(positionBXBatch);
                const RealSimd positionBY = RealSimd::load(positionBYBatch);

                const RealSimd cosB = RealSimd::load(cosBBatch);
                const RealSimd sinB = RealSimd::load(sinBBatch);

                const RealSimd radiusA = RealSimd::load(radiusABatch);
                const RealSimd halfLengthB = RealSimd::load(halfLengthBBatch);
                const RealSimd radiusB = RealSimd::load(radiusBBatch);

                const RealSimd dX = positionAX - positionBX;
                const RealSimd dY = positionAY - positionBY;

                // Circle center -> closest point of the capsule core: clamp(t, -halfLength, halfLength).
                const RealSimd negHalfLength = zero - halfLengthB;
                const RealSimd alongAxis = RealSimd::mulAdd(dX, cosB, dY * sinB);
                const RealSimd clampedLow = RealSimd::blendv(alongAxis, negHalfLength, alongAxis < negHalfLength);
                const RealSimd t = RealSimd::blendv(clampedLow, halfLengthB, halfLengthB < clampedLow);

                const RealSimd deltaX = cosB * t - dX;
                const RealSimd deltaY = sinB * t - dY;

                const RealSimd radiusSum = radiusA + radiusB;
                const RealSimd squaredDistance = RealSimd::mulAdd(deltaX, deltaX, deltaY * deltaY);

                unsigned int collisionMask = (squaredDistance < radiusSum * radiusSum).movemask();
                if (collisionMask == 0) continue;

                const RealSimd distance = RealSimd::sqrt(squaredDistance);
                const RealSimd invDistance = one / distance;
                const RealSimd isCloseToZeroMask = distance < RealSimd(ZERO_DIVISION_BOUNDARY);

                // Circle center on the core: push out along the capsule side, away from the circle's side.
                const RealSimd sideOffset = RealSimd::mulSub(dY, cosB, dX * sinB);
                const RealSimd flipMask = sideOffset < zero;
                const RealSimd fallbackX = RealSimd::blendv(sinB, -sinB, flipMask);
                const RealSimd fallbackY = RealSimd::blendv(-cosB, cosB, flipMask);

                const RealSimd normalX = RealSimd::blendv(deltaX * invDistance, fallbackX, isCloseToZeroMask);
                const RealSimd normalY = RealSimd::blendv(deltaY * invDistance, fallbackY, isCloseToZeroMask);
                const RealSimd depth = radiusSum - distance;

                normalX.store(normalXBatch);
                normalY.store(normalYBatch);
                depth.store(depthBatch);

                while (collisionMask != 0)
                {
                    const int lane = std::countr_zero(collisionMask);
                    collisionMask &= collisionMask - 1;

                    const ObjectPair pair = colliderPairsBatch[lane];
                    const Vec2 normal{ normalXBatch[lane], normalYBatch[lane] };
                    const Vec2 positionA{ positionAXBatch[lane], positionAYBatch[lane] };

                    outCollisionData.emplace_back(
                        pair.a, pair.b,
                        colliderOwnerPtr[pair.a], colliderOwnerPtr[pair.b],
                        normal,
                        depthBatch[lane],
                        1,
                        positionA + normal * radiusABatch[lane],
                        Vec2(),
                        0,
                        0
                    );
                }
            }
        }
        for (; i < pairCount; i++)
        {
            auto [indexA, indexB] = pairs[i];

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = circleRadiusPtr[shapeA];
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radiusB = capsuleRadiusPtr[shapeB];

            const Vec2 axisB = { cosB, sinB };
            const Vec2 d = positionA - positionB;

            // Circle center -> closest point of the capsule core.
            const Real t = clampValue(glm::dot(d, axisB), -halfLengthB, halfLengthB);
            const Vec2 delta = axisB * t - d;

            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(delta, delta);
            if (squaredDistance >= radiusSum * radiusSum) continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance < ZERO_DIVISION_BOUNDARY) [[unlikely]]
            {
                // Circle center on the core: push out along the capsule side, away from the circle's side.
                normal = { sinB, -cosB };
                flipSignIfNegative(normal, d.x * sinB + d.y * cosB);
            }
            else
            {
                normal = delta * (Real(1) / distance);
            }

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                positionA + normal * radiusA,
                Vec2(),
                0,
                0
            );
        }
    }
}