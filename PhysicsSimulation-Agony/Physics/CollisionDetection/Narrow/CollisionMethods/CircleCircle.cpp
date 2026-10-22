#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <bit>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    void NarrowPhaseCollisionDetector::collisionCircleCircle(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        const size_t pairCount = pairs.size();
        size_t i = 0;
        if constexpr (true)
        {
            ObjectPair colliderPairsBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionAXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionAYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionBXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionBYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real radiusABatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real radiusBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real normalXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real normalYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real depthBatch[RealSimd::lanes];

            const RealSimd normalBX(DEFAULT_NORMAL_X);
            const RealSimd normalBY(DEFAULT_NORMAL_Y);

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

                    const ObjectIndex shapeA = shapeIndexPtr[indexA];
                    const ObjectIndex shapeB = shapeIndexPtr[indexB];

                    radiusABatch[j] = radiusPtr[shapeA];
                    radiusBBatch[j] = radiusPtr[shapeB];
                }

                const RealSimd positionAX = RealSimd::load(positionAXBatch);
                const RealSimd positionAY = RealSimd::load(positionAYBatch);
                const RealSimd positionBX = RealSimd::load(positionBXBatch);
                const RealSimd positionBY = RealSimd::load(positionBYBatch);

                const RealSimd radiusA = RealSimd::load(radiusABatch);
                const RealSimd radiusB = RealSimd::load(radiusBBatch);

                const RealSimd deltaPositionX = positionBX - positionAX;
                const RealSimd deltaPositionY = positionBY - positionAY;

                const RealSimd radiusSum = radiusA + radiusB;
                const RealSimd radiusSumSquared = radiusSum * radiusSum;

                const RealSimd squaredDistance = RealSimd::mulAdd(deltaPositionX, deltaPositionX, deltaPositionY * deltaPositionY);

                const RealSimd isCollidingMask = squaredDistance < radiusSumSquared;

                unsigned int collisionMask = isCollidingMask.movemask();
                if (collisionMask == 0) continue;

                const RealSimd distance = RealSimd::sqrt(squaredDistance);
                const RealSimd depth = radiusSum - distance;

                const RealSimd isCloseToZeroMask = distance < RealSimd(ZERO_DIVISION_BOUNDARY);

                const RealSimd invDistance = RealSimd(1) / distance;

                const RealSimd normalAX = deltaPositionX * invDistance;
                const RealSimd normalAY = deltaPositionY * invDistance;

                const RealSimd normalX = RealSimd::blendv(normalAX, normalBX, isCloseToZeroMask);
                const RealSimd normalY = RealSimd::blendv(normalAY, normalBY, isCloseToZeroMask);

                normalX.store(normalXBatch);
                normalY.store(normalYBatch);
                depth.store(depthBatch);

                {
                    while (collisionMask != 0)
                    {
                        const int lane = std::countr_zero(collisionMask);
                        collisionMask &= collisionMask - 1;

                        const ObjectPair pair = colliderPairsBatch[lane];
                        const Vec2 normal{ normalXBatch[lane], normalYBatch[lane] };
                        const Real depth = depthBatch[lane];
                        const Vec2 positionA{ positionAXBatch[lane], positionAYBatch[lane] };
                        const Real radiusA = radiusABatch[lane];

                        outCollisionData.emplace_back(
                            pair.a, pair.b,
                            colliderOwnerPtr[pair.a], colliderOwnerPtr[pair.b],
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
        }
        for (; i < pairCount; i++)
        {
            auto [indexA, indexB] = pairs[i];

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real radiusB = radiusPtr[shapeB];

            const Vec2 deltaPosition = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);

            if (squaredDistance >= radiusSum * radiusSum) continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance < ZERO_DIVISION_BOUNDARY) [[unlikely]]
            {
                normal = { DEFAULT_NORMAL_X, DEFAULT_NORMAL_Y };
            }
            else
            {
                const Real invDistance = Real(1.0) / distance;
                normal = deltaPosition * invDistance;
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