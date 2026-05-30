#pragma once

#include "BodySoA.h"

namespace PS_AGONY
{
    class AABBSoAViewer
    {
        size_t count = 0;
    public:
        const Real* minX = nullptr;
        const Real* minY = nullptr;
        const Real* maxX = nullptr;
        const Real* maxY = nullptr;

        AABBSoAViewer() = default;

        AABBSoAViewer(const AABBSoA& aabb) :
            count(aabb.minX.size()),
            minX(aabb.minX.data()),
            minY(aabb.minY.data()),
            maxX(aabb.maxX.data()),
            maxY(aabb.maxY.data())
        {
        }

        size_t getCount() const noexcept { return count; }
    };

    class BodySoAViewer
    {
        struct AABBSoAViewer_Internal
        {
            const Real* minX = nullptr;
            const Real* minY = nullptr;
            const Real* maxX = nullptr;
            const Real* maxY = nullptr;

            AABBSoAViewer_Internal() = default;

            AABBSoAViewer_Internal(const AABBSoA& aabb) :
                minX(aabb.minX.data()),
                minY(aabb.minY.data()),
                maxX(aabb.maxX.data()),
                maxY(aabb.maxY.data())
            {}
        };

        size_t count = 0;
    public:
        const Real* positionX = nullptr;
        const Real* positionY = nullptr;

        const Real* velocityX = nullptr;
        const Real* velocityY = nullptr;

        const Real* rotation = nullptr;

        const Real* angularVelocity = nullptr;

        const Real* mass = nullptr;
        const Real* invMass = nullptr;

        const Real* inertia = nullptr;
        const Real* invInertia = nullptr;

        const MaterialIndex* materialIndex = nullptr;

        AABBSoAViewer_Internal aabb;

        const BodyType* bodyType = nullptr;
        const BodyIndex* shapeIndex = nullptr;

        const uint8_t* collisionDebug = nullptr;
    public:
        BodySoAViewer() = default;

        explicit BodySoAViewer(const BodySoA& bodies) noexcept :
            count(bodies.getCount()),
            positionX(bodies.positionX.data()),
            positionY(bodies.positionY.data()),
            velocityX(bodies.velocityX.data()),
            velocityY(bodies.velocityY.data()),
            rotation(bodies.rotation.data()),
            angularVelocity(bodies.angularVelocity.data()),
            mass(bodies.mass.data()),
            invMass(bodies.invMass.data()),
            inertia(bodies.inertia.data()),
            invInertia(bodies.invInertia.data()),
            materialIndex(bodies.materialIndex.data()),
            aabb(bodies.aabb),
            bodyType(bodies.bodyType.data()),
            shapeIndex(bodies.shapeIndex.data()),
            collisionDebug(bodies.collisionDebug.data())
        {}

        size_t getCount() const noexcept { return count; }
    };
}