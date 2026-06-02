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

        explicit AABBSoAViewer(const AABBSoA& data) :
            count(data.minX.size()),
            minX(data.minX.data()),
            minY(data.minY.data()),
            maxX(data.maxX.data()),
            maxY(data.maxY.data())
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

            AABBSoAViewer_Internal(const AABBSoA& data) :
                minX(data.minX.data()),
                minY(data.minY.data()),
                maxX(data.maxX.data()),
                maxY(data.maxY.data())
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
		const Real* localCenterOfMassX = nullptr;
		const Real* localCenterOfMassY = nullptr;
		const Real* rotationCos = nullptr;
		const Real* rotationSin = nullptr;

        const MaterialIndex* materialIndex = nullptr;
        AABBSoAViewer_Internal aabb;
        const BodyType* bodyType = nullptr;
        const BodyIndex* shapeIndex = nullptr;
    public:
        BodySoAViewer() = default;

        explicit BodySoAViewer(const BodySoA& data) noexcept :
            count(data.getCount()),
            positionX(data.positionX.data()),
            positionY(data.positionY.data()),
            velocityX(data.velocityX.data()),
            velocityY(data.velocityY.data()),
            rotation(data.rotation.data()),
            angularVelocity(data.angularVelocity.data()),
            mass(data.mass.data()),
            invMass(data.invMass.data()),
            inertia(data.inertia.data()),
            invInertia(data.invInertia.data()),
			localCenterOfMassX(data.localCenterOfMassX.data()),
			localCenterOfMassY(data.localCenterOfMassY.data()),
			rotationCos(data.rotationCos.data()),
			rotationSin(data.rotationSin.data()),
            materialIndex(data.materialIndex.data()),
            aabb(data.aabb),
            bodyType(data.bodyType.data()),
            shapeIndex(data.shapeIndex.data())
        {}

        size_t getCount() const noexcept { return count; }
    };

    class CircleSoAViewer
    {
        size_t count = 0;
    public:
        const BodyIndex* bodyIndices = nullptr;
        const Real* radius = nullptr;
    
        CircleSoAViewer() = default;

        explicit CircleSoAViewer(const CircleSoA& data) :
            count(data.getCount()),
            bodyIndices(data.bodyIndices.data()),
            radius(data.radius.data())
        {}

        size_t getCount() const noexcept { return count; }
    };

    class BoxSoAViewer
    {
        size_t count = 0;
    public:
        const BodyIndex* bodyIndices = nullptr;
        const Real* width = nullptr;
        const Real* height = nullptr;
    
        BoxSoAViewer() = default;

        explicit BoxSoAViewer(const BoxSoA& data) :
            count(data.getCount()),
            bodyIndices(data.bodyIndices.data()),
            width(data.width.data()),
            height(data.height.data())
        {
        }

        size_t getCount() const noexcept { return count; }
    };
}