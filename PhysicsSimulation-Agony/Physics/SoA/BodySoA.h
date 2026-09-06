#pragma once
#include "AABBSoA.h"

namespace PS_AGONY
{
    struct BodySoA
    {
        CacheLineAlignedVector<Real> renderOldOffsetX;
        CacheLineAlignedVector<Real> renderOldOffsetY;
        CacheLineAlignedVector<Real> renderOldRotation;
        CacheLineAlignedVector<Real> renderRotationWrapCount;

        CacheLineAlignedVector<Real> offsetX;
        CacheLineAlignedVector<Real> offsetY;
        CacheLineAlignedVector<Real> localCenterOfMassX;
        CacheLineAlignedVector<Real> localCenterOfMassY;
        CacheLineAlignedVector<Real> worldCenterX;
        CacheLineAlignedVector<Real> worldCenterY;

        CacheLineAlignedVector<Real> velocityX;
        CacheLineAlignedVector<Real> velocityY;
        CacheLineAlignedVector<Real> rotation;
        CacheLineAlignedVector<Real> angularVelocity;
        CacheLineAlignedVector<Real> mass;
        CacheLineAlignedVector<Real> invMass;
        CacheLineAlignedVector<Real> inertia;
        CacheLineAlignedVector<Real> invInertia;
        CacheLineAlignedVector<Real> rotationCos;
        CacheLineAlignedVector<Real> rotationSin;

        CacheLineAlignedVector<uint8_t> isStatic;

        CacheLineAlignedVector<std::vector<BodyAttachment>> attachments;      // Springs, etc.
        CacheLineAlignedVector<std::vector<ColliderIndex>> colliderIndices;   // Owned colliders.

        void append(
            Vec2 pos, Vec2 vel, Real rot, Real anglVel,
            Real mass, Real invMass, Real inertia, Real invInertia,
            Vec2 localCenterOfMass
        );

        void swapWithBack(size_t index);
        void popBack();

        void addAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);
        void removeAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);

        void addCollider(size_t bodyIndex, ColliderIndex colliderIndex);
        void removeCollider(size_t bodyIndex, ColliderIndex colliderIndex);

        size_t getCount() const noexcept { return offsetX.size(); }
        size_t getMemoryUsage() const noexcept;
    };

    class BodySoAViewer
    {
        size_t count = 0;
    public:
        const Real* renderOldOffsetX = nullptr;
        const Real* renderOldOffsetY = nullptr;
        const Real* renderOldRotation = nullptr;
        const Real* renderRotationWrapCount = nullptr;

        const Real* offsetX = nullptr;
        const Real* offsetY = nullptr;
        const Real* localCenterOfMassX = nullptr;
        const Real* localCenterOfMassY = nullptr;
        const Real* worldCenterX = nullptr;
        const Real* worldCenterY = nullptr;
        const Real* velocityX = nullptr;
        const Real* velocityY = nullptr;
        const Real* rotation = nullptr;
        const Real* angularVelocity = nullptr;
        const Real* mass = nullptr;
        const Real* invMass = nullptr;
        const Real* inertia = nullptr;
        const Real* invInertia = nullptr;
        const Real* rotationCos = nullptr;
        const Real* rotationSin = nullptr;

        const uint8_t* isStatic = nullptr;

        const std::vector<BodyAttachment>* attachments = nullptr;
    public:
        BodySoAViewer() = default;

        explicit BodySoAViewer(const BodySoA& data) noexcept :
            count(data.getCount()),
            renderOldOffsetX(data.renderOldOffsetX.data()),
            renderOldOffsetY(data.renderOldOffsetY.data()),
            renderOldRotation(data.renderOldRotation.data()),
            renderRotationWrapCount(data.renderRotationWrapCount.data()),

            offsetX(data.offsetX.data()),
            offsetY(data.offsetY.data()),
            localCenterOfMassX(data.localCenterOfMassX.data()),
            localCenterOfMassY(data.localCenterOfMassY.data()),
            worldCenterX(data.worldCenterX.data()),
            worldCenterY(data.worldCenterY.data()),
            velocityX(data.velocityX.data()),
            velocityY(data.velocityY.data()),
            rotation(data.rotation.data()),
            angularVelocity(data.angularVelocity.data()),
            mass(data.mass.data()),
            invMass(data.invMass.data()),
            inertia(data.inertia.data()),
            invInertia(data.invInertia.data()),
            rotationCos(data.rotationCos.data()),
            rotationSin(data.rotationSin.data()),
            isStatic(data.isStatic.data()),
            attachments(data.attachments.data())
        {}

        size_t getCount() const noexcept { return count; }
    };
}