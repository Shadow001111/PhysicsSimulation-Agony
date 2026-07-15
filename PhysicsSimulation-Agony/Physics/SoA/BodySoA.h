#pragma once
#include "AABBSoA.h"

namespace PS_AGONY
{
	struct BodySoA
	{
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

		CacheLineAlignedVector<uint8_t> isStatic; // TODO: Maybe use 1 bit?
		CacheLineAlignedVector<MaterialIndex> materialIndex;
		AABBSoA aabb;
		CacheLineAlignedVector<BodyType> bodyType;
		CacheLineAlignedVector<ObjectIndex> shapeIndex;

        CacheLineAlignedVector<std::vector<BodyAttachment>> attachments;

		void append(
			Vec2 pos,
			Vec2 vel,
			Real rot,
			Real anglVel,
			Real mass, Real invMass,
			Real inertia, Real invInertia,
			Vec2 localCenterOfMass,
			MaterialIndex materialIndex,
			BodyType bodyType,
			ObjectIndex shapeIndex
		);

		void swapWithBack(size_t index);

		void popBack();

        void addAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);
        void removeAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);

		size_t getCount() const noexcept { return offsetX.size(); }
		size_t getMemoryUsage() const noexcept;
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
            {
            }
        };

        size_t count = 0;
    public:
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
        const MaterialIndex* materialIndex = nullptr;
        AABBSoAViewer_Internal aabb;
        const BodyType* bodyType = nullptr;
        const ObjectIndex* shapeIndex = nullptr;

        const std::vector<BodyAttachment>* attachments = nullptr;
    public:
        BodySoAViewer() = default;

        explicit BodySoAViewer(const BodySoA& data) noexcept :
            count(data.getCount()),
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
            materialIndex(data.materialIndex.data()),
            aabb(data.aabb),
            bodyType(data.bodyType.data()),
            shapeIndex(data.shapeIndex.data()),
            attachments(data.attachments.data())
        {}

        size_t getCount() const noexcept { return count; }
    };
}