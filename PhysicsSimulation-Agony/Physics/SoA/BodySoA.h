#pragma once
#include "AABBSoA.h"

namespace PS_AGONY
{
	struct BodySoA
	{
		SimdAlignedVector<Real> offsetX;
		SimdAlignedVector<Real> offsetY;
		SimdAlignedVector<Real> localCenterOfMassX;
		SimdAlignedVector<Real> localCenterOfMassY;
		SimdAlignedVector<Real> worldCenterX;
		SimdAlignedVector<Real> worldCenterY;

		SimdAlignedVector<Real> velocityX;
		SimdAlignedVector<Real> velocityY;
		SimdAlignedVector<Real> rotation;
		SimdAlignedVector<Real> angularVelocity;
		SimdAlignedVector<Real> mass;
		SimdAlignedVector<Real> invMass;
		SimdAlignedVector<Real> inertia;
		SimdAlignedVector<Real> invInertia;
		SimdAlignedVector<Real> rotationCos;
		SimdAlignedVector<Real> rotationSin;

		std::vector<uint8_t> isStatic; // TODO: Maybe use 1 bit?
		std::vector<MaterialIndex> materialIndex;
		AABBSoA aabb;
		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;

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
			BodyIndex shapeIndex
		);

		void swapWithBack(size_t index);

		void popBack();

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
        const BodyIndex* shapeIndex = nullptr;
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
            shapeIndex(data.shapeIndex.data())
        {}

        size_t getCount() const noexcept { return count; }
    };
}