#pragma once
#include "GlmTypes.h"

#include "Core/MemoryAllocation/AlignedAllocator.h"
#include "Core/Simd.h"

#include <vector>

namespace PS_AGONY
{
	template<typename T>
	using SimdAlignedVector = std::vector<T, AlignedAllocator<T, Simd<T>::bytes>>;

	enum class BodyType : uint8_t
	{
		Circle,
		Box,
		Polygon,
		COUNT
	};

	struct AABB
	{
		Real minX, minY;
		Real maxX, maxY;
	};

	struct AABBSoA
	{
		SimdAlignedVector<Real> minX;
		SimdAlignedVector<Real> minY;
		SimdAlignedVector<Real> maxX;
		SimdAlignedVector<Real> maxY;

		size_t getCount() const noexcept { return minX.size(); }
	};

	struct BodySoA
	{
		SimdAlignedVector<Real> positionX;
		SimdAlignedVector<Real> positionY;
		SimdAlignedVector<Real> velocityX;
		SimdAlignedVector<Real> velocityY;
		SimdAlignedVector<Real> rotation;
		SimdAlignedVector<Real> angularVelocity;
		SimdAlignedVector<Real> mass;
		SimdAlignedVector<Real> invMass;
		SimdAlignedVector<Real> inertia;
		SimdAlignedVector<Real> invInertia;
		SimdAlignedVector<Real> localCenterOfMassX;
		SimdAlignedVector<Real> localCenterOfMassY;
		SimdAlignedVector<Real> rotationCos;
		SimdAlignedVector<Real> rotationSin;

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
			const AABB& aabb,
			BodyType bodyType,
			BodyIndex shapeIndex
		)
		{
			this->positionX.push_back(pos.x);
			this->positionY.push_back(pos.y);
			this->velocityX.push_back(vel.x);
			this->velocityY.push_back(vel.y);
			this->rotation.push_back(rot);
			this->angularVelocity.push_back(anglVel);
			this->mass.push_back(mass);
			this->invMass.push_back(invMass);
			this->inertia.push_back(inertia);
			this->invInertia.push_back(invInertia);
			this->localCenterOfMassX.push_back(localCenterOfMass.x);
			this->localCenterOfMassY.push_back(localCenterOfMass.y);
			this->rotationCos.push_back(std::cos(rot));
			this->rotationSin.push_back(std::sin(rot));
			this->materialIndex.push_back(materialIndex);
			this->aabb.minX.push_back(aabb.minX);
			this->aabb.minY.push_back(aabb.minY);
			this->aabb.maxX.push_back(aabb.maxX);
			this->aabb.maxY.push_back(aabb.maxY);
			this->bodyType.push_back(bodyType);
			this->shapeIndex.push_back(shapeIndex);
		}

		size_t getCount() const noexcept { return positionX.size(); }
	};

	struct CircleSoA
	{
		SimdAlignedVector<Real> radius;
		std::vector<BodyIndex> bodyIndices;

		void append(
			Real radius,
			BodyIndex bodyIndex
		)
		{
			this->radius.push_back(radius);
			this->bodyIndices.push_back(bodyIndex);
		}

		size_t getCount() const noexcept { return radius.size(); }
	};
}
