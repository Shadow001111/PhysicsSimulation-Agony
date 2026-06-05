#pragma once
#include "GlmTypes.h"

#include "Core/MemoryAllocation/AlignedAllocator.h"
#include "Core/Simd.h"

#include <vector>

namespace PS_AGONY
{
	template<typename Container>
	inline size_t getVectorMemoryUsage(const Container& container)
	{
		return container.capacity() * sizeof(container[0]);
	}

	template<typename T>
	using SimdAlignedVector = std::vector<T, AlignedAllocator<T, Simd<T>::bytes>>;

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

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(minX) +
				PS_AGONY::getVectorMemoryUsage(minY) +
				PS_AGONY::getVectorMemoryUsage(maxX) +
				PS_AGONY::getVectorMemoryUsage(maxY);
		}
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
			this->aabb.minX.push_back(0);
			this->aabb.minY.push_back(0);
			this->aabb.maxX.push_back(0);
			this->aabb.maxY.push_back(0);
			this->bodyType.push_back(bodyType);
			this->shapeIndex.push_back(shapeIndex);
		}

		size_t getCount() const noexcept { return positionX.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(positionX) +
				PS_AGONY::getVectorMemoryUsage(positionY) +
				PS_AGONY::getVectorMemoryUsage(velocityX) +
				PS_AGONY::getVectorMemoryUsage(velocityY) +
				PS_AGONY::getVectorMemoryUsage(rotation) +
				PS_AGONY::getVectorMemoryUsage(angularVelocity) +
				PS_AGONY::getVectorMemoryUsage(mass) +
				PS_AGONY::getVectorMemoryUsage(invMass) +
				PS_AGONY::getVectorMemoryUsage(inertia) +
				PS_AGONY::getVectorMemoryUsage(invInertia) +
				PS_AGONY::getVectorMemoryUsage(localCenterOfMassX) +
				PS_AGONY::getVectorMemoryUsage(localCenterOfMassY) +
				PS_AGONY::getVectorMemoryUsage(rotationCos) +
				PS_AGONY::getVectorMemoryUsage(rotationSin) +
				PS_AGONY::getVectorMemoryUsage(materialIndex) +
				aabb.getMemoryUsage() +
				PS_AGONY::getVectorMemoryUsage(bodyType) +
				PS_AGONY::getVectorMemoryUsage(shapeIndex);
		}
	};

	struct CircleSoA
	{
		std::vector<BodyIndex> bodyIndices;

		SimdAlignedVector<Real> radius;

		void append(
			BodyIndex bodyIndex,
			Real radius
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->radius.push_back(radius);
		}

		size_t getCount() const noexcept { return bodyIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndices) +
				PS_AGONY::getVectorMemoryUsage(radius);
		}
	};

	struct BoxSoA
	{
		std::vector<BodyIndex> bodyIndices;

		std::vector<Real> halfWidth;
		std::vector<Real> halfHeight;

		void append(
			BodyIndex bodyIndex,
			Real halfWidth,
			Real halfHeight
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->halfWidth.push_back(halfWidth);
			this->halfHeight.push_back(halfHeight);
		}

		size_t getCount() const noexcept { return bodyIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndices) +
				PS_AGONY::getVectorMemoryUsage(halfWidth) +
				PS_AGONY::getVectorMemoryUsage(halfHeight);
		}
	};
}
