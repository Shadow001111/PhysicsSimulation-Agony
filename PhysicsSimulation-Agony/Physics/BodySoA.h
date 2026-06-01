#pragma once
#include "Types.h"

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
		// SimdAlignedVector<Real> localCenterOfMassX;
		// SimdAlignedVector<Real> localCenterOfMassY;
		std::vector<MaterialIndex> materialIndex;
		AABBSoA aabb;
		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;
		std::vector<uint8_t> collisionDebug;

		size_t getCount() const noexcept { return positionX.size(); }
	};

	struct CircleSoA
	{
		SimdAlignedVector<Real> radius;
		std::vector<BodyIndex> bodyIndices;

		size_t getCount() const noexcept { return radius.size(); }
	};
}
