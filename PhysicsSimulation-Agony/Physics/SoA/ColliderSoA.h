#pragma once
#include "../GlmTypes.h"
#include "../ContainerUtilities.h"
#include "AABBSoA.h"
#include "../Material.h"
#include "../Constants.h"

#include <cmath>

namespace PS_AGONY
{
	struct ColliderSoA
	{
		CacheLineAlignedVector<ObjectIndex> bodyIndex;

		CacheLineAlignedVector<Real> localOffsetX;
		CacheLineAlignedVector<Real> localOffsetY;
		CacheLineAlignedVector<Real> localRotation; // Rigid local angle; wrapped to [0, 2pi) once, at creation.

		CacheLineAlignedVector<Real> worldPosX;
		CacheLineAlignedVector<Real> worldPosY;
		CacheLineAlignedVector<Real> worldRotation;    // body.rotation + localRotation, re-wrapped every step.
		CacheLineAlignedVector<Real> worldRotationCos;
		CacheLineAlignedVector<Real> worldRotationSin;

		CacheLineAlignedVector<MaterialIndex> materialIndex;
		CacheLineAlignedVector<BodyType> shapeType;
		CacheLineAlignedVector<ObjectIndex> shapeIndex;

		AABBSoA aabb;

		ColliderIndex append(
			ObjectIndex bodyIndexIn,
			Vec2 localOffset,
			Real localRotationIn,
			MaterialIndex materialIndexIn,
			BodyType shapeTypeIn,
			ObjectIndex shapeIndexIn
		)
		{
			const ColliderIndex newIndex = static_cast<ColliderIndex>(bodyIndex.size());

			bodyIndex.push_back(bodyIndexIn);

			localOffsetX.push_back(localOffset.x);
			localOffsetY.push_back(localOffset.y);

			// Wrap once here; local rotation is rigid and never changes afterward.
			Real wrappedLocal = std::fmod(localRotationIn, Constants::TWO_PI);
			if (wrappedLocal < Real(0)) wrappedLocal += Constants::TWO_PI;
			localRotation.push_back(wrappedLocal);

			worldPosX.push_back(0);
			worldPosY.push_back(0);
			worldRotation.push_back(wrappedLocal);
			worldRotationCos.push_back(std::cos(wrappedLocal));
			worldRotationSin.push_back(std::sin(wrappedLocal));

			materialIndex.push_back(materialIndexIn);
			shapeType.push_back(shapeTypeIn);
			shapeIndex.push_back(shapeIndexIn);

			aabb.minX.push_back(0);
			aabb.minY.push_back(0);
			aabb.maxX.push_back(0);
			aabb.maxY.push_back(0);

			return newIndex;
		}

		size_t getCount() const noexcept { return bodyIndex.size(); }

		size_t swapRemove(size_t index)
		{
			const size_t lastIndex = bodyIndex.size() - 1;

			if (index != lastIndex)
			{
				std::swap(bodyIndex[index], bodyIndex[lastIndex]);
				std::swap(localOffsetX[index], localOffsetX[lastIndex]);
				std::swap(localOffsetY[index], localOffsetY[lastIndex]);
				std::swap(localRotation[index], localRotation[lastIndex]);
				std::swap(worldPosX[index], worldPosX[lastIndex]);
				std::swap(worldPosY[index], worldPosY[lastIndex]);
				std::swap(worldRotation[index], worldRotation[lastIndex]);
				std::swap(worldRotationCos[index], worldRotationCos[lastIndex]);
				std::swap(worldRotationSin[index], worldRotationSin[lastIndex]);
				std::swap(materialIndex[index], materialIndex[lastIndex]);
				std::swap(shapeType[index], shapeType[lastIndex]);
				std::swap(shapeIndex[index], shapeIndex[lastIndex]);
				std::swap(aabb.minX[index], aabb.minX[lastIndex]);
				std::swap(aabb.minY[index], aabb.minY[lastIndex]);
				std::swap(aabb.maxX[index], aabb.maxX[lastIndex]);
				std::swap(aabb.maxY[index], aabb.maxY[lastIndex]);
			}

			bodyIndex.pop_back();
			localOffsetX.pop_back(); localOffsetY.pop_back();
			localRotation.pop_back();
			worldPosX.pop_back(); worldPosY.pop_back();
			worldRotation.pop_back();
			worldRotationCos.pop_back(); worldRotationSin.pop_back();
			materialIndex.pop_back();
			shapeType.pop_back();
			shapeIndex.pop_back();
			aabb.minX.pop_back(); aabb.minY.pop_back();
			aabb.maxX.pop_back(); aabb.maxY.pop_back();

			return lastIndex;
		}

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(bodyIndex) +
				PS_AGONY::getVectorMemoryUsage(localOffsetX) + PS_AGONY::getVectorMemoryUsage(localOffsetY) +
				PS_AGONY::getVectorMemoryUsage(localRotation) +
				PS_AGONY::getVectorMemoryUsage(worldPosX) + PS_AGONY::getVectorMemoryUsage(worldPosY) +
				PS_AGONY::getVectorMemoryUsage(worldRotation) +
				PS_AGONY::getVectorMemoryUsage(worldRotationCos) + PS_AGONY::getVectorMemoryUsage(worldRotationSin) +
				PS_AGONY::getVectorMemoryUsage(materialIndex) +
				PS_AGONY::getVectorMemoryUsage(shapeType) +
				PS_AGONY::getVectorMemoryUsage(shapeIndex) +
				aabb.getMemoryUsage();
		}
	};

	class ColliderSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndex = nullptr;

		const Real* localOffsetX = nullptr;
		const Real* localOffsetY = nullptr;
		const Real* localRotation = nullptr;

		const Real* worldPosX = nullptr;
		const Real* worldPosY = nullptr;
		const Real* worldRotation = nullptr;
		const Real* worldRotationCos = nullptr;
		const Real* worldRotationSin = nullptr;

		const MaterialIndex* materialIndex = nullptr;
		const BodyType* shapeType = nullptr;
		const ObjectIndex* shapeIndex = nullptr;

		const Real* aabbMinX = nullptr;
		const Real* aabbMinY = nullptr;
		const Real* aabbMaxX = nullptr;
		const Real* aabbMaxY = nullptr;

		ColliderSoAViewer() = default;

		explicit ColliderSoAViewer(const ColliderSoA& data) noexcept :
			count(data.getCount()),
			bodyIndex(data.bodyIndex.data()),
			localOffsetX(data.localOffsetX.data()),
			localOffsetY(data.localOffsetY.data()),
			localRotation(data.localRotation.data()),
			worldPosX(data.worldPosX.data()),
			worldPosY(data.worldPosY.data()),
			worldRotation(data.worldRotation.data()),
			worldRotationCos(data.worldRotationCos.data()),
			worldRotationSin(data.worldRotationSin.data()),
			materialIndex(data.materialIndex.data()),
			shapeType(data.shapeType.data()),
			shapeIndex(data.shapeIndex.data()),
			aabbMinX(data.aabb.minX.data()),
			aabbMinY(data.aabb.minY.data()),
			aabbMaxX(data.aabb.maxX.data()),
			aabbMaxY(data.aabb.maxY.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}