#pragma once
#include "../GlmTypes.h"
#include "../ContainerUtilities.h"
#include "AABBSoA.h"
#include "../Material.h"

namespace PS_AGONY
{
	// A Collider is a shape rigidly offset/rotated from its owning Body.
	// Bodies hold mass/inertia/velocity; Colliders hold geometry + material
	// and derive their world transform from the body's transform each step.
	struct ColliderSoA
	{
		CacheLineAlignedVector<ObjectIndex> bodyIndex;

		CacheLineAlignedVector<Real> localOffsetX;
		CacheLineAlignedVector<Real> localOffsetY;
		CacheLineAlignedVector<Real> localRotationCos;
		CacheLineAlignedVector<Real> localRotationSin;

		CacheLineAlignedVector<Real> worldPosX;
		CacheLineAlignedVector<Real> worldPosY;
		CacheLineAlignedVector<Real> worldRotationCos;
		CacheLineAlignedVector<Real> worldRotationSin;

		CacheLineAlignedVector<MaterialIndex> materialIndex;
		CacheLineAlignedVector<BodyType> shapeType;   // Circle/Box/Polygon.
		CacheLineAlignedVector<ObjectIndex> shapeIndex; // Index into the shape's own SoA.

		AABBSoA aabb;

		ColliderIndex append(
			ObjectIndex bodyIndexIn,
			Vec2 localOffset,
			Real localRotation,
			MaterialIndex materialIndexIn,
			BodyType shapeTypeIn,
			ObjectIndex shapeIndexIn
		)
		{
			const ColliderIndex newIndex = static_cast<ColliderIndex>(bodyIndex.size());

			bodyIndex.push_back(bodyIndexIn);

			localOffsetX.push_back(localOffset.x);
			localOffsetY.push_back(localOffset.y);
			localRotationCos.push_back(std::cos(localRotation));
			localRotationSin.push_back(std::sin(localRotation));

			worldPosX.push_back(0);
			worldPosY.push_back(0);
			worldRotationCos.push_back(1);
			worldRotationSin.push_back(0);

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

		// Swap-removes 'index'; returns the pre-removal last index (== index if none moved).
		size_t swapRemove(size_t index)
		{
			const size_t lastIndex = bodyIndex.size() - 1;

			if (index != lastIndex)
			{
				std::swap(bodyIndex[index], bodyIndex[lastIndex]);
				std::swap(localOffsetX[index], localOffsetX[lastIndex]);
				std::swap(localOffsetY[index], localOffsetY[lastIndex]);
				std::swap(localRotationCos[index], localRotationCos[lastIndex]);
				std::swap(localRotationSin[index], localRotationSin[lastIndex]);
				std::swap(worldPosX[index], worldPosX[lastIndex]);
				std::swap(worldPosY[index], worldPosY[lastIndex]);
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
			localRotationCos.pop_back(); localRotationSin.pop_back();
			worldPosX.pop_back(); worldPosY.pop_back();
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
				PS_AGONY::getVectorMemoryUsage(localRotationCos) + PS_AGONY::getVectorMemoryUsage(localRotationSin) +
				PS_AGONY::getVectorMemoryUsage(worldPosX) + PS_AGONY::getVectorMemoryUsage(worldPosY) +
				PS_AGONY::getVectorMemoryUsage(worldRotationCos) + PS_AGONY::getVectorMemoryUsage(worldRotationSin) +
				PS_AGONY::getVectorMemoryUsage(materialIndex) +
				PS_AGONY::getVectorMemoryUsage(shapeType) +
				PS_AGONY::getVectorMemoryUsage(shapeIndex) +
				aabb.getMemoryUsage();
		}
	};

	// Mutable viewer: broad/narrow phase read geometry; per-step transform/AABB build writes it.
	class ColliderSoAViewer
	{
		size_t count = 0;
	public:
		const ObjectIndex* bodyIndex = nullptr;

		const Real* localOffsetX = nullptr;
		const Real* localOffsetY = nullptr;
		const Real* localRotationCos = nullptr;
		const Real* localRotationSin = nullptr;

		Real* worldPosX = nullptr;
		Real* worldPosY = nullptr;
		Real* worldRotationCos = nullptr;
		Real* worldRotationSin = nullptr;

		const MaterialIndex* materialIndex = nullptr;
		const BodyType* shapeType = nullptr;
		const ObjectIndex* shapeIndex = nullptr;

		Real* aabbMinX = nullptr;
		Real* aabbMinY = nullptr;
		Real* aabbMaxX = nullptr;
		Real* aabbMaxY = nullptr;

		ColliderSoAViewer() = default;

		explicit ColliderSoAViewer(ColliderSoA& data) noexcept :
			count(data.getCount()),
			bodyIndex(data.bodyIndex.data()),
			localOffsetX(data.localOffsetX.data()),
			localOffsetY(data.localOffsetY.data()),
			localRotationCos(data.localRotationCos.data()),
			localRotationSin(data.localRotationSin.data()),
			worldPosX(data.worldPosX.data()),
			worldPosY(data.worldPosY.data()),
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