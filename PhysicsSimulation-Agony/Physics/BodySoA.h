#pragma once
#include "GlmTypes.h"

#include "EcstasyCore/Simd.h"
#include "Core/MemoryAllocation/AlignedAllocator.h"

#include <vector>
#include <algorithm>
#include <memory>

namespace PS_AGONY
{
	template<typename Container>
	inline size_t getVectorMemoryUsage(const Container& container)
	{
		return container.capacity() * sizeof(container[0]);
	}

	template<typename T, size_t aligment>
	using AlignedVector = std::vector<T, AlignedAllocator<T, aligment>>;

	template<typename T>
	using AlignedVector64 = std::vector<T, AlignedAllocator<T, 64>>;

	template<typename T>
	using SimdAlignedVector = AlignedVector<T, Ecstasy::Simd<T>::bytes>;

	template<typename T>
	using RealSimdAlignedVector = AlignedVector<T, Ecstasy::Simd<Real>::bytes>;

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
		SimdAlignedVector<Real> localCenterOfMassX;
		SimdAlignedVector<Real> localCenterOfMassY;
		SimdAlignedVector<Real> truePositionX;
		SimdAlignedVector<Real> truePositionY;

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

		std::vector<BodyTextureId> textureId;

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
			BodyIndex shapeIndex,
			BodyTextureId textureId
		)
		{
			this->positionX.push_back(pos.x);
			this->positionY.push_back(pos.y);
			this->localCenterOfMassX.push_back(localCenterOfMass.x);
			this->localCenterOfMassY.push_back(localCenterOfMass.y);
			this->truePositionX.push_back(0);
			this->truePositionY.push_back(0);
			this->velocityX.push_back(vel.x);
			this->velocityY.push_back(vel.y);
			this->rotation.push_back(rot);
			this->angularVelocity.push_back(anglVel);
			this->mass.push_back(mass);
			this->invMass.push_back(invMass);
			this->inertia.push_back(inertia);
			this->invInertia.push_back(invInertia);
			this->rotationCos.push_back(std::cos(rot));
			this->rotationSin.push_back(std::sin(rot));
			this->isStatic.push_back(invMass == 0);
			this->materialIndex.push_back(materialIndex);
			this->aabb.minX.push_back(0);
			this->aabb.minY.push_back(0);
			this->aabb.maxX.push_back(0);
			this->aabb.maxY.push_back(0);
			this->bodyType.push_back(bodyType);
			this->shapeIndex.push_back(shapeIndex);
			this->textureId.push_back(textureId);
		}

		size_t getCount() const noexcept { return positionX.size(); }

		size_t getMemoryUsage() const noexcept
		{
			return
				PS_AGONY::getVectorMemoryUsage(positionX) +
				PS_AGONY::getVectorMemoryUsage(positionY) +
				PS_AGONY::getVectorMemoryUsage(localCenterOfMassX) +
				PS_AGONY::getVectorMemoryUsage(localCenterOfMassY) +
				PS_AGONY::getVectorMemoryUsage(truePositionX) +
				PS_AGONY::getVectorMemoryUsage(truePositionY) +
				PS_AGONY::getVectorMemoryUsage(velocityX) +
				PS_AGONY::getVectorMemoryUsage(velocityY) +
				PS_AGONY::getVectorMemoryUsage(rotation) +
				PS_AGONY::getVectorMemoryUsage(angularVelocity) +
				PS_AGONY::getVectorMemoryUsage(mass) +
				PS_AGONY::getVectorMemoryUsage(invMass) +
				PS_AGONY::getVectorMemoryUsage(inertia) +
				PS_AGONY::getVectorMemoryUsage(invInertia) +
				PS_AGONY::getVectorMemoryUsage(rotationCos) +
				PS_AGONY::getVectorMemoryUsage(rotationSin) +
				PS_AGONY::getVectorMemoryUsage(isStatic) +
				PS_AGONY::getVectorMemoryUsage(materialIndex) +
				aabb.getMemoryUsage() +
				PS_AGONY::getVectorMemoryUsage(bodyType) +
				PS_AGONY::getVectorMemoryUsage(shapeIndex) +
				PS_AGONY::getVectorMemoryUsage(textureId);
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

	class VerticesContainer
	{
		std::unique_ptr<Vec2[]> dataPtr;
		size_t verticesCount = 0;
	public:
		VerticesContainer() = default;

		explicit VerticesContainer(const std::vector<Vec2>& vec) :
			verticesCount(vec.size())
		{
			dataPtr.reset(new Vec2[verticesCount]);
			std::copy(vec.begin(), vec.end(), dataPtr.get());
		}

		explicit VerticesContainer(std::vector<Vec2>&& vec) :
			verticesCount(vec.size())
		{
			dataPtr.reset(new Vec2[verticesCount]);
			std::move(vec.begin(), vec.end(), dataPtr.get());
		}

		explicit VerticesContainer(const Vec2* verticesPtr, size_t verticesCount) :
			verticesCount(verticesCount)
		{
			dataPtr.reset(new Vec2[verticesCount]);
			std::copy(verticesPtr, verticesPtr + verticesCount, dataPtr.get());
		}

		VerticesContainer(const VerticesContainer&) = delete;
		VerticesContainer& operator=(const VerticesContainer&) = delete;

		VerticesContainer(VerticesContainer&&) = default;
		VerticesContainer& operator=(VerticesContainer&&) = default;

		const Vec2* data() const noexcept { return dataPtr.get(); }
		size_t size() const noexcept { return verticesCount; }

		size_t getMemoryUsage() const noexcept { return verticesCount * sizeof(Vec2); }
	};

	struct PolygonSoA
	{
		std::vector<BodyIndex> bodyIndices;
		std::vector<VerticesContainer> localVertices;

		void append(
			BodyIndex bodyIndex,
			const Vec2* localVertices,
			size_t verticesCount
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->localVertices.emplace_back(localVertices, verticesCount);
		}

		size_t getCount() const noexcept { return bodyIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			size_t total =
				PS_AGONY::getVectorMemoryUsage(bodyIndices) +
				PS_AGONY::getVectorMemoryUsage(localVertices);
			for (const auto& v : localVertices)
				total += v.getMemoryUsage();
			return total;
		}
	};
}
