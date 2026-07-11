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

	struct CircleSoA
	{
		SimdAlignedVector<BodyIndex> bodyIndices;

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
		SimdAlignedVector<BodyIndex> bodyIndices;

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

		Vec2* begin() noexcept { return dataPtr.get(); }
		Vec2* end() noexcept { return dataPtr.get() + verticesCount; }

		size_t getMemoryUsage() const noexcept { return verticesCount * sizeof(Vec2); }
	};

	struct PolygonSoA
	{
		SimdAlignedVector<BodyIndex> bodyIndices;
		std::vector<VerticesContainer> localVertices;

		void append(
			BodyIndex bodyIndex,
			VerticesContainer&& vertices
		)
		{
			this->bodyIndices.push_back(bodyIndex);
			this->localVertices.emplace_back(std::move(vertices));
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
