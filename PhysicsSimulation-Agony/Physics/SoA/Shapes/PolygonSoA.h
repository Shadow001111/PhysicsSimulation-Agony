#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

#include <memory>

namespace PS_AGONY
{
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
		SimdAlignedVector<ColliderIndex> colliderIndices;
		std::vector<VerticesContainer> localVertices;

		void append(
			ColliderIndex colliderIndex,
			VerticesContainer&& vertices
		)
		{
			this->colliderIndices.push_back(colliderIndex);
			this->localVertices.emplace_back(std::move(vertices));
		}

		size_t getCount() const noexcept { return colliderIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			size_t total =
				PS_AGONY::getVectorMemoryUsage(colliderIndices) +
				PS_AGONY::getVectorMemoryUsage(localVertices);
			for (const auto& v : localVertices)
				total += v.getMemoryUsage();
			return total;
		}
	};

	class PolygonSoAViewer
	{
		size_t count = 0;
	public:
		const ColliderIndex* colliderIndices = nullptr;
		const VerticesContainer* localVertices = nullptr;

		PolygonSoAViewer() = default;

		explicit PolygonSoAViewer(const PolygonSoA& data) :
			count(data.getCount()),
			colliderIndices(data.colliderIndices.data()),
			localVertices(data.localVertices.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}