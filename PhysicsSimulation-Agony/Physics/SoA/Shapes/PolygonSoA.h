#pragma once
#include "../../GlmTypes.h"
#include "../../ContainerUtilities.h"

#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

namespace PS_AGONY
{
	// Convex polygon local-space geometry: vertices plus precomputed outward unit edge normals.
	// Single allocation laid out as [vertices | normals]. normals()[i] belongs to edge i -> i+1.
	// Degenerate edges have a zero normal. Immutable after construction.
	struct PolygonGeometry
	{
	private:
		static constexpr Real DEGENERATE_EDGE_LENGTH_SQUARED = Real(1e-8);

		std::unique_ptr<Vec2[]> dataPtr;
		size_t verticesCount = 0;

		void allocate(size_t count)
		{
			verticesCount = count;
			dataPtr.reset(new Vec2[count * 2]);
		}

		void computeNormals() noexcept
		{
			const Vec2* v = dataPtr.get();
			Vec2* n = dataPtr.get() + verticesCount;
			for (size_t i = 0; i < verticesCount; i++)
			{
				const size_t next = (i + 1 == verticesCount) ? 0 : i + 1;
				const Vec2 edge = v[next] - v[i];
				Vec2 normal = { edge.y, -edge.x };
				const Real lenSquared = normal.x * normal.x + normal.y * normal.y;
				if (lenSquared >= DEGENERATE_EDGE_LENGTH_SQUARED) [[likely]]
					normal *= Real(1) / std::sqrt(lenSquared);
				else
					normal = Vec2(0, 0);
				n[i] = normal;
			}
		}

	public:
		PolygonGeometry() = default;

		explicit PolygonGeometry(const std::vector<Vec2>& vec)
		{
			allocate(vec.size());
			std::copy(vec.begin(), vec.end(), dataPtr.get());
			computeNormals();
		}

		explicit PolygonGeometry(std::vector<Vec2>&& vec)
		{
			allocate(vec.size());
			std::move(vec.begin(), vec.end(), dataPtr.get());
			computeNormals();
		}

		explicit PolygonGeometry(const Vec2* verticesPtr, size_t count)
		{
			allocate(count);
			std::copy(verticesPtr, verticesPtr + count, dataPtr.get());
			computeNormals();
		}

		PolygonGeometry(const PolygonGeometry&) = delete;
		PolygonGeometry& operator=(const PolygonGeometry&) = delete;

		PolygonGeometry(PolygonGeometry&&) = default;
		PolygonGeometry& operator=(PolygonGeometry&&) = default;

		const Vec2* vertices() const noexcept { return dataPtr.get(); }
		const Vec2* normals() const noexcept { return dataPtr.get() + verticesCount; }
		size_t size() const noexcept { return verticesCount; }

		const Vec2* begin() const noexcept { return vertices(); }
		const Vec2* end() const noexcept { return vertices() + verticesCount; }

		size_t getMemoryUsage() const noexcept { return verticesCount * 2 * sizeof(Vec2); }
	};

	struct PolygonSoA
	{
		SimdAlignedVector<ColliderIndex> colliderIndices;
		std::vector<PolygonGeometry> geometry;

		void append(
			ColliderIndex colliderIndex,
			PolygonGeometry&& polygonGeometry
		)
		{
			this->colliderIndices.push_back(colliderIndex);
			this->geometry.emplace_back(std::move(polygonGeometry));
		}

		size_t getCount() const noexcept { return colliderIndices.size(); }

		size_t getMemoryUsage() const noexcept
		{
			size_t total =
				PS_AGONY::getVectorMemoryUsage(colliderIndices) +
				PS_AGONY::getVectorMemoryUsage(geometry);
			for (const auto& g : geometry)
				total += g.getMemoryUsage();
			return total;
		}
	};

	class PolygonSoAViewer
	{
		size_t count = 0;
	public:
		const ColliderIndex* colliderIndices = nullptr;
		const PolygonGeometry* geometry = nullptr;

		PolygonSoAViewer() = default;

		explicit PolygonSoAViewer(const PolygonSoA& data) :
			count(data.getCount()),
			colliderIndices(data.colliderIndices.data()),
			geometry(data.geometry.data())
		{
		}

		size_t getCount() const noexcept { return count; }
	};
}