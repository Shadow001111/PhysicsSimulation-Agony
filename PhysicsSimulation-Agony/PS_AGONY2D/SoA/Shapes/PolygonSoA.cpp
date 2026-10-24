#include "PolygonSoA.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace PS_AGONY2D
{
	void PolygonGeometry::allocate(size_t count)
	{
		verticesCount = count;
		dataPtr.reset(new Vec2[count * 2]);
	}

	void PolygonGeometry::computeNormals() noexcept
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

	PolygonGeometry::PolygonGeometry(const std::vector<Vec2>& vec)
	{
		allocate(vec.size());
		std::copy(vec.begin(), vec.end(), dataPtr.get());
		computeNormals();
	}

	PolygonGeometry::PolygonGeometry(std::vector<Vec2>&& vec)
	{
		allocate(vec.size());
		std::move(vec.begin(), vec.end(), dataPtr.get());
		computeNormals();
	}

	PolygonGeometry::PolygonGeometry(const Vec2* verticesPtr, size_t count)
	{
		allocate(count);
		std::copy(verticesPtr, verticesPtr + count, dataPtr.get());
		computeNormals();
	}

	size_t PolygonGeometry::getMemoryUsage() const noexcept
	{
		return verticesCount * 2 * sizeof(Vec2);
	}

	void PolygonSoA::append(
		ColliderIndex colliderIndex,
		PolygonGeometry&& polygonGeometry
	)
	{
		this->colliderIndices.push_back(colliderIndex);
		this->geometry.emplace_back(std::move(polygonGeometry));
	}

	void PolygonSoA::swapWithBack(size_t index)
	{
		if (index >= colliderIndices.size()) [[unlikely]]
		{
			return;
		}

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD
	}

	void PolygonSoA::popBack()
	{
		if (colliderIndices.empty()) [[unlikely]]
		{
			return;
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	size_t PolygonSoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		for (const auto& g : geometry)
		{
			total += g.getMemoryUsage();
		}
		return total;
	}

	PolygonSoAViewer::PolygonSoAViewer(const PolygonSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
