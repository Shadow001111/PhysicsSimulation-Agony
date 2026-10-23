#pragma once
#include "Physics/ContainerUtilities.h"
#include "Physics/Types/Types.h"
#include <memory>
#include <vector>

namespace PS_AGONY2D
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

		void allocate(size_t count);
		void computeNormals() noexcept;

	public:
		PolygonGeometry() = default;
		explicit PolygonGeometry(const std::vector<Vec2>& vec);
		explicit PolygonGeometry(std::vector<Vec2>&& vec);
		explicit PolygonGeometry(const Vec2* verticesPtr, size_t count);

		PolygonGeometry(const PolygonGeometry&) = delete;
		PolygonGeometry& operator=(const PolygonGeometry&) = delete;

		PolygonGeometry(PolygonGeometry&&) = default;
		PolygonGeometry& operator=(PolygonGeometry&&) = default;

		const Vec2* vertices() const noexcept { return dataPtr.get(); }
		const Vec2* normals() const noexcept { return dataPtr.get() + verticesCount; }
		size_t size() const noexcept { return verticesCount; }

		const Vec2* begin() const noexcept { return vertices(); }
		const Vec2* end() const noexcept { return vertices() + verticesCount; }

		size_t getMemoryUsage() const noexcept;
	};

	// Define all scalar SoA fields.
	#define PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(F) \
		F(colliderIndices, ColliderIndex) \
		F(geometry, PolygonGeometry)

	struct PolygonSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(DECLARE_FIELD)
		#undef DECLARE_FIELD

		void append(
			ColliderIndex colliderIndex,
			PolygonGeometry&& polygonGeometry
		);

		void swapWithBack(size_t index);
		void popBack();

		size_t getCount() const noexcept { return colliderIndices.size(); }
		size_t getMemoryUsage() const noexcept;
	};

	class PolygonSoAViewer
	{
		size_t count = 0;
	public:
		#define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
		PS_AGONY_POLYGON_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
		#undef DECLARE_VIEWER_FIELD

		PolygonSoAViewer() = default;
		explicit PolygonSoAViewer(const PolygonSoA& data) noexcept;

		size_t getCount() const noexcept { return count; }
	};
}
