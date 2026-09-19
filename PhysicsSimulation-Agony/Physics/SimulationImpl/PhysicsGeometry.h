#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/SoA/Shapes/PolygonSoA.h"

#include <utility>
#include <optional>
#include <vector>

namespace PS_AGONY::PhysicsGeometry
{
	Real calculateCircleInertia(
		Real mass,
		Real radius,
		Vec2 centerOfMass
	);

	Real calculateBoxInertia(
		Real mass,
		Real width, Real height,
		Vec2 centerOfMass
	);

	std::pair<Real, Vec2> calculatePolygonInertia(
		Real mass,
		VerticesContainer& verticesContainer,
		std::optional<Vec2> centerOfMass = std::nullopt
	);

	// Filters out consecutive/any duplicate vertices from a raw vertex buffer.
	// Shared by createPolygon() and createPolygonCollider().
	std::vector<Vec2> filterDuplicateVertices(const Vec2* localVertices, size_t verticesCount);

	// Returns true if 'vertices' form a convex polygon (either winding order).
	// Fewer than 4 vertices is trivially convex.
	bool isConvexPolygon(const std::vector<Vec2>& vertices);

	// Decomposes a simple (non-self-intersecting) polygon - convex or concave, either winding -
	// into a small set of non-overlapping convex polygons, via ear-clipping triangulation
	// followed by greedy convex merging (Hertel-Mehlhorn: at most 4x the optimal piece count,
	// usually far fewer in practice). Every returned piece is wound CCW.
	// If the input is already convex, a single CCW-normalized piece is returned with no
	// triangulation cost.
	// NOTE: this is an O(n^2)-ish one-time authoring-cost routine (triangulate + merge),
	// meant to run once at collider-creation time, not per physics step.
	// Shared by createPolygon() and createPolygonCollider().
	std::vector<std::vector<Vec2>> decomposeConvex(const std::vector<Vec2>& vertices);
}