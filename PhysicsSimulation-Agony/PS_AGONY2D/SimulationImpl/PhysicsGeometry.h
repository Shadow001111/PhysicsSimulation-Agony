#pragma once

#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/SoA/Shapes/PolygonSoA.h"
#include "PolygonDecomposition.h"

#include <utility>
#include <optional>
#include <vector>
#include <span>

namespace PS_AGONY2D::PhysicsGeometry
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
		std::span<const Vec2> vertices,
		std::optional<Vec2> centerOfMass = std::nullopt
	);

	Real calculateCapsuleInertia(
		Real mass,
		Real length, Real radius,
		Vec2 centerOfMass
	);

	// Areas. 'length' is the FULL capsule length (same convention as calculateCapsuleInertia).
	// Polygon expects CCW winding; a CW or degenerate input returns 0.
	Real calculateCircleArea(Real radius);
	Real calculateBoxArea(Real width, Real height);
	Real calculateCapsuleArea(Real length, Real radius);
	Real calculatePolygonArea(std::span<const Vec2> vertices);

	// Largest circle around the polygon's local origin that fits inside it. 0 when the origin is outside.
	Real calculatePolygonInRadius(const PolygonGeometry& geometry);

	// Creation-time sanitizer: dedupes neighbors, strips collinear vertices, auto-reverses CW to CCW.
	// Returns nullopt if the result is degenerate (<3 vertices, ~zero area) or not simple (self-intersecting).
	std::optional<std::vector<Vec2>> preparePolygonOutline(std::span<const Vec2> vertices);

	// Filters out adjacent (neighboring) duplicate vertices using a distance epsilon check.
	// Preserves non-adjacent identical vertices to maintain polygon topology.
	std::vector<Vec2> filterDuplicateVertices(
		std::span<const Vec2> vertices,
		Real epsilon = Real(1e-5)
	);
}