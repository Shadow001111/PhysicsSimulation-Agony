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
}