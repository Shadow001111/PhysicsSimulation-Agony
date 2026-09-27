#pragma once

#include "Physics/GlmTypes.h"
#include <span>
#include <vector>

namespace PS_AGONY::PhysicsGeometry
{
    enum class DecompositionMethod
    {
        EarClipping, // Triangles only.
        Bayazit, // The best. Guarantees min count of convex parts.
        ReflexAngleBisector // Worse than 'bayazit, but has different topology, which MAY be better, at least by visuals.
    };

    // Returns true if 'vertices' form a convex polygon (either winding order).
    // Fewer than 4 vertices is trivially convex.
    bool isConvexPolygon(std::span<const Vec2> vertices);

    // Individual decomposition algorithm implementations.
    std::vector<std::vector<Vec2>> decomposeEarClipping(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeBayazit(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeReflexBisector(std::span<const Vec2> vertices);

    // Main entry point decomposing simple polygons into non-overlapping convex polygons.
    std::vector<std::vector<Vec2>> decomposeConvex(
        std::span<const Vec2> vertices,
        DecompositionMethod method = DecompositionMethod::Bayazit
    );
}