#pragma once

#include "Physics/GlmTypes.h"
#include <span>
#include <vector>

namespace PS_AGONY::PhysicsGeometry
{
    enum class DecompositionMethod
    {
        EarClipping, // Triangles only.
        Bayazit, // Recursive split at reflex vertices + greedy merge. Few parts, not guaranteed minimal.
        ReflexAngleBisector // Worse than 'bayazit, but has different topology, which MAY be better, at least by visuals.
    };

    // Returns true if CCW 'vertices' form a convex polygon (winding is NOT normalized).
    // Fewer than 4 vertices is trivially convex.
    bool isConvexPolygon(std::span<const Vec2> vertices);

    // True if the closed outline is simple: no edge crossings/touching, no spikes,
    // no coincident non-adjacent vertices. O(n^2); meant for creation-time validation.
    bool isSimplePolygon(std::span<const Vec2> vertices);

    // Individual decomposition algorithm implementations.
    std::vector<std::vector<Vec2>> decomposeEarClipping(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeBayazit(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeReflexBisector(std::span<const Vec2> vertices);

    // Main entry point decomposing simple polygons into non-overlapping convex polygons.
    // Precondition: simple, CCW, no duplicate/collinear vertices (see PhysicsGeometry::preparePolygonOutline).
    std::vector<std::vector<Vec2>> decomposeConvex(
        std::span<const Vec2> vertices,
        DecompositionMethod method = DecompositionMethod::Bayazit
    );
}