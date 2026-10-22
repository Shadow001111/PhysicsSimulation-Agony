#pragma once

#include "Physics/Types/Types.h"
#include <cstddef>
#include <span>
#include <vector>

namespace PS_AGONY::PhysicsGeometry
{
    enum class ConcaveDecompositionMethod
    {
        EarClipping, // Triangles only.
        Bayazit, // Recursive split at reflex vertices + greedy merge. Few parts, not guaranteed minimal.
        ReflexAngleBisector // Worse than 'bayazit, but has different topology, which MAY be better, at least by visuals.
    };

    enum class ConvexDecompositionMethod
    {
        Fan,   // Fan from vertex 0 in chunks. O(n), fewest pieces (ceil((n-2)/(max-2))), but thin slivers.
        Bisect // Recursive cut between opposite vertices. Compact, balanced pieces; slightly more of them.
    };

    // Returns true if CCW 'vertices' form a convex polygon (winding is NOT normalized).
    // Fewer than 4 vertices is trivially convex.
    bool isConvexPolygon(std::span<const Vec2> vertices);

    // True if the closed outline is simple: no edge crossings/touching, no spikes,
    // no coincident non-adjacent vertices. O(n^2); meant for creation-time validation.
    bool isSimplePolygon(std::span<const Vec2> vertices);

    // Individual decomposition algorithm implementations.
    std::vector<std::vector<Vec2>> decomposeConcaveEarClipping(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeConcaveBayazit(std::span<const Vec2> vertices);
    std::vector<std::vector<Vec2>> decomposeConcaveReflexBisector(std::span<const Vec2> vertices);

    // Main entry point decomposing simple polygons into non-overlapping convex polygons.
    // Precondition: simple, CCW, no duplicate/collinear vertices (see PhysicsGeometry::preparePolygonOutline).
    std::vector<std::vector<Vec2>> decomposeConcave(
        std::span<const Vec2> vertices,
        ConcaveDecompositionMethod method = ConcaveDecompositionMethod::Bayazit
    );

    // Splits a CONVEX CCW polygon into convex pieces of at most maxVertexCount vertices (clamped to >= 3).
    // A polygon already within the limit is returned as a single piece.
    std::vector<std::vector<Vec2>> decomposeConvexFan(std::span<const Vec2> vertices, size_t maxVertexCount);
    std::vector<std::vector<Vec2>> decomposeConvexBisect(std::span<const Vec2> vertices, size_t maxVertexCount);
    std::vector<std::vector<Vec2>> decomposeConvex(
        std::span<const Vec2> vertices,
        size_t maxVertexCount,
        ConvexDecompositionMethod method = ConvexDecompositionMethod::Fan
    );

    // In-place: replaces every convex piece exceeding maxVertexCount with its decomposition.
    // No-op (no allocation) when all pieces already fit. Call after decomposeConcave().
    void limitConvexPieceVertexCount(
        std::vector<std::vector<Vec2>>& pieces,
        size_t maxVertexCount,
        ConvexDecompositionMethod method = ConvexDecompositionMethod::Fan
    );
}