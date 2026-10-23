#pragma once
#include "PhysicsGeometry.h"
#include "PolygonDecomposition.h"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace PS_AGONY2D::PhysicsGeometry
{
    enum class PolygonBuildError : uint8_t
    {
        None,
        TooFewVertices,
        InvalidOutline,       // Degenerate or self-intersecting.
        DecompositionFailed
    };

    const char* polygonBuildErrorMessage(PolygonBuildError error) noexcept;

    // Single source of truth for turning user vertices into a body-ready polygon.
    // Shared by ObjectManager::createPolygon and the creator preview.
    struct PolygonBuild
    {
        std::vector<Vec2> outline;                 // Cleaned, CCW, shifted so the true COM is the origin.
        std::vector<std::vector<Vec2>> pieces;     // Convex pieces, same (COM-centered) frame.
        Vec2 trueCenterOfMass = Vec2(0);           // In the caller's original vertex frame.
        Vec2 neededCenterOfMass = Vec2(0);         // Requested pivot relative to true COM.
        Real inertia = Real(0);                    // About the needed pivot.
        PolygonBuildError error = PolygonBuildError::None;

        bool ok() const noexcept { return error == PolygonBuildError::None; }
    };

    PolygonBuild buildPolygon(
        std::span<const Vec2> vertices,
        ConcaveDecompositionMethod concaveMethod,
        Real mass,
        std::optional<Vec2> centerOfMass,
        ConvexDecompositionMethod convexMethod = ConvexDecompositionMethod::Fan
    );
}