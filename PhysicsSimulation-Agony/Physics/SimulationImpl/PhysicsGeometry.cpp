#include "PhysicsGeometry.h"
#include "PolygonBuild.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace PS_AGONY::PhysicsGeometry
{
    Real calculateCircleInertia(Real mass, Real radius, Vec2 centerOfMass)
    {
        const Real radiusSquared = radius * radius;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return (Real(0.5) * radiusSquared + deltaSquared) * mass;
    }

    Real calculateBoxInertia(Real mass, Real width, Real height, Vec2 centerOfMass)
    {
        constexpr Real div = 1.0 / 12.0;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return mass * (div * (width * width + height * height) + deltaSquared);
    }

    std::pair<Real, Vec2> calculatePolygonInertia(
        Real mass,
        VerticesContainer& verticesContainer,
        std::optional<Vec2> centerOfMass
    )
    {
        const size_t verticesCount = verticesContainer.size();
        if (verticesCount < 3)
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        Real signedArea = Real(0);
        Real cx = Real(0);
        Real cy = Real(0);
        Real xx = Real(0);
        Real yy = Real(0);

        const bool computeCOM = !centerOfMass.has_value();
        const Vec2* verticesPtr = verticesContainer.data();

        for (size_t i = 0; i < verticesCount; i++)
        {
            const Vec2& p0 = verticesPtr[i];
            const Vec2& p1 = verticesPtr[(i + 1) % verticesCount];

            Real cross = p0.x * p1.y - p1.x * p0.y;
            signedArea += cross;

            if (computeCOM)
            {
                cx += (p0.x + p1.x) * cross;
                cy += (p0.y + p1.y) * cross;
            }

            // Area moments about origin.
            xx += (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) * cross;
            yy += (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) * cross;
        }

        // Precondition: CCW winding (guaranteed by preparePolygonOutline); a CW input yields zero inertia.

        signedArea *= Real(0.5);
        const Real absoluteArea = signedArea;
        if (absoluteArea < std::numeric_limits<Real>::epsilon())
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        // Determine final Center of Mass.
        Vec2 finalCOM;
        if (computeCOM)
        {
            finalCOM = Vec2(
                cx / (Real(6) * signedArea),
                cy / (Real(6) * signedArea)
            );
        }
        else
        {
            finalCOM = centerOfMass.value();
        }

        xx /= Real(12);
        yy /= Real(12);

        // Local/World polar moment of area scaled to mass moment
        Real inertia = (mass / absoluteArea) * (xx + yy);

        if (!computeCOM)
        {
            Real deltaSquared = glm::dot(finalCOM, finalCOM);
            inertia += mass * deltaSquared;
        }
        else
        {
            inertia -= mass * glm::dot(finalCOM, finalCOM);
        }

        return { inertia, finalCOM };
    }

    std::optional<std::vector<Vec2>> preparePolygonOutline(std::span<const Vec2> vertices)
    {
        std::vector<Vec2> out = filterDuplicateVertices(vertices);

        // Strip straight (collinear, same-direction) vertices; repeat since removals can expose new ones.
        bool removed = true;
        while (removed && out.size() >= 3)
        {
            removed = false;
            const size_t n = out.size();
            std::vector<Vec2> kept;
            kept.reserve(n);
            for (size_t i = 0; i < n; ++i)
            {
                const Vec2 e1 = out[i] - out[(i + n - 1) % n];
                const Vec2 e2 = out[(i + 1) % n] - out[i];
                const bool straight = glm::dot(e1, e2) > Real(0) &&
                    std::abs(e1.x * e2.y - e1.y * e2.x) <= Real(1e-6) * glm::length(e1) * glm::length(e2);
                if (straight) removed = true;
                else kept.push_back(out[i]);
            }
            out = std::move(kept);
        }
        if (out.size() < 3) return std::nullopt;

        Real area2 = Real(0);
        for (size_t i = 0, n = out.size(); i < n; ++i)
        {
            const Vec2& p0 = out[i];
            const Vec2& p1 = out[(i + 1) % n];
            area2 += p0.x * p1.y - p1.x * p0.y;
        }
        if (area2 < Real(0)) std::reverse(out.begin(), out.end()); // Auto-fix CW -> CCW.
        if (std::abs(area2) * Real(0.5) <= std::numeric_limits<Real>::epsilon()) return std::nullopt;

        if (!isSimplePolygon(out)) return std::nullopt;
        return out;
    }

    std::vector<Vec2> filterDuplicateVertices(std::span<const Vec2> vertices, Real epsilon)
    {
        if (vertices.empty())
        {
            return {};
        }

        const Real epsSq = epsilon * epsilon;
        auto isClose = [epsSq](const Vec2& a, const Vec2& b)
            {
                const auto dpos = a - b;
                return glm::dot(dpos, dpos) <= epsSq;
            };

        std::vector<Vec2> uniqueVertices;
        uniqueVertices.reserve(vertices.size());

        // Step 1: Forward pass collapsing contiguous duplicate neighbors in sequence.
        for (const Vec2& current : vertices)
        {
            if (uniqueVertices.empty() || !isClose(uniqueVertices.back(), current))
            {
                uniqueVertices.push_back(current);
            }
        }

        // Step 2: Cyclic wrap-around pass (checking boundary adjacency between last and first vertex).
        while (uniqueVertices.size() > 1 && isClose(uniqueVertices.back(), uniqueVertices.front()))
        {
            uniqueVertices.pop_back();
        }

        return uniqueVertices;
    }

    const char* polygonBuildErrorMessage(PolygonBuildError error) noexcept
    {
        switch (error)
        {
        case PolygonBuildError::None:                return "";
        case PolygonBuildError::TooFewVertices:      return "Needs at least 3 vertices.";
        case PolygonBuildError::InvalidOutline:      return "Outline is degenerate or self-intersecting.";
        case PolygonBuildError::DecompositionFailed: return "Convex decomposition produced no pieces.";
        }
        return "Unknown error.";
    }

    PolygonBuild buildPolygon(
        std::span<const Vec2> vertices,
        DecompositionMethod method,
        Real mass,
        std::optional<Vec2> centerOfMass
    )
    {
        PolygonBuild result;
        if (vertices.size() < 3)
        {
            result.error = PolygonBuildError::TooFewVertices;
            return result;
        }

        std::optional<std::vector<Vec2>> outline = preparePolygonOutline(vertices);
        if (!outline)
        {
            result.error = PolygonBuildError::InvalidOutline;
            return result;
        }
        result.outline = std::move(*outline); // Deduped, no collinear points, CCW, simple.

        // Shoelace formulas are valid for any simple polygon, so the full (possibly concave) outline is used.
        VerticesContainer container{ result.outline.data(), result.outline.size() };
        const auto [inertiaAboutTrueCOM, trueCOM] = calculatePolygonInertia(mass, container, std::nullopt);

        result.trueCenterOfMass = trueCOM;
        result.neededCenterOfMass = centerOfMass.value_or(trueCOM) - trueCOM;

        // Origin == true COM from here on.
        for (Vec2& v : result.outline) v -= trueCOM;

        // Parallel-axis shift to the requested pivot.
        result.inertia = inertiaAboutTrueCOM + mass * glm::dot(result.neededCenterOfMass, result.neededCenterOfMass);

        result.pieces = decomposeConvex(result.outline, method);
        if (result.pieces.empty()) result.error = PolygonBuildError::DecompositionFailed;
        return result;
    }
}