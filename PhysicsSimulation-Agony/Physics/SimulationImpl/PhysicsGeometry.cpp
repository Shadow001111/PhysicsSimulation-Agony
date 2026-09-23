#include "PhysicsGeometry.h"

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

        // If winding order is clockwise, reverse the container to make it counter-clockwise.
        // Since all accumulated values are linear with respect to 'cross', we can just negate them.
        if (signedArea < Real(0))
        {
            std::reverse(verticesContainer.begin(), verticesContainer.end());
            signedArea = -signedArea;
            xx = -xx;
            yy = -yy;
            if (computeCOM)
            {
                cx = -cx;
                cy = -cy;
            }
        }

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

        // If COM was explicitly provided, treat vertices as local space 
        // and shift to world origin via the parallel axis theorem.
        if (!computeCOM)
        {
            Real deltaSquared = glm::dot(finalCOM, finalCOM);
            inertia += mass * deltaSquared;
        }
        else
        {
            // Moments were accumulated about the origin; move them to the COM (parallel axis theorem).
            inertia -= mass * glm::dot(finalCOM, finalCOM);
        }

        return { inertia, finalCOM };
    }

    std::vector<Vec2> filterDuplicateVertices(const Vec2* localVertices, size_t verticesCount)
    {
        std::vector<Vec2> uniqueVertices;
        uniqueVertices.reserve(verticesCount);
        for (size_t i = 0; i < verticesCount; ++i)
        {
            const Vec2& current = localVertices[i];

            bool isDuplicate = false;
            for (const Vec2& existing : uniqueVertices)
            {
                if (existing.x == current.x && existing.y == current.y)
                {
                    isDuplicate = true;
                    break;
                }
            }

            if (!isDuplicate)
            {
                uniqueVertices.push_back(current);
            }
        }
        return uniqueVertices;
    }

    namespace
    {
        constexpr Real kConvexEpsilon = Real(1e-9);

        inline Real crossZ(const Vec2& a, const Vec2& b, const Vec2& c)
        {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        }

        Real signedAreaIndexed(const std::vector<int>& poly, const std::vector<Vec2>& v)
        {
            Real area = Real(0);
            const size_t n = poly.size();
            for (size_t i = 0; i < n; ++i)
            {
                const Vec2& p0 = v[poly[i]];
                const Vec2& p1 = v[poly[(i + 1) % n]];
                area += p0.x * p1.y - p1.x * p0.y;
            }
            return area * Real(0.5);
        }

        bool pointInTriangle(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c)
        {
            const Real d1 = crossZ(a, b, p);
            const Real d2 = crossZ(b, c, p);
            const Real d3 = crossZ(c, a, p);
            const bool hasNeg = (d1 < Real(0)) || (d2 < Real(0)) || (d3 < Real(0));
            const bool hasPos = (d1 > Real(0)) || (d2 > Real(0)) || (d3 > Real(0));
            return !(hasNeg && hasPos);
        }

        // Convexity check over indices into the shared vertex buffer. Assumes CCW winding.
        bool isConvexIndexed(const std::vector<int>& poly, const std::vector<Vec2>& v)
        {
            const size_t n = poly.size();
            if (n < 4) return true; // triangles are always convex

            for (size_t i = 0; i < n; ++i)
            {
                const Vec2& a = v[poly[(i + n - 1) % n]];
                const Vec2& b = v[poly[i]];
                const Vec2& c = v[poly[(i + 1) % n]];
                if (crossZ(a, b, c) < -kConvexEpsilon)
                {
                    return false;
                }
            }
            return true;
        }

        // Ear-clipping triangulation. 'poly' must be a CCW list of indices into 'v'.
        std::vector<std::vector<int>> triangulateEarClip(std::vector<int> poly, const std::vector<Vec2>& v)
        {
            std::vector<std::vector<int>> triangles;

            while (poly.size() > 3)
            {
                const size_t n = poly.size();
                bool clipped = false;

                for (size_t i = 0; i < n; ++i)
                {
                    const size_t prev = (i + n - 1) % n;
                    const size_t next = (i + 1) % n;
                    const Vec2& a = v[poly[prev]];
                    const Vec2& b = v[poly[i]];
                    const Vec2& c = v[poly[next]];

                    if (crossZ(a, b, c) <= kConvexEpsilon) continue; // reflex/collinear: not an ear tip

                    bool anyInside = false;
                    for (size_t k = 0; k < n; ++k)
                    {
                        if (k == prev || k == i || k == next) continue;
                        if (pointInTriangle(v[poly[k]], a, b, c))
                        {
                            anyInside = true;
                            break;
                        }
                    }
                    if (anyInside) continue;

                    triangles.push_back({ poly[prev], poly[i], poly[next] });
                    poly.erase(poly.begin() + i);
                    clipped = true;
                    break;
                }

                if (!clipped)
                {
                    // Degenerate/numerical fallback (e.g. near-collinear input): fan-triangulate
                    // the remainder instead of looping forever.
                    for (size_t i = 1; i + 1 < poly.size(); ++i)
                    {
                        triangles.push_back({ poly[0], poly[i], poly[i + 1] });
                    }
                    poly.clear();
                    break;
                }
            }

            if (poly.size() == 3)
            {
                triangles.push_back({ poly[0], poly[1], poly[2] });
            }
            return triangles;
        }

        // Finds an edge shared (in opposite directions) by CCW pieces 'a' and 'b'.
        // On success, edgeA/edgeB are the positions of each edge's start vertex within its list.
        bool findSharedEdge(const std::vector<int>& a, const std::vector<int>& b, size_t& edgeA, size_t& edgeB)
        {
            const size_t na = a.size();
            const size_t nb = b.size();
            for (size_t i = 0; i < na; ++i)
            {
                const int a0 = a[i];
                const int a1 = a[(i + 1) % na];
                for (size_t j = 0; j < nb; ++j)
                {
                    if (b[j] == a1 && b[(j + 1) % nb] == a0)
                    {
                        edgeA = i;
                        edgeB = j;
                        return true;
                    }
                }
            }
            return false;
        }

        // Fuses 'a' and 'b' across the shared edge at (edgeA in a, edgeB in b) into one CCW loop.
        std::vector<int> mergeAcrossEdge(const std::vector<int>& a, const std::vector<int>& b, size_t edgeA, size_t edgeB)
        {
            const size_t na = a.size();
            const size_t nb = b.size();

            std::vector<int> merged;
            merged.reserve(na + nb - 2);

            for (size_t k = 0; k <= edgeA; ++k) merged.push_back(a[k]);
            for (size_t c = 0; c < nb - 2; ++c) merged.push_back(b[(edgeB + 2 + c) % nb]);
            for (size_t k = edgeA + 1; k < na; ++k) merged.push_back(a[k]);

            return merged;
        }

        // Greedy Hertel-Mehlhorn merge: repeatedly fuses adjacent pieces across a shared edge
        // whenever the result stays convex, shrinking the piece count toward-optimal.
        void mergeConvexPieces(std::vector<std::vector<int>>& pieces, const std::vector<Vec2>& v)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                for (size_t a = 0; a < pieces.size() && !changed; ++a)
                {
                    for (size_t b = a + 1; b < pieces.size() && !changed; ++b)
                    {
                        size_t edgeA, edgeB;
                        if (!findSharedEdge(pieces[a], pieces[b], edgeA, edgeB)) continue;

                        std::vector<int> merged = mergeAcrossEdge(pieces[a], pieces[b], edgeA, edgeB);
                        if (!isConvexIndexed(merged, v)) continue;

                        pieces[a] = std::move(merged);
                        pieces.erase(pieces.begin() + b);
                        changed = true;
                    }
                }
            }
        }
    }

    bool isConvexPolygon(const std::vector<Vec2>& vertices)
    {
        const size_t n = vertices.size();
        if (n < 4) return true;

        Real area = Real(0);
        for (size_t i = 0; i < n; ++i)
        {
            const Vec2& p0 = vertices[i];
            const Vec2& p1 = vertices[(i + 1) % n];
            area += p0.x * p1.y - p1.x * p0.y;
        }
        const Real sign = area < Real(0) ? Real(-1) : Real(1); // tolerate either winding

        for (size_t i = 0; i < n; ++i)
        {
            const Vec2& a = vertices[(i + n - 1) % n];
            const Vec2& b = vertices[i];
            const Vec2& c = vertices[(i + 1) % n];
            if (sign * crossZ(a, b, c) < -kConvexEpsilon)
            {
                return false;
            }
        }
        return true;
    }

    std::vector<std::vector<Vec2>> decomposeConvex(const std::vector<Vec2>& vertices)
    {
        if (vertices.size() < 3)
        {
            return {};
        }

        if (isConvexPolygon(vertices))
        {
            // Fast path: no triangulation needed, just normalize winding for downstream consistency.
            std::vector<Vec2> single = vertices;
            const size_t n = single.size();
            Real area = Real(0);
            for (size_t i = 0; i < n; ++i)
            {
                area += single[i].x * single[(i + 1) % n].y - single[(i + 1) % n].x * single[i].y;
            }
            if (area < Real(0))
            {
                std::reverse(single.begin(), single.end());
            }
            return { std::move(single) };
        }

        std::vector<int> indices(vertices.size());
        for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<int>(i);
        if (signedAreaIndexed(indices, vertices) < Real(0))
        {
            std::reverse(indices.begin(), indices.end());
        }

        std::vector<std::vector<int>> pieces = triangulateEarClip(std::move(indices), vertices);
        mergeConvexPieces(pieces, vertices);

        std::vector<std::vector<Vec2>> result;
        result.reserve(pieces.size());
        for (const std::vector<int>& piece : pieces)
        {
            std::vector<Vec2> poly;
            poly.reserve(piece.size());
            for (int idx : piece) poly.push_back(vertices[idx]);
            result.push_back(std::move(poly));
        }
        return result;
    }
}