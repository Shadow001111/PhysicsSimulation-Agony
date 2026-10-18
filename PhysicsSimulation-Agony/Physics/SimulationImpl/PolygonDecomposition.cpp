#include "PolygonDecomposition.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace PS_AGONY::PhysicsGeometry
{
    namespace
    {
        constexpr Real kConvexEpsilon = Real(1e-9);

        inline Real cross2D(Vec2 a, Vec2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        inline Real crossZ(const Vec2& a, const Vec2& b, const Vec2& c)
        {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        }

        Real polygonArea(std::span<const Vec2> poly)
        {
            const size_t n = poly.size();
            if (n < 3) return Real(0);
            Real area = Real(0);
            for (size_t i = 0; i < n; ++i)
            {
                const Vec2& p0 = poly[i];
                const Vec2& p1 = poly[(i + 1) % n];
                area += p0.x * p1.y - p1.x * p0.y;
            }
            return area * Real(0.5);
        }

        bool isReflexVertex(const Vec2& prev, const Vec2& curr, const Vec2& next)
        {
            return crossZ(prev, curr, next) < -kConvexEpsilon;
        }

        bool pointInPolygon(Vec2 pt, std::span<const Vec2> polygon)
        {
            const size_t n = polygon.size();
            bool inside = false;
            for (size_t i = 0; i < n; ++i)
            {
                Vec2 p1 = polygon[i];
                Vec2 p2 = polygon[(i + 1) % n];
                if ((p1.y > pt.y) != (p2.y > pt.y))
                {
                    Real xInt = (p2.x - p1.x) * (pt.y - p1.y) / (p2.y - p1.y + Real(1e-12)) + p1.x;
                    if (pt.x < xInt)
                    {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

        bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d)
        {
            auto ccw = [](Vec2 p1, Vec2 p2, Vec2 p3) {
                return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
                };

            Real cp1 = ccw(a, b, c);
            Real cp2 = ccw(a, b, d);
            Real cp3 = ccw(c, d, a);
            Real cp4 = ccw(c, d, b);

            return ((cp1 > kConvexEpsilon && cp2 < -kConvexEpsilon) || (cp1 < -kConvexEpsilon && cp2 > kConvexEpsilon)) &&
                ((cp3 > kConvexEpsilon && cp4 < -kConvexEpsilon) || (cp3 < -kConvexEpsilon && cp4 > kConvexEpsilon));
        }

        constexpr Real kTouchEpsSq = Real(1e-12); // (1e-6)^2

        inline Real pointSegDistSq(const Vec2& p, const Vec2& a, const Vec2& b)
        {
            const Vec2 ab = b - a;
            const Real len2 = glm::dot(ab, ab);
            Real t = len2 > Real(0) ? glm::dot(p - a, ab) / len2 : Real(0);
            t = std::clamp(t, Real(0), Real(1));
            const Vec2 d = p - (a + ab * t);
            return glm::dot(d, d);
        }

        bool isValidDiagonal(std::span<const Vec2> polygon, size_t i, size_t j)
        {
            const size_t n = polygon.size();
            if (i == j || (i + 1) % n == j || (j + 1) % n == i) return false;

            Vec2 pi = polygon[i];
            Vec2 pj = polygon[j];

            for (size_t k = 0; k < n; ++k)
            {
                size_t kNext = (k + 1) % n;
                if (k != i && k != j && pointSegDistSq(polygon[k], pi, pj) <= kTouchEpsSq) return false; // Diagonal grazes a vertex.
                if (k == i || k == j || kNext == i || kNext == j) continue;
                if (segmentsIntersect(pi, pj, polygon[k], polygon[kNext])) return false;
            }

            Vec2 midpt = (pi + pj) * Real(0.5);
            if (!pointInPolygon(midpt, polygon)) return false;

            // Cone test at i
            Vec2 prevI = polygon[(i + n - 1) % n];
            Vec2 currI = polygon[i];
            Vec2 nextI = polygon[(i + 1) % n];
            Vec2 vDiag = pj - currI;
            Vec2 vIn = nextI - currI;
            Vec2 vOut = prevI - currI;

            if (isReflexVertex(prevI, currI, nextI))
            {
                if (cross2D(vIn, vDiag) <= kConvexEpsilon && cross2D(vDiag, vOut) <= kConvexEpsilon) return false;
            }
            else
            {
                if (cross2D(vIn, vDiag) <= kConvexEpsilon || cross2D(vDiag, vOut) <= kConvexEpsilon) return false;
            }

            // Cone test at j
            Vec2 prevJ = polygon[(j + n - 1) % n];
            Vec2 currJ = polygon[j];
            Vec2 nextJ = polygon[(j + 1) % n];
            Vec2 vDiagJ = currI - currJ;
            Vec2 vInJ = nextJ - currJ;
            Vec2 vOutJ = prevJ - currJ;

            if (isReflexVertex(prevJ, currJ, nextJ))
            {
                if (cross2D(vInJ, vDiagJ) <= kConvexEpsilon && cross2D(vDiagJ, vOutJ) <= kConvexEpsilon) return false;
            }
            else
            {
                if (cross2D(vInJ, vDiagJ) <= kConvexEpsilon || cross2D(vDiagJ, vOutJ) <= kConvexEpsilon) return false;
            }

            return true;
        }

        std::vector<Vec2> convexHull2D(std::span<const Vec2> pts)
        {
            std::vector<Vec2> cleaned;
            for (const Vec2& p : pts)
            {
                bool exists = false;
                for (const Vec2& c : cleaned)
                {
                    if (glm::distance(p, c) < Real(1e-7)) { exists = true; break; }
                }
                if (!exists) cleaned.push_back(p);
            }
            if (cleaned.size() < 3) return cleaned;

            std::sort(cleaned.begin(), cleaned.end(), [](const Vec2& a, const Vec2& b) {
                if (a.x != b.x) return a.x < b.x;
                return a.y < b.y;
                });

            std::vector<Vec2> lower;
            for (const Vec2& p : cleaned)
            {
                while (lower.size() >= 2)
                {
                    Vec2 v1 = lower.back() - lower[lower.size() - 2];
                    Vec2 v2 = p - lower.back();
                    if (cross2D(v1, v2) <= Real(1e-9)) lower.pop_back();
                    else break;
                }
                lower.push_back(p);
            }

            std::vector<Vec2> upper;
            for (auto it = cleaned.rbegin(); it != cleaned.rend(); ++it)
            {
                const Vec2& p = *it;
                while (upper.size() >= 2)
                {
                    Vec2 v1 = upper.back() - upper[upper.size() - 2];
                    Vec2 v2 = p - upper.back();
                    if (cross2D(v1, v2) <= Real(1e-9)) upper.pop_back();
                    else break;
                }
                upper.push_back(p);
            }

            lower.pop_back();
            upper.pop_back();
            lower.insert(lower.end(), upper.begin(), upper.end());

            return lower; // Monotone-chain output is already CCW.
        }

        std::optional<std::vector<Vec2>> tryMergeConvex(std::span<const Vec2> polyA, std::span<const Vec2> polyB)
        {
            Real areaA = std::abs(polygonArea(polyA));
            Real areaB = std::abs(polygonArea(polyB));
            if (areaA < Real(1e-8) || areaB < Real(1e-8)) return std::nullopt;

            std::vector<Vec2> combined;
            combined.reserve(polyA.size() + polyB.size());
            combined.insert(combined.end(), polyA.begin(), polyA.end());
            combined.insert(combined.end(), polyB.begin(), polyB.end());

            std::vector<Vec2> hull = convexHull2D(combined);
            Real areaHull = std::abs(polygonArea(hull));

            if (std::abs(areaHull - (areaA + areaB)) < Real(1e-6))
            {
                return hull;
            }
            return std::nullopt;
        }

        std::vector<std::vector<Vec2>> mergeConvexPolygons(std::vector<std::vector<Vec2>> polygons)
        {
            if (polygons.size() <= 1) return polygons;

            std::vector<std::vector<Vec2>> polys;
            for (auto& p : polygons)
            {
                if (std::abs(polygonArea(p)) > Real(1e-8))
                {
                    polys.push_back(std::move(p));
                }
            }

            bool mergedAny = true;
            while (mergedAny)
            {
                mergedAny = false;
                size_t n = polys.size();
                for (size_t i = 0; i < n && !mergedAny; ++i)
                {
                    for (size_t j = i + 1; j < n && !mergedAny; ++j)
                    {
                        auto merged = tryMergeConvex(polys[i], polys[j]);
                        if (merged.has_value())
                        {
                            polys[i] = std::move(*merged);
                            polys.erase(polys.begin() + j);
                            mergedAny = true;
                        }
                    }
                }
            }
            return polys;
        }

        // ==============================================================================
        // 1. PURE BAYAZIT (No cross-calling)
        // ==============================================================================

        std::optional<std::vector<std::vector<Vec2>>> tryBayazitRec(std::vector<Vec2> polygon)
        {
            const size_t n = polygon.size();
            if (n <= 3 || isConvexPolygon(polygon)) return std::vector<std::vector<Vec2>>{ polygon };

            int reflexIdx = -1;
            for (size_t i = 0; i < n; ++i)
            {
                if (isReflexVertex(polygon[(i + n - 1) % n], polygon[i], polygon[(i + 1) % n]))
                {
                    reflexIdx = static_cast<int>(i);
                    break;
                }
            }

            if (reflexIdx == -1) return std::vector<std::vector<Vec2>>{ polygon };

            int bestJ = -1;
            Real minDist = std::numeric_limits<Real>::max();

            for (size_t j = 0; j < n; ++j)
            {
                int ij = static_cast<int>(j);
                if (ij == reflexIdx || ij == (reflexIdx - 1 + static_cast<int>(n)) % static_cast<int>(n) || ij == (reflexIdx + 1) % static_cast<int>(n))
                {
                    continue;
                }

                if (isValidDiagonal(polygon, reflexIdx, ij))
                {
                    Real dist = glm::distance(polygon[ij], polygon[reflexIdx]);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        bestJ = ij;
                    }
                }
            }

            // Dead-end: No valid diagonal found. Signal failure.
            if (bestJ == -1)
            {
                return std::nullopt;
            }

            int iIdx = std::min(reflexIdx, bestJ);
            int jIdx = std::max(reflexIdx, bestJ);

            std::vector<Vec2> poly1(polygon.begin() + iIdx, polygon.begin() + jIdx + 1);

            std::vector<Vec2> poly2;
            poly2.reserve(polygon.size() - (jIdx - iIdx) + 1);
            for (size_t idx = jIdx; idx < polygon.size(); ++idx) poly2.push_back(polygon[idx]);
            for (int idx = 0; idx <= iIdx; ++idx) poly2.push_back(polygon[idx]);

            auto res1 = tryBayazitRec(std::move(poly1));
            if (!res1.has_value()) return std::nullopt;

            auto res2 = tryBayazitRec(std::move(poly2));
            if (!res2.has_value()) return std::nullopt;

            res1->insert(res1->end(), std::make_move_iterator(res2->begin()), std::make_move_iterator(res2->end()));
            return res1;
        }

        std::optional<std::vector<std::vector<Vec2>>> tryBayazit(std::span<const Vec2> vertices)
        {
            std::vector<Vec2> poly(vertices.begin(), vertices.end());
            auto raw = tryBayazitRec(std::move(poly));
            if (!raw.has_value()) return std::nullopt;
            return mergeConvexPolygons(std::move(*raw));
        }

        // ==============================================================================
        // 2. PURE REFLEX ANGLE BISECTOR (No cross-calling)
        // ==============================================================================

        std::optional<std::vector<std::vector<Vec2>>> tryReflexBisectorRec(std::vector<Vec2> polygon)
        {
            const size_t n = polygon.size();
            if (n <= 3 || isConvexPolygon(polygon)) return std::vector<std::vector<Vec2>>{ polygon };

            int reflexIdx = -1;
            for (size_t i = 0; i < n; ++i)
            {
                if (isReflexVertex(polygon[(i + n - 1) % n], polygon[i], polygon[(i + 1) % n]))
                {
                    reflexIdx = static_cast<int>(i);
                    break;
                }
            }

            if (reflexIdx == -1) return std::vector<std::vector<Vec2>>{ polygon };

            Vec2 curr = polygon[reflexIdx];
            Vec2 prevPt = polygon[(reflexIdx + n - 1) % n];
            Vec2 nextPt = polygon[(reflexIdx + 1) % n];

            Vec2 vPrev = prevPt - curr;
            Real lenPrev = glm::length(vPrev);
            if (lenPrev > Real(1e-12)) vPrev /= lenPrev;

            Vec2 vNext = nextPt - curr;
            Real lenNext = glm::length(vNext);
            if (lenNext > Real(1e-12)) vNext /= lenNext;

            Vec2 bisector = -(vPrev + vNext);
            Real normB = glm::length(bisector);
            if (normB < Real(1e-9))
            {
                bisector = Vec2(-vPrev.y, vPrev.x);
            }
            else
            {
                bisector /= normB;
            }

            Real bestT = std::numeric_limits<Real>::max();
            Vec2 bestHit(0);
            int bestEdgeK = -1;

            for (size_t k = 0; k < n; ++k)
            {
                size_t kNext = (k + 1) % n;
                if (static_cast<int>(k) == reflexIdx || static_cast<int>(kNext) == reflexIdx) continue;

                Vec2 p1 = polygon[k];
                Vec2 p2 = polygon[kNext];
                Vec2 vEdge = p2 - p1;
                Real denom = cross2D(bisector, vEdge);

                if (std::fabs(denom) < Real(1e-9)) continue;

                Real t = cross2D(p1 - curr, vEdge) / denom;
                Real u = cross2D(p1 - curr, bisector) / denom;

                if (t > Real(1e-6) && u >= Real(0) && u <= Real(1))
                {
                    if (t < bestT)
                    {
                        bestT = t;
                        bestHit = curr + bisector * t;
                        bestEdgeK = static_cast<int>(k);
                    }
                }
            }

            // Dead-end: Ray failed to hit a valid edge. Signal failure.
            if (bestEdgeK == -1)
            {
                return std::nullopt;
            }

            Vec2 p1 = polygon[bestEdgeK];
            Vec2 p2 = polygon[(bestEdgeK + 1) % n];

            int targetIdx = -1;
            if (glm::distance(bestHit, p1) < Real(1e-5))
            {
                targetIdx = bestEdgeK;
            }
            else if (glm::distance(bestHit, p2) < Real(1e-5))
            {
                targetIdx = (bestEdgeK + 1) % static_cast<int>(n);
            }
            else
            {
                int splitIdx = bestEdgeK + 1;
                polygon.insert(polygon.begin() + splitIdx, bestHit);
                if (splitIdx <= reflexIdx)
                {
                    reflexIdx++;
                }
                targetIdx = splitIdx;
            }

            // Degenerate hit (reflex vertex or its neighbor): the split would not shrink the polygon.
            const int m = static_cast<int>(polygon.size());
            if (targetIdx == reflexIdx || (targetIdx + 1) % m == reflexIdx || (reflexIdx + 1) % m == targetIdx)
            {
                return std::nullopt;
            }

            int iIdx = std::min(reflexIdx, targetIdx);
            int jIdx = std::max(reflexIdx, targetIdx);

            std::vector<Vec2> poly1(polygon.begin() + iIdx, polygon.begin() + jIdx + 1);

            std::vector<Vec2> poly2;
            poly2.reserve(polygon.size() - (jIdx - iIdx) + 1);
            for (size_t idx = jIdx; idx < polygon.size(); ++idx) poly2.push_back(polygon[idx]);
            for (int idx = 0; idx <= iIdx; ++idx) poly2.push_back(polygon[idx]);

            auto res1 = tryReflexBisectorRec(std::move(poly1));
            if (!res1.has_value()) return std::nullopt;

            auto res2 = tryReflexBisectorRec(std::move(poly2));
            if (!res2.has_value()) return std::nullopt;

            res1->insert(res1->end(), std::make_move_iterator(res2->begin()), std::make_move_iterator(res2->end()));
            return res1;
        }

        std::optional<std::vector<std::vector<Vec2>>> tryReflexBisector(std::span<const Vec2> vertices)
        {
            std::vector<Vec2> poly(vertices.begin(), vertices.end());
            auto raw = tryReflexBisectorRec(std::move(poly));
            if (!raw.has_value()) return std::nullopt;
            return mergeConvexPolygons(std::move(*raw));
        }

        // ==============================================================================
        // 3. EAR CLIPPING
        // ==============================================================================

        bool pointInTriangle(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c)
        {
            const Real d1 = crossZ(a, b, p);
            const Real d2 = crossZ(b, c, p);
            const Real d3 = crossZ(c, a, p);
            const bool hasNeg = (d1 < Real(0)) || (d2 < Real(0)) || (d3 < Real(0));
            const bool hasPos = (d1 > Real(0)) || (d2 > Real(0)) || (d3 > Real(0));
            return !(hasNeg && hasPos);
        }

        bool isConvexIndexed(std::span<const int> poly, std::span<const Vec2> v)
        {
            const size_t n = poly.size();
            if (n < 4) return true;

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

        std::vector<std::vector<int>> triangulateEarClip(std::vector<int> poly, std::span<const Vec2> v)
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

                    if (crossZ(a, b, c) <= kConvexEpsilon) continue;

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

        bool findSharedEdge(std::span<const int> a, std::span<const int> b, size_t& edgeA, size_t& edgeB)
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

        std::vector<int> mergeAcrossEdge(std::span<const int> a, std::span<const int> b, size_t edgeA, size_t edgeB)
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

        void mergeConvexPieces(std::vector<std::vector<int>>& pieces, std::span<const Vec2> v)
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

    bool isConvexPolygon(std::span<const Vec2> vertices)
    {
        const size_t n = vertices.size();
        if (n < 4) return true;

        for (size_t i = 0; i < n; ++i)
        {
            const Vec2& a = vertices[(i + n - 1) % n];
            const Vec2& b = vertices[i];
            const Vec2& c = vertices[(i + 1) % n];
            if (crossZ(a, b, c) < -kConvexEpsilon)
            {
                return false;
            }
        }
        return true;
    }

    bool isSimplePolygon(std::span<const Vec2> v)
    {
        const size_t n = v.size();
        if (n < 3) return false;

        // Spikes (180-degree reversals) overlap their own edge.
        for (size_t i = 0; i < n; ++i)
        {
            const Vec2 e1 = v[i] - v[(i + n - 1) % n];
            const Vec2 e2 = v[(i + 1) % n] - v[i];
            if (glm::dot(e1, e2) < Real(0) &&
                std::abs(cross2D(e1, e2)) <= Real(1e-6) * glm::length(e1) * glm::length(e2))
            {
                return false;
            }
        }

        for (size_t i = 0; i < n; ++i)
        {
            const Vec2& a = v[i];
            const Vec2& b = v[(i + 1) % n];
            for (size_t j = i + 1; j < n; ++j)
            {
                if (j == i + 1 || (i == 0 && j == n - 1)) continue; // Adjacent edges share a vertex.
                const Vec2& c = v[j];
                const Vec2& d = v[(j + 1) % n];
                if (segmentsIntersect(a, b, c, d)) return false;
                if (pointSegDistSq(a, c, d) <= kTouchEpsSq || pointSegDistSq(b, c, d) <= kTouchEpsSq ||
                    pointSegDistSq(c, a, b) <= kTouchEpsSq || pointSegDistSq(d, a, b) <= kTouchEpsSq)
                {
                    return false; // Touching, T-junction, collinear overlap or coincident vertices.
                }
            }
        }
        return true;
    }

    std::vector<std::vector<Vec2>> decomposeConcaveEarClipping(std::span<const Vec2> vertices)
    {
        std::vector<int> indices(vertices.size());
        for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<int>(i);

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

    std::vector<std::vector<Vec2>> decomposeConcaveBayazit(std::span<const Vec2> vertices)
    {
        return decomposeConcave(vertices, ConcaveDecompositionMethod::Bayazit);
    }

    std::vector<std::vector<Vec2>> decomposeConcaveReflexBisector(std::span<const Vec2> vertices)
    {
        return decomposeConcave(vertices, ConcaveDecompositionMethod::ReflexAngleBisector);
    }

    // High-level pipeline dispatcher
    std::vector<std::vector<Vec2>> decomposeConcave(
        std::span<const Vec2> vertices,
        ConcaveDecompositionMethod method
    )
    {
        if (vertices.size() < 3)
        {
            return {};
        }

        if (isConvexPolygon(vertices))
        {
            return { std::vector<Vec2>(vertices.begin(), vertices.end()) };
        }

        switch (method)
        {
        case ConcaveDecompositionMethod::ReflexAngleBisector:
        {
            auto res = tryReflexBisector(vertices);
            if (res.has_value()) return std::move(*res);
            [[fallthrough]];
        }
        case ConcaveDecompositionMethod::Bayazit:
        {
            auto res = tryBayazit(vertices);
            if (res.has_value()) return std::move(*res);
            [[fallthrough]];
        }
        case ConcaveDecompositionMethod::EarClipping:
        default:
            return decomposeConcaveEarClipping(vertices);
        }
    }

    namespace
    {
        void bisectConvexRec(std::span<const Vec2> v, size_t maxCount, std::vector<std::vector<Vec2>>& out)
        {
            const size_t n = v.size();
            if (n <= maxCount)
            {
                out.emplace_back(v.begin(), v.end());
                return;
            }
            const size_t j = n / 2; // Diagonal 0 -> j; both halves have >= 3 vertices for n >= 4.
            bisectConvexRec(v.subspan(0, j + 1), maxCount, out);

            std::vector<Vec2> second(v.begin() + j, v.end());
            second.push_back(v[0]);
            bisectConvexRec(second, maxCount, out);
        }
    }

    std::vector<std::vector<Vec2>> decomposeConvexFan(std::span<const Vec2> vertices, size_t maxVertexCount)
    {
        const size_t n = vertices.size();
        if (n < 3) return {};
        maxVertexCount = std::max<size_t>(maxVertexCount, 3);
        if (n <= maxVertexCount) return { std::vector<Vec2>(vertices.begin(), vertices.end()) };

        std::vector<std::vector<Vec2>> result;
        result.reserve((n - 2 + maxVertexCount - 3) / (maxVertexCount - 2));

        size_t start = 1;
        while (true)
        {
            const size_t end = std::min(start + maxVertexCount - 2, n - 1);
            std::vector<Vec2> piece;
            piece.reserve(end - start + 2);
            piece.push_back(vertices[0]);
            for (size_t k = start; k <= end; ++k) piece.push_back(vertices[k]);
            result.push_back(std::move(piece));
            if (end == n - 1) break;
            start = end; // Shared fan edge.
        }
        return result;
    }

    std::vector<std::vector<Vec2>> decomposeConvexBisect(std::span<const Vec2> vertices, size_t maxVertexCount)
    {
        if (vertices.size() < 3) return {};
        maxVertexCount = std::max<size_t>(maxVertexCount, 3);

        std::vector<std::vector<Vec2>> result;
        bisectConvexRec(vertices, maxVertexCount, result);
        return result;
    }

    std::vector<std::vector<Vec2>> decomposeConvex(
        std::span<const Vec2> vertices,
        size_t maxVertexCount,
        ConvexDecompositionMethod method
    )
    {
        switch (method)
        {
        case ConvexDecompositionMethod::Bisect: return decomposeConvexBisect(vertices, maxVertexCount);
        case ConvexDecompositionMethod::Fan:
        default:                                return decomposeConvexFan(vertices, maxVertexCount);
        }
    }

    void limitConvexPieceVertexCount(
        std::vector<std::vector<Vec2>>& pieces,
        size_t maxVertexCount,
        ConvexDecompositionMethod method
    )
    {
        maxVertexCount = std::max<size_t>(maxVertexCount, 3);
        const bool anyOversized = std::any_of(pieces.begin(), pieces.end(),
            [maxVertexCount](const std::vector<Vec2>& p) { return p.size() > maxVertexCount; });
        if (!anyOversized) return;

        std::vector<std::vector<Vec2>> out;
        out.reserve(pieces.size() + 4);
        for (std::vector<Vec2>& piece : pieces)
        {
            if (piece.size() <= maxVertexCount)
            {
                out.push_back(std::move(piece));
                continue;
            }
            for (std::vector<Vec2>& sub : decomposeConvex(piece, maxVertexCount, method))
                out.push_back(std::move(sub));
        }
        pieces = std::move(out);
    }
}