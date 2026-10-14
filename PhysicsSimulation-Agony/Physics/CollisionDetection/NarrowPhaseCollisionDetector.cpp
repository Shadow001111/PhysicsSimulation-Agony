#include "NarrowPhaseCollisionDetector.h"

#include "Physics/Threading.h"
#include "Physics/Limits.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <bit>
#include <tuple>
#include <algorithm>
#include <iostream>

/*
 * NARROW PHASE: DESIGN NOTES
 *
 * Pipeline
 *   processPairs partitions pairs by (typeA <= typeB) into per-type vectors (same type: higher collider index
 *   first), then dispatches one collision method per non-empty group via 'collisionFuncs'. Every method
 *   therefore sees one shape-type combination: no per-pair type branching. Methods only append to
 *   'outCollisionData'; warm-start data is attached afterwards by contact id.
 *
 * Conventions (all methods)
 *   - Normal points from A to B. Depth > 0 means penetration. Contact points lie on the surface of the shape
 *     that is the "reference" for the contact (circle/capsule surface, or the clipped incident edge).
 *   - Max 2 contacts. Ids must be stable between frames (warm starting):
 *       polygon/box face = REFERENCE_BODY_TAG | (faceIndex << 1) | clippedEndpoint, vertex contact = vertex tag | index,
 *       circle-box = BOX_FACE_* region flags, capsule-capsule = overlap end (0/1) or 2 for the closest point.
 *   - Rotation is stored as (cos, sin): right = (cos, sin), up = (-sin, cos). No trig at runtime.
 *   - Epsilons: ZERO_DIVISION_BOUNDARY(_SQUARED) guards divisions and degenerate features, SAT_EPSILON biases
 *     towards faces so the feature (and its id) does not flicker between frames.
 *   - Compiled with /fp:fast: NaN is not handled. Use std::fmin/std::fmax for clamps (not std::min/max or
 *     ternaries). Never rely on NaN comparisons.
 *   - Polygons are validated at build time (>= 3 vertices, no degenerate edges, unit normals cached in
 *     PolygonGeometry). Narrow phase does not re-check; do not add per-pair guards.
 *
 * Method style
 *   - One flat loop per method. Raw ECSTASY_RESTRICT pointers into the SoA arrays, hoisted out of the loop.
 *     No virtual calls, no helper calls in hot paths: small helpers are __forceinline, one-off logic is a lambda
 *     or a single template function. Early-out as soon as one separating test fails (cheapest tests first).
 *   - Work in the local space of the cheaper side: rotate the small shape (box, circle, capsule core: O(1)) into
 *     the polygon's frame instead of transforming all polygon vertices. Output is rotated back to world space.
 *   - Use reciprocal multiply (v * (1 / len)) instead of vector / scalar. Use squared distances until a root is needed.
 *   - Do not add more per-shape cached data (memory is already heavy with cached normals).
 *
 * How to add or change a method
 *   1. Keep the conventions above (normal A->B, stable ids, max 2 contacts).
 *   2. Pick the frame that avoids O(N) transforms. Prefer analytic projections for boxes and capsules.
 *   3. Reject early with bounding circles or the first separating axis. Avoid sqrt and division before the reject.
 *   4. Pure-math methods with a rare branch: use the SIMD batch pattern of circle-circle. Branch-heavy ones
 *      (clipping, feature selection) stay scalar.
 *   5. Register in 'collisionFuncs'. Argument order is (lower BodyType, higher BodyType).
 */

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    namespace
    {
        // Numeric tolerances.
        constexpr Real ZERO_DIVISION_BOUNDARY = 1e-4;
        constexpr Real ZERO_DIVISION_BOUNDARY_SQUARED = 1e-8; // ZERO_DIVISION_BOUNDARY* ZERO_DIVISION_BOUNDARY;
        constexpr Real SAT_EPSILON = 1e-4;
        constexpr Real HALF = Real(0.5);

        // Fallback normal for coincident circle centers.
        constexpr Real DEFAULT_NORMAL_X = 1;
        constexpr Real DEFAULT_NORMAL_Y = 0;

        // Threading.
        constexpr size_t MULTITHREAD_PAIR_THRESHOLD = 6000;
        constexpr size_t PAIRS_PER_WORKER = 3000;
        constexpr size_t LOAD_BALANCING_FACTOR = 4;

        // Memory layout.
        constexpr size_t CACHE_LINE_ALIGNMENT = std::hardware_destructive_interference_size;

        // Manifolds and contact ids.
        constexpr size_t MAX_CONTACTS_PER_MANIFOLD = 2;
        constexpr uint32_t REFERENCE_BODY_TAG = 0x80000000u;
        constexpr uint32_t INVALID_CONTACT_ID = std::numeric_limits<uint32_t>::max();
        constexpr ColliderIndex INVALID_COLLIDER_INDEX = std::numeric_limits<ColliderIndex>::max();

        // Circle-box contact ids: circle center outside the box (region flags).
        constexpr uint32_t BOX_FACE_RIGHT = 1;
        constexpr uint32_t BOX_FACE_LEFT = 2;
        constexpr uint32_t BOX_FACE_TOP = 4;
        constexpr uint32_t BOX_FACE_BOTTOM = 8;

        // Box incident edge ids (box vertex ids), indexed by [useX][sign > 0].
        constexpr uint32_t BOX_INCIDENT_EDGE_IDS[2][2][2] = {
            { { 1, 2 }, { 0, 3 } },
            { { 3, 2 }, { 0, 1 } }
        };

        // Capsules.
        constexpr uint32_t CAPSULE_VERTEX_CONTACT_TAG = 0x40000000u; // Polygon vertex vs capsule core contact ids.
        constexpr Real CAPSULE_PARALLEL_SIN_SQUARED = Real(1e-3);    // Below this, capsule axes count as parallel (2 contacts).

        // Box as a convex polygon: counter-clockwise, normal i belongs to edge i -> i + 1.
        const Vec2 BOX_NORMALS[4] = {
            Vec2(Real(0), Real(-1)),
            Vec2(Real(1), Real(0)),
            Vec2(Real(0), Real(1)),
            Vec2(Real(-1), Real(0))
        };

        // Result of a capsule vs convex hull test, in world space.
        struct CapsuleManifold
        {
            Vec2 normal;   // From the hull towards the capsule.
            Real depth;
            uint32_t contactCount;
            Vec2 points[MAX_CONTACTS_PER_MANIFOLD]; // On the capsule surface.
            uint32_t ids[MAX_CONTACTS_PER_MANIFOLD];
        };
    }


    template <typename Map>
    size_t robinHoodMapMemoryUsage(const Map& map)
    {
        // mask() + 1 == current bucket count (always a power of two).
        auto const numElementsWithBuffer = map.calcNumElementsWithBuffer(map.mask() + 1);
        return map.calcNumBytesTotal(numElementsWithBuffer);
    }


    const SymmetricMatrix<NarrowPhaseCollisionDetector::CollisionFunc, NarrowPhaseCollisionDetector::BODY_TYPE_COUNT>
        NarrowPhaseCollisionDetector::collisionFuncs = [] {
        SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> mat;

        mat((size_t)BodyType::Circle, (size_t)BodyType::Circle) = &NarrowPhaseCollisionDetector::collisionCircleCircle;
        mat((size_t)BodyType::Circle, (size_t)BodyType::Box) = &NarrowPhaseCollisionDetector::collisionCircleBox;
        mat((size_t)BodyType::Circle, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionCirclePolygon;
        mat((size_t)BodyType::Circle, (size_t)BodyType::Capsule) = &NarrowPhaseCollisionDetector::collisionCircleCapsule;

        mat((size_t)BodyType::Box, (size_t)BodyType::Box) = &NarrowPhaseCollisionDetector::collisionBoxBox;
        mat((size_t)BodyType::Box, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionBoxPolygon;
        mat((size_t)BodyType::Box, (size_t)BodyType::Capsule) = &NarrowPhaseCollisionDetector::collisionBoxCapsule;

        mat((size_t)BodyType::Polygon, (size_t)BodyType::Polygon) = &NarrowPhaseCollisionDetector::collisionPolygonPolygon;
        mat((size_t)BodyType::Polygon, (size_t)BodyType::Capsule) = &NarrowPhaseCollisionDetector::collisionPolygonCapsule;

        mat((size_t)BodyType::Capsule, (size_t)BodyType::Capsule) = &NarrowPhaseCollisionDetector::collisionCapsuleCapsule;
        return mat;
        }();

    /* HELPER METHODS */

    // Next index with wrap-around; replaces a runtime integer modulo.
    template <typename Index>
    [[nodiscard]] static __forceinline Index nextIndex(Index index, Index count)
    {
        return index + 1 == count ? Index(0) : Index(index + 1);
    }

    static __forceinline void flipSignIfNegative(Vec2& v, const Real& sign)
    {
        using Int = std::conditional_t<sizeof(Real) == 8, uint64_t, uint32_t>;
        constexpr Int signBit = Int(1) << (sizeof(Real) * 8 - 1);

        const Int signMask = std::bit_cast<Int>(sign) & signBit;
        v.x = std::bit_cast<Real>(std::bit_cast<Int>(v.x) ^ signMask);
        v.y = std::bit_cast<Real>(std::bit_cast<Int>(v.y) ^ signMask);
    }

    [[nodiscard]] static bool clipSegment(Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal)
    {
        const Real d1 = glm::dot(p1 - planePoint, planeNormal);
        const Real d2 = glm::dot(p2 - planePoint, planeNormal);

        if (d1 >= 0 && d2 >= 0) return false; // Both inside.
        if (d1 < 0 && d2 < 0) return true;  // Both outside.

        const Real t = d1 / (d1 - d2);
        const Vec2 dir = p2 - p1;
        const Vec2 intersect = p1 + dir * t;

        if (d1 >= 0) p2 = intersect;
        else         p1 = intersect;

        return false;
    }

    static void projectVerticesOnAxis(
        const Vec2* ECSTASY_RESTRICT verticesPtr,
        size_t vertexCount,
        const Vec2& axis,
        Real& ECSTASY_RESTRICT minOut,
        Real& ECSTASY_RESTRICT maxOut
    )
    {
        minOut = std::numeric_limits<Real>::max();
        maxOut = -std::numeric_limits<Real>::max();

        for (size_t i = 0; i < vertexCount; i++)
        {
            const Real proj = glm::dot(verticesPtr[i], axis);
            minOut = std::fmin(minOut, proj);
            maxOut = std::fmax(maxOut, proj);
        }
    }

    [[nodiscard]] static Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b)
    {
        const Vec2 ab = b - a;
        const Real denom = glm::dot(ab, ab);
        if (denom < ZERO_DIVISION_BOUNDARY_SQUARED) return a;
        const Real t = glm::dot(p - a, ab) / denom;
        return a + ab * std::fmin(std::fmax(t, Real(0)), Real(1));
    }

    // Finds the polygon edge with the greatest signed separation from 'point' (polygon-local space).
    // Returns false as soon as any edge separates the point by more than 'maxDistance': no contact possible.
    [[nodiscard]] static bool findSupportEdge(
        const Vec2* vertsPtr,
        const Vec2* normalsPtr,
        size_t vertexCount,
        const Vec2& point,
        Real maxDistance,
        uint32_t& supportEdgeOut,
        Real& maxSeparationOut
    )
    {
        Real maxSeparation = -std::numeric_limits<Real>::max();
        uint32_t supportEdge = 0;
        for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
        {
            const Real separation = glm::dot(normalsPtr[i], point - vertsPtr[i]);
            if (separation > maxDistance) return false;
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
                supportEdge = i;
            }
        }
        supportEdgeOut = supportEdge;
        maxSeparationOut = maxSeparation;
        return true;
    }

    [[nodiscard]] static __forceinline Real clampValue(Real value, Real lo, Real hi)
    {
        return std::fmin(std::fmax(value, lo), hi);
    }

    // Capsule (B) vs convex hull (A): a box or a polygon. Runs in the hull-local space and returns the
    // manifold in world space: the normal points from the hull towards the capsule, the points lie on
    // the capsule surface.
    // Only the 2 core endpoints are projected (O(n)) and the capsule side axis is folded into the same
    // pass. Disjoint cores resolve the exact closest feature, so rounded corners are exact.
    // 2 contacts are produced whenever the capsule lies along a hull face.
    // IsBox: analytic separation from 'boxHalfExtents'. Otherwise the polygon arrays are used; the polygon
    // must be valid (>= 3 vertices, no degenerate edges), this is not checked here.
    template <bool IsBox>
    [[nodiscard]] static __forceinline bool collideCapsuleConvex(
        [[maybe_unused]] const Vec2* ECSTASY_RESTRICT vertsPtr,
        [[maybe_unused]] const Vec2* ECSTASY_RESTRICT normalsPtr,
        [[maybe_unused]] uint32_t vertexCount,
        [[maybe_unused]] const Vec2& boxHalfExtents,
        const Vec2& positionA,
        const Vec2& rightA, // Hull rotation (cos, sin).
        const Vec2& positionB,
        const Vec2& axisB,  // Capsule rotation (cos, sin).
        Real halfLengthB,
        Real radius,
        CapsuleManifold& out
    )
    {
        const Vec2 upA = { -rightA.y, rightA.x };
        const Vec2 centerDelta = positionB - positionA;

        if constexpr (IsBox)
        {
            // Bounding circle rejection (skips the AABB corner cases from the broad phase).
            const Real reach = std::sqrt(glm::dot(boxHalfExtents, boxHalfExtents)) + halfLengthB + radius;
            if (glm::dot(centerDelta, centerDelta) >= reach * reach) return false;
        }

        // Capsule pose in hull-local space.
        const Vec2 centerLocal = { glm::dot(centerDelta, rightA), glm::dot(centerDelta, upA) };
        const Vec2 axis = { rightA.x * axisB.x + rightA.y * axisB.y, rightA.x * axisB.y - rightA.y * axisB.x };
        const Vec2 halfCore = axis * halfLengthB;
        const Vec2 s0 = centerLocal - halfCore;
        const Vec2 s1 = centerLocal + halfCore;
        const Real length = halfLengthB * Real(2);

        const Vec2 sideAxis = { -axis.y, axis.x };
        const Real sideOffset = glm::dot(sideAxis, centerLocal);

        const auto vertexAt = [&](uint32_t i) -> Vec2
            {
                if constexpr (IsBox) return Vec2(((i + 1) & 2u) ? boxHalfExtents.x : -boxHalfExtents.x, (i & 2u) ? boxHalfExtents.y : -boxHalfExtents.y);
                else return vertsPtr[i];
            };
        const auto normalAt = [&](uint32_t i) -> Vec2
            {
                if constexpr (IsBox) return BOX_NORMALS[i];
                else return normalsPtr[i];
            };

        // Best hull face and hull extent along the capsule side axis.
        Real faceSeparation;
        uint32_t faceIndex = 0;
        Real sideSeparation;
        Vec2 sideNormal;
        uint32_t sideVertex;

        if constexpr (IsBox)
        {
            const Real minX = std::fmin(s0.x, s1.x);
            const Real maxX = std::fmax(s0.x, s1.x);
            const Real minY = std::fmin(s0.y, s1.y);
            const Real maxY = std::fmax(s0.y, s1.y);

            // Faces in id order: bottom, right, top, left.
            faceSeparation = -maxY - boxHalfExtents.y;
            const Real sepRight = minX - boxHalfExtents.x;
            const Real sepTop = minY - boxHalfExtents.y;
            const Real sepLeft = -maxX - boxHalfExtents.x;
            if (sepRight > faceSeparation) { faceSeparation = sepRight; faceIndex = 1; }
            if (sepTop > faceSeparation) { faceSeparation = sepTop; faceIndex = 2; }
            if (sepLeft > faceSeparation) { faceSeparation = sepLeft; faceIndex = 3; }
            if (faceSeparation >= radius) return false;

            const Real sideExtent = boxHalfExtents.x * std::fabs(sideAxis.x) + boxHalfExtents.y * std::fabs(sideAxis.y);
            sideSeparation = std::fabs(sideOffset) - sideExtent;
            if (sideSeparation >= radius) return false;

            // Box corner that pokes furthest towards the capsule side.
            sideNormal = sideOffset >= Real(0) ? sideAxis : -sideAxis;
            const uint32_t signX = sideNormal.x >= Real(0) ? 1u : 0u;
            const uint32_t signY = sideNormal.y >= Real(0) ? 1u : 0u;
            sideVertex = (signY << 1) | (signX ^ signY);
        }
        else
        {
            faceSeparation = -std::numeric_limits<Real>::max();
            Real minSide = std::numeric_limits<Real>::max();
            Real maxSide = -std::numeric_limits<Real>::max();
            uint32_t minSideVertex = 0;
            uint32_t maxSideVertex = 0;

            for (uint32_t i = 0; i < vertexCount; i++)
            {
                const Vec2 v = vertsPtr[i];
                const Vec2 n = normalsPtr[i];

                const Real separation = std::fmin(glm::dot(n, s0 - v), glm::dot(n, s1 - v));
                if (separation >= radius) return false;
                if (separation > faceSeparation)
                {
                    faceSeparation = separation;
                    faceIndex = i;
                }

                const Real side = glm::dot(v, sideAxis);
                if (side < minSide) { minSide = side; minSideVertex = i; }
                if (side > maxSide) { maxSide = side; maxSideVertex = i; }
            }

            const Real sidePlus = sideOffset - maxSide;   // Capsule on the +side of the polygon.
            const Real sideMinus = minSide - sideOffset;  // Capsule on the -side of the polygon.
            sideSeparation = std::fmax(sidePlus, sideMinus);
            if (sideSeparation >= radius) return false;

            const bool onPlusSide = sidePlus >= sideMinus;
            sideNormal = onPlusSide ? sideAxis : -sideAxis;
            sideVertex = onPlusSide ? maxSideVertex : minSideVertex;
        }

        out.points[1] = Vec2();
        out.ids[1] = 0;

        // Squared distance from 'w' to the capsule core.
        const auto coreDistSquared = [&](const Vec2& w) -> Real
            {
                const Vec2 d = s0 + axis * clampValue(glm::dot(w - s0, axis), Real(0), length) - w;
                return glm::dot(d, d);
            };

        // A face contact fills 'out' in the do-block. Every vertex outcome only records
        // (vertexId, vertexNormal, vertexSeparation) and leaves the block: one shared vertex tail below.
        bool faceContact = false;
        uint32_t vertexId = 0;
        Vec2 vertexNormal;
        Real vertexSeparation;

        do
        {
            // Capsule side axis separates best (with a bias towards faces for stable ids): a vertex pokes the capsule side.
            if (sideSeparation > faceSeparation + SAT_EPSILON)
            {
                vertexId = sideVertex;
                vertexNormal = sideNormal;
                vertexSeparation = sideSeparation;
                break;
            }

            // Face contact: the hull face is the reference, the capsule core the incident segment.
            const uint32_t i0 = faceIndex;
            const uint32_t i1 = IsBox ? ((i0 + 1u) & 3u) : nextIndex(i0, vertexCount);
            const Vec2 a = vertexAt(i0);
            const Vec2 b = vertexAt(i1);
            const Vec2 n = normalAt(i0);
            const Vec2 tangent = { -n.y, n.x };

            vertexNormal = n;
            vertexSeparation = faceSeparation;

            Vec2 c0 = s0;
            Vec2 c1 = s1;
            Real e0 = glm::dot(s0 - a, n);
            Real e1 = glm::dot(s1 - a, n);

            if (faceSeparation > Real(0))
            {
                // Disjoint cores: only the part of the segment above the edge sees the face;
                // an overhanging part may be closer to an end vertex of the edge.
                const Real edgeLength = glm::dot(b - a, tangent);
                const Real t0 = glm::dot(s0 - a, tangent);
                const Real t1 = glm::dot(s1 - a, tangent);

                const bool beyondStart = t0 < Real(0) || t1 < Real(0);
                const bool beyondEnd = t0 > edgeLength || t1 > edgeLength;
                if (beyondStart || beyondEnd)
                {
                    if (clipSegment(c0, c1, a, tangent)) { vertexId = i0; break; }
                    if (clipSegment(c0, c1, b, -tangent)) { vertexId = i1; break; }

                    e0 = glm::dot(c0 - a, n);
                    e1 = glm::dot(c1 - a, n);

                    const Real minE = std::fmin(e0, e1);
                    const Real faceDistSquared = minE * minE;

                    if (beyondStart && coreDistSquared(a) + SAT_EPSILON < faceDistSquared) { vertexId = i0; break; }
                    if (beyondEnd && coreDistSquared(b) + SAT_EPSILON < faceDistSquared) { vertexId = i1; break; }
                }
            }

            const Real minSeparation = faceSeparation > Real(0) ? std::fmin(e0, e1) : faceSeparation;
            const Vec2 span = c1 - c0;
            const bool hasSecond = glm::dot(span, span) >= ZERO_DIVISION_BOUNDARY_SQUARED;
            const Real contactLimit = radius + SAT_EPSILON;
            const uint32_t idBase = REFERENCE_BODY_TAG | (i0 << 1);

            uint32_t contactCount = 0;
            if (e0 <= contactLimit)
            {
                out.points[contactCount] = c0 - n * radius;
                out.ids[contactCount] = idBase;
                contactCount++;
            }
            if (hasSecond && e1 <= contactLimit)
            {
                out.points[contactCount] = c1 - n * radius;
                out.ids[contactCount] = idBase | 1u;
                contactCount++;
            }
            if (contactCount == 0) [[unlikely]] return false;

            out.normal = n;
            out.depth = radius - minSeparation;
            out.contactCount = contactCount;
            faceContact = true;
        } while (false);

        if (!faceContact)
        {
            // Single contact between a hull vertex and the capsule core.
            // 'vertexSeparation' > 0 means the cores are disjoint: the exact vertex distance decides.
            const Vec2 w = vertexAt(vertexId);
            const Vec2 q = s0 + axis * clampValue(glm::dot(w - s0, axis), Real(0), length);

            Vec2 normal = vertexNormal;
            Real depth = radius - vertexSeparation;
            if (vertexSeparation > Real(0))
            {
                const Vec2 d = q - w;
                const Real distSquared = glm::dot(d, d);
                if (distSquared >= radius * radius) return false;
                if (distSquared >= ZERO_DIVISION_BOUNDARY_SQUARED)
                {
                    const Real distance = std::sqrt(distSquared);
                    normal = d * (Real(1) / distance);
                    depth = radius - distance;
                }
            }

            out.normal = normal;
            out.depth = depth;
            out.contactCount = 1;
            out.points[0] = q - normal * radius;
            out.ids[0] = CAPSULE_VERTEX_CONTACT_TAG | vertexId;
        }

        // Hull-local -> world.
        out.normal = rightA * out.normal.x + upA * out.normal.y;
        out.points[0] = positionA + rightA * out.points[0].x + upA * out.points[0].y;
        out.points[1] = positionA + rightA * out.points[1].x + upA * out.points[1].y;
        return true;
    }

    /*               */
    NarrowPhaseCollisionDetector::NarrowPhaseCollisionDetector()
    {
    }

    void NarrowPhaseCollisionDetector::setDataViewers(
        const BodySoAViewer& bodies,
        const ColliderSoAViewer& colliders,
        const CircleSoAViewer& circles,
        const BoxSoAViewer& boxes,
        const PolygonSoAViewer& polygons,
        const CapsuleSoAViewer& capsules
    )
    {
        this->bodies = bodies;
        this->colliders = colliders;
        this->circles = circles;
        this->boxes = boxes;
        this->polygons = polygons;
        this->capsules = capsules;
    }

    const std::vector<BodyCollisionData>& NarrowPhaseCollisionDetector::findCollisions(
        const std::vector<ObjectPair>& colliderPairs,
        ExecutionPolicy executionPolicy
    )
    {
        TRACY_SCOPE_N("Narrow phase");

        allCollisionData.clear();

        if (colliderPairs.empty())
        {
            return allCollisionData;
        }

        allCollisionData.reserve(colliderPairs.size());

        // Determine to use threading or not.
        bool useThreading = false;
        if (executionPolicy == ExecutionPolicy::ForceMultiThreaded)
        {
            useThreading = true;
        }
        else if (executionPolicy == ExecutionPolicy::ForceSingleThreaded)
        {
            useThreading = false;
        }
        else
        {
            useThreading = colliderPairs.size() > MULTITHREAD_PAIR_THRESHOLD;
        }

        // Find collisions.
        if (useThreading)
        {
            findCollisionsMultiThreaded(colliderPairs, colliderPairs.size() / PAIRS_PER_WORKER);
        }
        else
        {
            findCollisionsSingleThreaded(colliderPairs);
        }

        return allCollisionData;
    }

    void NarrowPhaseCollisionDetector::findCollisionsInCircle(
        const std::vector<ObjectIndex>& collidersToCheck,
        Vec2 pos, Real radius,
        std::vector<std::pair<ObjectIndex, Real>>& outColliding
    ) const
    {
        for (ObjectIndex colliderIndex : collidersToCheck)
        {
            const BodyType type = colliders.shapeType[colliderIndex];
            const ObjectIndex shapeIdx = colliders.shapeIndex[colliderIndex];
            const Vec2 colliderPos = { colliders.worldPosX[colliderIndex], colliders.worldPosY[colliderIndex] };

            Real sdf = std::numeric_limits<Real>::max();

            if (type == BodyType::Circle)
            {
                const Real otherRadius = circles.radius[shapeIdx];
                const Vec2 delta = colliderPos - pos;
                const Real distSquared = glm::dot(delta, delta);

                // Colliding iff dist < otherRadius + radius: decide on squared values, sqrt only on a hit.
                const Real radiusSum = otherRadius + radius;
                if (radiusSum <= Real(0) || distSquared >= radiusSum * radiusSum) continue;

                sdf = std::sqrt(distSquared) - otherRadius;
            }
            else if (type == BodyType::Box)
            {
                const Real cosB = colliders.worldRotationCos[colliderIndex];
                const Real sinB = colliders.worldRotationSin[colliderIndex];
                const Real halfWidthB = boxes.halfWidth[shapeIdx];
                const Real halfHeightB = boxes.halfHeight[shapeIdx];

                const Vec2 right = { cosB, sinB };
                const Vec2 up = { -sinB, cosB };

                const Vec2 d = pos - colliderPos;
                const Vec2 circleLocalPosition = {
                    glm::dot(d, right),
                    glm::dot(d, up)
                };

                // Exact 2D Box Signed Distance Field (SDF) representation
                const Real dx = std::fabs(circleLocalPosition.x) - halfWidthB;
                const Real dy = std::fabs(circleLocalPosition.y) - halfHeightB;

                // Outside along one axis by at least 'radius': the distance cannot be smaller. Skips the sqrt.
                const Real maxD = std::fmax(dx, dy);
                if (maxD > Real(0) && maxD >= radius) continue;

                const Real extX = std::fmax(Real(0), dx);
                const Real extY = std::fmax(Real(0), dy);
                const Real distOutside = std::sqrt(extX * extX + extY * extY);
                const Real distInside = std::fmin(Real(0), maxD);

                sdf = distInside + distOutside;
            }
            else if (type == BodyType::Polygon)
            {
                const Real cosB = colliders.worldRotationCos[colliderIndex];
                const Real sinB = colliders.worldRotationSin[colliderIndex];
                const PolygonGeometry& localPolygonVertices = polygons.geometry[shapeIdx];
                const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.vertices();
                const Vec2* ECSTASY_RESTRICT localNormals = localPolygonVertices.normals();
                const size_t vertexCount = localPolygonVertices.size();

                const Vec2 rightB = { cosB,  sinB };
                const Vec2 upB = { -sinB, cosB };

                const Vec2 d = pos - colliderPos;
                const Vec2 circleLocal = {
                    glm::dot(d, rightB),
                    glm::dot(d, upB)
                };

                uint32_t supportEdge = 0;
                Real maxSeparation = 0;
                if (!findSupportEdge(localVerts, localNormals, vertexCount, circleLocal, radius, supportEdge, maxSeparation)) continue;

                if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
                {
                    const Vec2 a = localVerts[supportEdge];
                    const Vec2 b = localVerts[nextIndex(supportEdge, uint32_t(vertexCount))];
                    const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                    sdf = glm::length(circleLocal - q);
                }
                else
                {
                    // Point is inside the polygon; maxSeparation represents negative depth
                    sdf = maxSeparation;
                }
            }
            else if (type == BodyType::Capsule)
            {
                const Real halfLengthB = capsules.halfLength[shapeIdx];
                const Real capsuleRadius = capsules.radius[shapeIdx];
                const Vec2 axis = { colliders.worldRotationCos[colliderIndex], colliders.worldRotationSin[colliderIndex] };

                // Closest point of the capsule core to the query center.
                const Vec2 d = pos - colliderPos;
                const Real t = clampValue(glm::dot(d, axis), -halfLengthB, halfLengthB);
                const Vec2 delta = d - axis * t;
                const Real distSquared = glm::dot(delta, delta);

                // Colliding iff dist < capsuleRadius + radius: decide on squared values, sqrt only on a hit.
                const Real radiusSum = capsuleRadius + radius;
                if (radiusSum <= Real(0) || distSquared >= radiusSum * radiusSum) continue;

                sdf = std::sqrt(distSquared) - capsuleRadius;
            }

            // Check if the collider collides with the query circle
            if (sdf < radius)
            {
                outColliding.emplace_back(colliderIndex, sdf);
            }
        }
    }

    void NarrowPhaseCollisionDetector::updatePersistentContactData()
    {
        TRACY_SCOPE_N("Update persistent contact data");

        previousContactDataContainer.clear();
        previousContactDataContainer.reserve(allCollisionData.size());
        for (BodyCollisionData& collData : allCollisionData)
        {
            ColliderPairKey key{ collData.colliderA , collData.colliderB };

            // Default CachedContactPair already has invalid ids; reassign in case of a duplicate key.
            CachedContactPair& data = previousContactDataContainer.try_emplace(key).first->second;
            data = CachedContactPair{};

            // Only cache valid active contacts.
            for (uint32_t i = 0; i < collData.contactCount; i++)
            {
                data.contactIds[i] = collData.contactIds[i];
                data.contactData[i] = collData.persistentContactData[i];
            }
        }
    }

    void NarrowPhaseCollisionDetector::remapPersistentContactData(const std::vector<ObjectDeletion>& deletedColliders)
    {
        if (deletedColliders.empty() || previousContactDataContainer.empty())
        {
            return;
        }

        TRACY_SCOPE_N("Remap persistent contact data");

        // Compose the whole deletion log once, tracking only touched indices.
        // slotToEntity: slot -> ORIGINAL index of the collider currently in that slot,
        // or INVALID_COLLIDER_INDEX if the slot holds a collider created after the cache was built
        // (or was vacated). Slots never touched are identity.
        robin_hood::unordered_flat_map<ColliderIndex, ColliderIndex> slotToEntity;
        robin_hood::unordered_flat_set<ColliderIndex> dead;
        slotToEntity.reserve(deletedColliders.size() * 2);
        dead.reserve(deletedColliders.size());

        auto entityAt = [&](ColliderIndex slot) -> ColliderIndex
            {
                const auto it = slotToEntity.find(slot);
                return it == slotToEntity.end() ? slot : it->second;
            };

        for (const auto& deletion : deletedColliders)
        {
            const ColliderIndex d = deletion.deletedIndex;
            const ColliderIndex s = deletion.swappedFromIndex;

            const ColliderIndex victim = entityAt(d);
            if (victim != INVALID_COLLIDER_INDEX) dead.insert(victim);

            if (s != INVALID_COLLIDER_INDEX && s != d)
            {
                const ColliderIndex moved = entityAt(s);
                slotToEntity[d] = moved;
                slotToEntity[s] = INVALID_COLLIDER_INDEX; // Vacated; a later create may reuse it.
            }
            else
            {
                slotToEntity[d] = INVALID_COLLIDER_INDEX;
            }
        }

        // original index -> final index (INVALID_COLLIDER_INDEX if deleted). Untouched indices are absent.
        robin_hood::unordered_flat_map<ColliderIndex, ColliderIndex> remap;
        remap.reserve(dead.size() + slotToEntity.size());
        for (const ColliderIndex e : dead) remap[e] = INVALID_COLLIDER_INDEX;
        for (const auto& [slot, entity] : slotToEntity)
        {
            if (entity != INVALID_COLLIDER_INDEX && entity != slot) remap[entity] = slot;
        }

        // Update in place: only affected entries are extracted and re-inserted.
        std::vector<std::pair<ColliderPairKey, CachedContactPair>> reinsert;

        for (auto it = previousContactDataContainer.begin(); it != previousContactDataContainer.end();)
        {
            ColliderIndex a = it->first.colliderA;
            ColliderIndex b = it->first.colliderB;

            const auto ra = remap.find(a);
            const auto rb = remap.find(b);
            if (ra == remap.end() && rb == remap.end())
            {
                ++it;
                continue;
            }

            if (ra != remap.end()) a = ra->second;
            if (rb != remap.end()) b = rb->second;

            if (a != INVALID_COLLIDER_INDEX && b != INVALID_COLLIDER_INDEX)
            {
                // Re-sort the indices to preserve narrow-phase invariants.
                const BodyType typeA = colliders.shapeType[a];
                const BodyType typeB = colliders.shapeType[b];

                if (typeA > typeB || (typeA == typeB && b > a))
                {
                    std::swap(a, b);
                }

                reinsert.emplace_back(ColliderPairKey{ a, b }, it->second);
            }

            it = previousContactDataContainer.erase(it);
        }

        for (auto& [key, data] : reinsert)
        {
            previousContactDataContainer.insert_or_assign(key, data);
        }
    }

    void NarrowPhaseCollisionDetector::clearData()
    {
        allCollisionData.clear();
    }

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = sizeof(NarrowPhaseCollisionDetector);
        total += PS_AGONY::getVectorMemoryUsage(allCollisionData);

        total += PS_AGONY::getVectorMemoryUsage(chunks);

        for (const auto& chunkData : chunks)
        {
            for (const auto& vec : chunkData.pairs.getDirectAccess())
            {
                total += PS_AGONY::getVectorMemoryUsage(vec);
            }
            total += PS_AGONY::getVectorMemoryUsage(chunkData.results);
        }

        total += robinHoodMapMemoryUsage(previousContactDataContainer);

        return total;
    }

    void NarrowPhaseCollisionDetector::findCollisionsSingleThreaded(const std::vector<ObjectPair>& colliderPairs)
    {
        TRACY_SCOPE_N("Single-threaded narrow phase");

        if (chunks.empty()) chunks.resize(1);
        ChunkData& cd = chunks[0];

        // Same chunk partitioning as the multi-threaded path (when DETERMINISTIC), but results
        // are appended directly to 'allCollisionData': no intermediate buffer, no merge copy.
        const size_t total = colliderPairs.size();
        const size_t step = DETERMINISTIC ? DETERMINISTIC_CHUNK_SIZE : total;
        for (size_t start = 0; start < total; start += step)
        {
            cd.start = start;
            cd.end = std::min(start + step, total);
            cd.clear();
            processPairs(colliderPairs, cd, allCollisionData);
        }
    }

    void NarrowPhaseCollisionDetector::findCollisionsMultiThreaded(const std::vector<ObjectPair>& colliderPairs, size_t workerCount)
    {
        TRACY_SCOPE_N("Multi-threaded narrow phase");

        auto& threadPool = Threading::getGlobalThreadPool();
        workerCount = std::max<size_t>(std::min<size_t>(workerCount, threadPool.getThreadCount()), 1); // At least the calling thread.

        size_t chunkCount, chunkSize;
        if constexpr (DETERMINISTIC)
        {
            // Fixed chunk size: partitioning (and thus output order) is independent of thread count.
            chunkSize = DETERMINISTIC_CHUNK_SIZE;
            chunkCount = (colliderPairs.size() + chunkSize - 1) / chunkSize;
        }
        else
        {
            std::tie(chunkCount, chunkSize) = Ecstasy::Core::Threading::ParallelForRangeExecutor::getChunkCountAndSize(
                threadPool, colliderPairs.size(), LOAD_BALANCING_FACTOR);
        }

        if (chunks.size() < chunkCount) chunks.resize(chunkCount); // Grow-only: keeps chunk buffers alive.

        for (size_t i = 0; i < chunkCount; i++)
        {
            ChunkData& cd = chunks[i];
            cd.start = i * chunkSize;
            cd.end = std::min(cd.start + chunkSize, colliderPairs.size());
            cd.finished = false;
        }

        std::vector<Ecstasy::Core::Threading::Task> tasks;
        tasks.reserve(workerCount);

        alignas(CACHE_LINE_ALIGNMENT) std::atomic<uint32_t> workerChunkIndex{ 0 };
        alignas(CACHE_LINE_ALIGNMENT) std::atomic<uint32_t> destroyedWorkerCount{ 0 };

        // Only the calling thread merges, strictly in chunk order. It merges whatever is
        // already finished between its own chunks, so most copying overlaps with the workers.
        size_t nextToMerge = 0;
        auto mergeReady = [&](bool blocking)
            {
                while (nextToMerge < chunkCount)
                {
                    const ChunkData& cd = chunks[nextToMerge];
                    if (blocking) cd.finished.wait(false, std::memory_order_acquire);
                    else if (!cd.finished.load(std::memory_order_acquire)) return;

                    allCollisionData.insert(allCollisionData.end(), cd.results.begin(), cd.results.end());
                    nextToMerge++;
                }
            };

        auto workerFunc = [&](bool isMainThread)
            {
                while (true)
                {
                    const uint32_t chunkIndex = workerChunkIndex.fetch_add(1, std::memory_order_relaxed);
                    if (chunkIndex >= chunkCount) break;

                    ChunkData& cd = chunks[chunkIndex];
                    cd.clear();
                    processPairs(colliderPairs, cd, cd.results);

                    cd.finished.store(true, std::memory_order_release);
                    if (isMainThread)
                    {
                        mergeReady(false);
                    }
                    else
                    {
                        cd.finished.notify_one();
                    }
                }
                destroyedWorkerCount.fetch_add(1, std::memory_order_release);
                destroyedWorkerCount.notify_one();
            };

        // One less task submitted to the pool: the calling thread takes that slot itself.
        for (size_t i = 0; i + 1 < workerCount; i++)
        {
            tasks.emplace_back([&workerFunc]() { workerFunc(false); });
        }
        threadPool.enqueueBulk(tasks);

        {
            TRACY_SCOPE_N("Participate in work");
            workerFunc(true);
        }

        {
            TRACY_SCOPE_N("Wait for workers to finish and combine data");
            mergeReady(true);
        }
        {
            TRACY_SCOPE_N("Wait for workers to get destroyed");
            while (true)
            {
                const auto value = destroyedWorkerCount.load(std::memory_order_acquire);
                if (value == workerCount) break;
                destroyedWorkerCount.wait(value, std::memory_order_acquire);
            }
        }
    }

    void NarrowPhaseCollisionDetector::processPairs(const std::vector<ObjectPair>& pairs, ChunkData& chunkData, std::vector<BodyCollisionData>& out)
    {
        TRACY_SCOPE_N("Process pair range");

        const size_t firstResult = out.size();

        const BodyType* ECSTASY_RESTRICT shapeTypePtr = colliders.shapeType;

        // Partition the given range into type-specific vectors.
        {
            TRACY_SCOPE_N("Partition");
            for (size_t i = chunkData.start; i < chunkData.end; i++)
            {
                auto [colliderIndexA, colliderIndexB] = pairs[i];

                BodyType typeA = shapeTypePtr[colliderIndexA];
                BodyType typeB = shapeTypePtr[colliderIndexB];

                if (typeA > typeB)
                {
                    std::swap(colliderIndexA, colliderIndexB);
                    std::swap(typeA, typeB);
                }
                else if (typeA == typeB && colliderIndexB > colliderIndexA)
                {
                    std::swap(colliderIndexA, colliderIndexB);
                }

                chunkData.pairs((size_t)typeA, (size_t)typeB).emplace_back(colliderIndexA, colliderIndexB);
            }
        }

        // Dispatch each non-empty type group.
        const auto& pairsDA = chunkData.pairs.getDirectAccess();
        const auto& collisionFuncsDA = collisionFuncs.getDirectAccess();

        constexpr size_t count = chunkData.pairs.STORED_COUNT;
        for (size_t i = 0; i < count; i++)
        {
            auto& vec = pairsDA[i];
            if (!vec.empty())
            {
                CollisionFunc func = collisionFuncsDA[i];
                if (func) [[likely]]
                {
                    (this->*func)(vec, out);
                }
            }
        }

        // Move persistent contact data from previous frame.
        {
            TRACY_SCOPE_N("Move previous contact data");
            for (size_t r = firstResult; r < out.size(); r++)
            {
                BodyCollisionData& collData = out[r];
                ColliderPairKey key{ collData.colliderA , collData.colliderB };

                // Check if pair existed in previous frame.
                const auto it = previousContactDataContainer.find(key);
                if (it == previousContactDataContainer.end()) continue;

                const CachedContactPair& data = it->second;

                // Match current contacts with previous frame contacts using contactIds
                for (uint32_t i = 0; i < collData.contactCount; i++)
                {
                    if (data.contactIds[0] != INVALID_CONTACT_ID && collData.contactIds[i] == data.contactIds[0])
                    {
                        collData.persistentContactData[i] = data.contactData[0];
                    }
                    else if (data.contactIds[1] != INVALID_CONTACT_ID && collData.contactIds[i] == data.contactIds[1])
                    {
                        collData.persistentContactData[i] = data.contactData[1];
                    }
                }
            }
        }
    }


    /*===COLLISION METHODS===*/

    void NarrowPhaseCollisionDetector::collisionCircleCircle(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-circle collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        const size_t pairCount = pairs.size();
        size_t i = 0;
        if constexpr (true)
        {
            ObjectPair colliderPairsBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionAXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionAYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionBXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionBYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real radiusABatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real radiusBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real normalXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real normalYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real depthBatch[RealSimd::lanes];

            const RealSimd normalBX(DEFAULT_NORMAL_X);
            const RealSimd normalBY(DEFAULT_NORMAL_Y);

            for (; i + RealSimd::lanes <= pairCount; i += RealSimd::lanes)
            {
                for (size_t j = 0; j < RealSimd::lanes; j++)
                {
                    const auto pair = pairs[i + j];
                    colliderPairsBatch[j] = pair;

                    const ObjectIndex indexA = pair.a;
                    const ObjectIndex indexB = pair.b;

                    positionAXBatch[j] = positionXPtr[indexA];
                    positionAYBatch[j] = positionYPtr[indexA];
                    positionBXBatch[j] = positionXPtr[indexB];
                    positionBYBatch[j] = positionYPtr[indexB];

                    const ObjectIndex shapeA = shapeIndexPtr[indexA];
                    const ObjectIndex shapeB = shapeIndexPtr[indexB];

                    radiusABatch[j] = radiusPtr[shapeA];
                    radiusBBatch[j] = radiusPtr[shapeB];
                }

                const RealSimd positionAX = RealSimd::load(positionAXBatch);
                const RealSimd positionAY = RealSimd::load(positionAYBatch);
                const RealSimd positionBX = RealSimd::load(positionBXBatch);
                const RealSimd positionBY = RealSimd::load(positionBYBatch);

                const RealSimd radiusA = RealSimd::load(radiusABatch);
                const RealSimd radiusB = RealSimd::load(radiusBBatch);

                const RealSimd deltaPositionX = positionBX - positionAX;
                const RealSimd deltaPositionY = positionBY - positionAY;

                const RealSimd radiusSum = radiusA + radiusB;
                const RealSimd radiusSumSquared = radiusSum * radiusSum;

                const RealSimd squaredDistance = RealSimd::mulAdd(deltaPositionX, deltaPositionX, deltaPositionY * deltaPositionY);

                const RealSimd isCollidingMask = squaredDistance < radiusSumSquared;

                unsigned int collisionMask = isCollidingMask.movemask();
                if (collisionMask == 0) continue;

                const RealSimd distance = RealSimd::sqrt(squaredDistance);
                const RealSimd depth = radiusSum - distance;

                const RealSimd isCloseToZeroMask = distance < RealSimd(ZERO_DIVISION_BOUNDARY);

                const RealSimd invDistance = RealSimd(1) / distance;

                const RealSimd normalAX = deltaPositionX * invDistance;
                const RealSimd normalAY = deltaPositionY * invDistance;

                const RealSimd normalX = RealSimd::blendv(normalAX, normalBX, isCloseToZeroMask);
                const RealSimd normalY = RealSimd::blendv(normalAY, normalBY, isCloseToZeroMask);

                normalX.store(normalXBatch);
                normalY.store(normalYBatch);
                depth.store(depthBatch);

                {
                    while (collisionMask != 0)
                    {
                        const int lane = std::countr_zero(collisionMask);
                        collisionMask &= collisionMask - 1;

                        const ObjectPair pair = colliderPairsBatch[lane];
                        const Vec2 normal{ normalXBatch[lane], normalYBatch[lane] };
                        const Real depth = depthBatch[lane];
                        const Vec2 positionA{ positionAXBatch[lane], positionAYBatch[lane] };
                        const Real radiusA = radiusABatch[lane];

                        outCollisionData.emplace_back(
                            pair.a, pair.b,
                            colliderOwnerPtr[pair.a], colliderOwnerPtr[pair.b],
                            normal,
                            depth,
                            1,
                            positionA + normal * radiusA,
                            Vec2(),
                            0,
                            0
                        );
                    }
                }
            }
        }
        for (; i < pairCount; i++)
        {
            auto [indexA, indexB] = pairs[i];

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real radiusB = radiusPtr[shapeB];

            const Vec2 deltaPosition = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);

            if (squaredDistance >= radiusSum * radiusSum) continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance < ZERO_DIVISION_BOUNDARY) [[unlikely]]
            {
                normal = { DEFAULT_NORMAL_X, DEFAULT_NORMAL_Y };
            }
            else
            {
                const Real invDistance = Real(1.0) / distance;
                normal = deltaPosition * invDistance;
            }

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                positionA + normal * radiusA,
                Vec2(),
                0,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-box collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Vec2 right = { cosB, sinB };
            const Vec2 up = { -sinB, cosB };

            const Vec2 d = positionA - positionB;
            const Vec2 circleLocalPosition = {
                glm::dot(d, right),
                glm::dot(d, up)
            };

            const Vec2 closestLocal = {
                std::fmin(std::fmax(circleLocalPosition.x, -halfWidthB),  halfWidthB),
                std::fmin(std::fmax(circleLocalPosition.y, -halfHeightB), halfHeightB)
            };

            const Vec2 deltaLocal = closestLocal - circleLocalPosition;
            const Real squaredDistance = glm::dot(deltaLocal, deltaLocal);

            if (squaredDistance >= radiusA * radiusA) continue;

            if (squaredDistance >= ZERO_DIVISION_BOUNDARY_SQUARED)
            {
                const Real distance = std::sqrt(squaredDistance);
                const Real invDistance = Real(1) / distance;

                const Vec2 worldDelta = {
                    cosB * deltaLocal.x - sinB * deltaLocal.y,
                    sinB * deltaLocal.x + cosB * deltaLocal.y
                };
                const Vec2 normal = worldDelta * invDistance;
                const Real depth = radiusA - distance;
                const Vec2 contactOnCircle = positionA + normal * radiusA;

                uint32_t contactId = 0;
                if (circleLocalPosition.x > halfWidthB)  contactId |= BOX_FACE_RIGHT;
                else if (circleLocalPosition.x < -halfWidthB)  contactId |= BOX_FACE_LEFT;
                if (circleLocalPosition.y > halfHeightB) contactId |= BOX_FACE_TOP;
                else if (circleLocalPosition.y < -halfHeightB) contactId |= BOX_FACE_BOTTOM;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normal,
                    depth,
                    1,
                    contactOnCircle,
                    Vec2(),
                    contactId,
                    0
                );
                continue;
            }

            // Circle center inside box: choose nearest face.
            const Real dx = halfWidthB - std::fabs(circleLocalPosition.x);
            const Real dy = halfHeightB - std::fabs(circleLocalPosition.y);
            const bool useX = dx < dy;
            const Real minPen = useX ? dx : dy;

            const Real sx = std::copysign(Real(1), circleLocalPosition.x);
            const Real sy = std::copysign(Real(1), circleLocalPosition.y);

            Vec2 normalLocal;
            uint32_t contactId;
            if (useX)
            {
                normalLocal = Vec2(sx, Real(0));
                contactId = sx > 0 ? BOX_FACE_RIGHT : BOX_FACE_LEFT;
            }
            else
            {
                normalLocal = Vec2(Real(0), sy);
                contactId = sy > 0 ? BOX_FACE_TOP : BOX_FACE_BOTTOM;
            }

            const Vec2 normal = -Vec2{
                cosB * normalLocal.x - sinB * normalLocal.y,
                sinB * normalLocal.x + cosB * normalLocal.y
            };
            const Real depth = radiusA + minPen;
            const Vec2 contactOnCircle = positionA + normal * radiusA;


            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                contactOnCircle,
                Vec2(),
                contactId,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-polygon collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = radiusPtr[shapeA];

            const PolygonGeometry& localPolygonVertices = polyGeometryPtr[shapeB];
            const Vec2* ECSTASY_RESTRICT localVerts = localPolygonVertices.vertices();
            const Vec2* ECSTASY_RESTRICT localNormals = localPolygonVertices.normals();
            const size_t vertexCount = localPolygonVertices.size();

            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            const Vec2 abPosDelta = positionA - positionB;
            const Vec2 circleLocal = {
                glm::dot(abPosDelta, rightB),
                glm::dot(abPosDelta, upB)
            };

            uint32_t supportEdge = 0;
            Real maxSeparation = 0;
            if (!findSupportEdge(localVerts, localNormals, vertexCount, circleLocal, radiusA, supportEdge, maxSeparation)) continue;

            Vec2 normalLocal;
            Real separation;
            if (maxSeparation >= ZERO_DIVISION_BOUNDARY)
            {
                // Center is outside the polygon but within radiusA of it.
                const Vec2 a = localVerts[supportEdge];
                const Vec2 b = localVerts[nextIndex(supportEdge, uint32_t(vertexCount))];
                const Vec2 q = closestPointOnSegment(circleLocal, a, b);
                const Vec2 d = circleLocal - q;
                const Real distSquared = glm::dot(d, d);

                if (distSquared < ZERO_DIVISION_BOUNDARY_SQUARED)
                {
                    normalLocal = localNormals[supportEdge];
                    separation = 0;
                }
                else
                {
                    const Real dist = std::sqrt(distSquared);
                    normalLocal = d * (Real(1) / dist);
                    separation = dist;
                }

                if (separation > radiusA) continue;
            }
            else
            {
                // Circle center is inside the polygon.
                normalLocal = localNormals[supportEdge];
                separation = maxSeparation;
            }

            Vec2 normal = -Vec2{
                rightB.x * normalLocal.x + upB.x * normalLocal.y,
                rightB.y * normalLocal.x + upB.y * normalLocal.y
            };

            const Real depth = radiusA - separation;
            const Vec2 contactOnCircle = positionA + normal * radiusA;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                contactOnCircle,
                Vec2(),
                supportEdge,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCircleCapsule(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Circle-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT circleRadiusPtr = circles.radius;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        const size_t pairCount = pairs.size();
        size_t i = 0;
        {
            ObjectPair colliderPairsBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionAXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionAYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real positionBXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real positionBYBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real cosBBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real sinBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real radiusABatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real halfLengthBBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real radiusBBatch[RealSimd::lanes];

            alignas(RealSimd::bytes) Real normalXBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real normalYBatch[RealSimd::lanes];
            alignas(RealSimd::bytes) Real depthBatch[RealSimd::lanes];

            const RealSimd zero(Real(0));
            const RealSimd one(Real(1));

            for (; i + RealSimd::lanes <= pairCount; i += RealSimd::lanes)
            {
                for (size_t j = 0; j < RealSimd::lanes; j++)
                {
                    const auto pair = pairs[i + j];
                    colliderPairsBatch[j] = pair;

                    const ObjectIndex indexA = pair.a;
                    const ObjectIndex indexB = pair.b;

                    positionAXBatch[j] = positionXPtr[indexA];
                    positionAYBatch[j] = positionYPtr[indexA];
                    positionBXBatch[j] = positionXPtr[indexB];
                    positionBYBatch[j] = positionYPtr[indexB];

                    cosBBatch[j] = rotationCosPtr[indexB];
                    sinBBatch[j] = rotationSinPtr[indexB];

                    const ObjectIndex shapeA = shapeIndexPtr[indexA];
                    const ObjectIndex shapeB = shapeIndexPtr[indexB];

                    radiusABatch[j] = circleRadiusPtr[shapeA];
                    halfLengthBBatch[j] = halfLengthPtr[shapeB];
                    radiusBBatch[j] = capsuleRadiusPtr[shapeB];
                }

                const RealSimd positionAX = RealSimd::load(positionAXBatch);
                const RealSimd positionAY = RealSimd::load(positionAYBatch);
                const RealSimd positionBX = RealSimd::load(positionBXBatch);
                const RealSimd positionBY = RealSimd::load(positionBYBatch);

                const RealSimd cosB = RealSimd::load(cosBBatch);
                const RealSimd sinB = RealSimd::load(sinBBatch);

                const RealSimd radiusA = RealSimd::load(radiusABatch);
                const RealSimd halfLengthB = RealSimd::load(halfLengthBBatch);
                const RealSimd radiusB = RealSimd::load(radiusBBatch);

                const RealSimd dX = positionAX - positionBX;
                const RealSimd dY = positionAY - positionBY;

                // Circle center -> closest point of the capsule core: clamp(t, -halfLength, halfLength).
                const RealSimd negHalfLength = zero - halfLengthB;
                const RealSimd alongAxis = RealSimd::mulAdd(dX, cosB, dY * sinB);
                const RealSimd clampedLow = RealSimd::blendv(alongAxis, negHalfLength, alongAxis < negHalfLength);
                const RealSimd t = RealSimd::blendv(clampedLow, halfLengthB, halfLengthB < clampedLow);

                const RealSimd deltaX = cosB * t - dX;
                const RealSimd deltaY = sinB * t - dY;

                const RealSimd radiusSum = radiusA + radiusB;
                const RealSimd squaredDistance = RealSimd::mulAdd(deltaX, deltaX, deltaY * deltaY);

                unsigned int collisionMask = (squaredDistance < radiusSum * radiusSum).movemask();
                if (collisionMask == 0) continue;

                const RealSimd distance = RealSimd::sqrt(squaredDistance);
                const RealSimd invDistance = one / distance;
                const RealSimd isCloseToZeroMask = distance < RealSimd(ZERO_DIVISION_BOUNDARY);

                // Circle center on the core: push out along the capsule side, away from the circle's side.
                const RealSimd sideOffset = dY * cosB - dX * sinB;
                const RealSimd flipMask = sideOffset < zero;
                const RealSimd fallbackX = RealSimd::blendv(sinB, zero - sinB, flipMask);
                const RealSimd fallbackY = RealSimd::blendv(zero - cosB, cosB, flipMask);

                const RealSimd normalX = RealSimd::blendv(deltaX * invDistance, fallbackX, isCloseToZeroMask);
                const RealSimd normalY = RealSimd::blendv(deltaY * invDistance, fallbackY, isCloseToZeroMask);
                const RealSimd depth = radiusSum - distance;

                normalX.store(normalXBatch);
                normalY.store(normalYBatch);
                depth.store(depthBatch);

                while (collisionMask != 0)
                {
                    const int lane = std::countr_zero(collisionMask);
                    collisionMask &= collisionMask - 1;

                    const ObjectPair pair = colliderPairsBatch[lane];
                    const Vec2 normal{ normalXBatch[lane], normalYBatch[lane] };
                    const Vec2 positionA{ positionAXBatch[lane], positionAYBatch[lane] };

                    outCollisionData.emplace_back(
                        pair.a, pair.b,
                        colliderOwnerPtr[pair.a], colliderOwnerPtr[pair.b],
                        normal,
                        depthBatch[lane],
                        1,
                        positionA + normal * radiusABatch[lane],
                        Vec2(),
                        0,
                        0
                    );
                }
            }
        }
        for (; i < pairCount; i++)
        {
            auto [indexA, indexB] = pairs[i];

            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real radiusA = circleRadiusPtr[shapeA];
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radiusB = capsuleRadiusPtr[shapeB];

            const Vec2 axisB = { cosB, sinB };
            const Vec2 d = positionA - positionB;

            // Circle center -> closest point of the capsule core.
            const Real t = clampValue(glm::dot(d, axisB), -halfLengthB, halfLengthB);
            const Vec2 delta = axisB * t - d;

            const Real radiusSum = radiusA + radiusB;
            const Real squaredDistance = glm::dot(delta, delta);
            if (squaredDistance >= radiusSum * radiusSum) continue;

            const Real distance = std::sqrt(squaredDistance);
            const Real depth = radiusSum - distance;

            Vec2 normal;
            if (distance < ZERO_DIVISION_BOUNDARY) [[unlikely]]
            {
                // Circle center on the core: push out along the capsule side, away from the circle's side.
                normal = { sinB, -cosB };
                flipSignIfNegative(normal, glm::dot(d, Vec2(-sinB, cosB)));
            }
            else
            {
                normal = delta * (Real(1) / distance);
            }

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                1,
                positionA + normal * radiusA,
                Vec2(),
                0,
                0
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-box collision");

        enum class SATAxis : uint32_t
        {
            A_RIGHT,
            A_UP,
            B_RIGHT,
            B_UP
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Vec2 rightA = { cosA,  sinA };
            const Vec2 upA = { -sinA, cosA };
            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            const Real absRelativeCos = std::fabs(cosA * cosB + sinA * sinB);
            const Real absRelativeSin = std::fabs(cosA * sinB - sinA * cosB);

            const Vec2 centerDelta = positionB - positionA;
            const Real centerDeltaOnRightA = glm::dot(centerDelta, rightA);
            const Real centerDeltaOnUpA = glm::dot(centerDelta, upA);
            const Real centerDeltaOnRightB = glm::dot(centerDelta, rightB);
            const Real centerDeltaOnUpB = glm::dot(centerDelta, upB);

            const Real projectedRadiusBOnRightA = halfWidthB * absRelativeCos + halfHeightB * absRelativeSin;
            const Real projectedRadiusBOnUpA = halfWidthB * absRelativeSin + halfHeightB * absRelativeCos;
            const Real projectedRadiusAOnRightB = halfWidthA * absRelativeCos + halfHeightA * absRelativeSin;
            const Real projectedRadiusAOnUpB = halfWidthA * absRelativeSin + halfHeightA * absRelativeCos;

            Vec2 normal{};
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxis = SATAxis::A_RIGHT;

            auto sat = [&](Real radiusSum, Real centerDeltaOnAxis, Vec2 axis, SATAxis axisType) -> bool
                {
                    const Real overlap = radiusSum - std::fabs(centerDeltaOnAxis);
                    if (overlap < Real(0)) return false;
                    if (overlap < depth)
                    {
                        depth = overlap;
                        normal = axis;
                        flipSignIfNegative(normal, centerDeltaOnAxis);
                        bestAxis = axisType;
                    }
                    return true;
                };

            if (!sat(projectedRadiusBOnRightA + halfWidthA, centerDeltaOnRightA, rightA, SATAxis::A_RIGHT)) continue;
            if (!sat(projectedRadiusBOnUpA + halfHeightA, centerDeltaOnUpA, upA, SATAxis::A_UP))    continue;
            if (!sat(projectedRadiusAOnRightB + halfWidthB, centerDeltaOnRightB, rightB, SATAxis::B_RIGHT)) continue;
            if (!sat(projectedRadiusAOnUpB + halfHeightB, centerDeltaOnUpB, upB, SATAxis::B_UP))    continue;

            const bool refIsA = bestAxis < SATAxis::B_RIGHT;

            Vec2 positionRef, positionInc;
            Real halfWidthRef, halfHeightRef, halfWidthInc, halfHeightInc;
            Vec2 rightRef, upRef, rightInc, upInc;
            Vec2 refNormal;

            if (refIsA)
            {
                positionRef = positionA;
                halfWidthRef = halfWidthA;
                halfHeightRef = halfHeightA;
                rightRef = rightA;
                upRef = upA;

                positionInc = positionB;
                halfWidthInc = halfWidthB;
                halfHeightInc = halfHeightB;
                rightInc = rightB;
                upInc = upB;

                refNormal = normal;
            }
            else
            {
                positionRef = positionB;
                halfWidthRef = halfWidthB;
                halfHeightRef = halfHeightB;
                rightRef = rightB;
                upRef = upB;

                positionInc = positionA;
                halfWidthInc = halfWidthA;
                halfHeightInc = halfHeightA;
                rightInc = rightA;
                upInc = upA;

                refNormal = -normal;
            }

            Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;
            {
                // The reference face axis is the SAT axis with the minimum overlap.
                const bool useX = bestAxis == SATAxis::A_RIGHT || bestAxis == SATAxis::B_RIGHT;
                const Real sign = std::copysign(Real(1), glm::dot(refNormal, useX ? rightRef : upRef));

                if (useX)
                {
                    refFaceCenter = positionRef + rightRef * (sign * halfWidthRef);
                    sideDir = upRef;
                }
                else
                {
                    refFaceCenter = positionRef + upRef * (sign * halfHeightRef);
                    sideDir = rightRef;
                }
                const Vec2 edgeOffset = sideDir * (useX ? halfHeightRef : halfWidthRef);
                refEdgeStart = refFaceCenter + edgeOffset;
                refEdgeEnd = refFaceCenter - edgeOffset;
            }

            Vec2 incEdgeStart, incEdgeEnd;
            {
                const Real dotX = glm::dot(refNormal, rightInc);
                const Real dotY = glm::dot(refNormal, upInc);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                Vec2 faceCenter, edgeOffset;
                if (useX)
                {
                    faceCenter = positionInc + rightInc * (sign * halfWidthInc);
                    edgeOffset = upInc * halfHeightInc;
                }
                else
                {
                    faceCenter = positionInc + upInc * (sign * halfHeightInc);
                    edgeOffset = rightInc * halfWidthInc;
                }
                incEdgeStart = faceCenter + edgeOffset;
                incEdgeEnd = faceCenter - edgeOffset;
            }

            Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStart, incEdgeEnd };
            if (clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir)) continue;
            if (clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir)) continue;

            Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
            uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);
            const uint32_t contactIdBase = (uint32_t)bestAxis << 1;
            for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
            {
                if (glm::dot(clipped[i], refNormal) <= refPlaneDist + SAT_EPSILON)
                {
                    contactPoints[contactCount] = clipped[i];
                    contactIds[contactCount] = contactIdBase | i;
                    contactCount++;
                }
            }
            if (contactCount == 0) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                contactCount,
                contactPoints[0],
                contactPoints[1],
                contactIds[0],
                contactIds[1]
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-polygon collision");

        // NOTE: This routine works in Polygon B's local space rather than Box A's.
        // The box is described by only a center point and two axes, so transforming
        // it into B's frame is O(1); transforming B's (potentially large) vertex
        // list into A's frame, as the box-box/polygon-polygon routines do, would be
        // O(N). Polygon vertices are therefore read directly from localVertsB with
        // no per-pair transform or scratch buffer.

        enum class SATAxis : uint32_t
        {
            BOX_RIGHT,
            BOX_UP,
            POLY_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];

            const PolygonGeometry& localPolygonVertices = polyGeometryPtr[shapeB];
            const Vec2* localVertsB = localPolygonVertices.vertices();
            const Vec2* localNormalsB = localPolygonVertices.normals();
            const size_t vertexCount = localPolygonVertices.size();

            const Vec2 rightB = { cosB, sinB };
            const Vec2 upB = { -sinB, cosB };

            // Rotation taking a vector from Box A's local space into Polygon B's
            // local space (the inverse of the A<-B rotation used elsewhere in this
            // file), plus Box A's center expressed in B's local space.
            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2 rightA_inB = { relCos, -relSin };
            const Vec2 upA_inB = { relSin, relCos };

            const Vec2 centerDeltaBA = positionA - positionB;
            const Vec2 boxCenterInB = {
                glm::dot(centerDeltaBA, rightB),
                glm::dot(centerDeltaBA, upB)
            };

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::BOX_RIGHT;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            Real polyMinX, polyMaxX, polyMinY, polyMaxY;

            // Test Box X axis (rightA_inB), values kept relative to the box center
            // to match the box-local-space convention the rest of the routine uses.
            {
                Real polyMinXRaw, polyMaxXRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, rightA_inB, polyMinXRaw, polyMaxXRaw);

                const Real boxCenterProjX = glm::dot(boxCenterInB, rightA_inB);
                polyMinX = polyMinXRaw - boxCenterProjX;
                polyMaxX = polyMaxXRaw - boxCenterProjX;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfWidthA - polyMinX, polyMaxX + halfWidthA);
                if (overlap < Real(0)) continue;

                const Real polyCenterX = HALF * (polyMinX + polyMaxX);
                const Real axisFacing = -std::fabs(polyCenterX);
                depth = overlap;
                normalLocal = rightA_inB;
                flipSignIfNegative(normalLocal, polyCenterX);
                bestAxisType = SATAxis::BOX_RIGHT;
                bestAxisFacing = axisFacing;
            }

            // Test Box Y axis (upA_inB).
            {
                Real polyMinYRaw, polyMaxYRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, upA_inB, polyMinYRaw, polyMaxYRaw);

                const Real boxCenterProjY = glm::dot(boxCenterInB, upA_inB);
                polyMinY = polyMinYRaw - boxCenterProjY;
                polyMaxY = polyMaxYRaw - boxCenterProjY;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfHeightA - polyMinY, polyMaxY + halfHeightA);
                if (overlap < Real(0)) continue;

                const Real polyCenterY = HALF * (polyMinY + polyMaxY);
                const Real axisFacing = -std::fabs(polyCenterY);
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;
                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    normalLocal = upA_inB;
                    flipSignIfNegative(normalLocal, polyCenterY);
                    bestAxisType = SATAxis::BOX_UP;
                    bestAxisFacing = axisFacing;
                }
            }

            // Test Polygon edge outward normals. These are already expressed in
            // Polygon B's own local space, so no per-edge rotation is needed here
            // (unlike the box-local-space version of this routine).
            const Real polyCenterX = HALF * (polyMinX + polyMaxX);
            const Real polyCenterY = HALF * (polyMinY + polyMaxY);
            const Vec2 polyCenterRelVec = polyCenterX * rightA_inB + polyCenterY * upA_inB;
            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                const Vec2 nB = localNormalsB[i];
                Vec2 axisLocal = nB;

                const Real boxRadius = halfWidthA * std::fabs(glm::dot(axisLocal, rightA_inB)) + halfHeightA * std::fabs(glm::dot(axisLocal, upA_inB));

                // One-sided face separation: polygon max extent along its own outward
                // normal is dot(n, v_i); box min extent is (center - radius).
                const Real boxCenterProj = glm::dot(boxCenterInB, axisLocal);
                const Real overlap = glm::dot(axisLocal, localVertsB[i]) - (boxCenterProj - boxRadius);
                if (overlap < Real(0)) goto nextPair;

                const Real axisFacing = glm::dot(axisLocal, polyCenterRelVec);
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;

                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    // axisLocal is B's own outward edge normal (unambiguous from local winding);
                    // negate to match the "A to B" normal convention, no position heuristic needed.
                    normalLocal = -axisLocal;
                    bestAxisType = SATAxis::POLY_EDGE;
                    bestAxisIndex = i;
                    bestAxisFacing = axisFacing;
                }
            }

            {
                const bool refIsBox = bestAxisType != SATAxis::POLY_EDGE;

                Vec2 refNormalLocal;
                Vec2 refFaceCenterLocal;
                Vec2 sideDirLocal;
                Vec2 refEdgeStartLocal;
                Vec2 refEdgeEndLocal;

                if (refIsBox)
                {
                    refNormalLocal = normalLocal;

                    // normalLocal is exactly +-rightA_inB (BOX_RIGHT) or +-upA_inB (BOX_UP),
                    // so its box-local "x"/"y" components collapse to a known best-axis
                    // check plus a dot product against the corresponding box axis.
                    const bool useX = bestAxisType == SATAxis::BOX_RIGHT;
                    const Real sign = useX ? glm::dot(refNormalLocal, rightA_inB) : glm::dot(refNormalLocal, upA_inB);

                    if (useX)
                    {
                        refFaceCenterLocal = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        sideDirLocal = upA_inB;
                    }
                    else
                    {
                        refFaceCenterLocal = boxCenterInB + upA_inB * (sign * halfHeightA);
                        sideDirLocal = rightA_inB;
                    }

                    const Vec2 edgeOffset = sideDirLocal * (useX ? halfHeightA : halfWidthA);
                    refEdgeStartLocal = refFaceCenterLocal + edgeOffset;
                    refEdgeEndLocal = refFaceCenterLocal - edgeOffset;
                }
                else
                {
                    refNormalLocal = -normalLocal;

                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = nextIndex(i0, uint32_t(vertexCount));

                    refEdgeStartLocal = localVertsB[i0];
                    refEdgeEndLocal = localVertsB[i1];
                    refFaceCenterLocal = (refEdgeStartLocal + refEdgeEndLocal) * HALF;

                    const Vec2 edge = refEdgeEndLocal - refEdgeStartLocal;
                    const Real edgeLenSquared = glm::dot(edge, edge);

                    sideDirLocal = edge * (Real(-1) / std::sqrt(edgeLenSquared));
                }

                Vec2 incEdgeStartLocal, incEdgeEndLocal;
                uint32_t incidentEdge, incidentEdge2;

                if (refIsBox)
                {
                    incidentEdge = 0;
                    Real minDot = std::numeric_limits<Real>::max();

                    for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
                    {
                        // Already in Polygon B's local space, same frame as refNormalLocal:
                        // no per-edge rotation needed here (unlike the box-local-space version).
                        const Vec2 nB = localNormalsB[i];
                        const Real d = glm::dot(refNormalLocal, nB);
                        if (d < minDot)
                        {
                            minDot = d;
                            incidentEdge = i;
                        }
                    }

                    incidentEdge2 = nextIndex(incidentEdge, uint32_t(vertexCount));
                    incEdgeStartLocal = localVertsB[incidentEdge];
                    incEdgeEndLocal = localVertsB[incidentEdge2];
                }
                else
                {
                    // refNormalLocal is B's polygon-edge normal expressed in B's local
                    // space already; project it onto the box's axes-in-B to recover the
                    // same box-local x/y components the box-local-space version reads
                    // directly, then pick the box's incident edge exactly as before.
                    const Real dotX = glm::dot(refNormalLocal, rightA_inB);
                    const Real dotY = glm::dot(refNormalLocal, upA_inB);
                    const bool useX = std::fabs(dotX) > std::fabs(dotY);
                    const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                    const uint32_t* const incidentEdgeIds = BOX_INCIDENT_EDGE_IDS[useX][sign > 0];
                    incidentEdge = incidentEdgeIds[0];
                    incidentEdge2 = incidentEdgeIds[1];

                    Vec2 faceCenter, edgeOffset;
                    if (useX)
                    {
                        faceCenter = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        edgeOffset = upA_inB * halfHeightA;
                    }
                    else
                    {
                        faceCenter = boxCenterInB + upA_inB * (sign * halfHeightA);
                        edgeOffset = rightA_inB * halfWidthA;
                    }

                    incEdgeStartLocal = faceCenter + edgeOffset;
                    incEdgeEndLocal = faceCenter - edgeOffset;
                }

                Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStartLocal, incEdgeEndLocal };
                if (clipSegment(clipped[0], clipped[1], refEdgeStartLocal, -sideDirLocal)) continue;
                if (clipSegment(clipped[0], clipped[1], refEdgeEndLocal, sideDirLocal)) continue;

                const uint32_t bodyTag = refIsBox ? REFERENCE_BODY_TAG : 0u;
                uint32_t id1 = bodyTag | incidentEdge;
                uint32_t id2 = bodyTag | incidentEdge2;

                Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactCount = 0;
                const Real refPlaneDist = glm::dot(refFaceCenterLocal, refNormalLocal);

                const Vec2 normalWorld = rightB * normalLocal.x + upB * normalLocal.y;

                for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionB + rightB * clipped[i].x + upB * clipped[i].y;
                        contactIds[contactCount] = (i == 0) ? id1 : id2;
                        contactCount++;
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normalWorld,
                    depth,
                    contactCount,
                    contactPoints[0],
                    contactPoints[1],
                    contactIds[0],
                    contactIds[1]
                );
            }

        nextPair:
            continue;
        }
    }

    void NarrowPhaseCollisionDetector::collisionBoxCapsule(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            CapsuleManifold manifold;
            if (!collideCapsuleConvex<true>(
                nullptr, nullptr, 4, Vec2(halfWidthPtr[shapeA], halfHeightPtr[shapeA]),
                Vec2(positionXPtr[indexA], positionYPtr[indexA]), Vec2(rotationCosPtr[indexA], rotationSinPtr[indexA]),
                Vec2(positionXPtr[indexB], positionYPtr[indexB]), Vec2(rotationCosPtr[indexB], rotationSinPtr[indexB]),
                halfLengthPtr[shapeB], capsuleRadiusPtr[shapeB], manifold)) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                manifold.normal,
                manifold.depth,
                manifold.contactCount,
                manifold.points[0],
                manifold.points[1],
                manifold.ids[0],
                manifold.ids[1]
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-polygon collision");

        enum class SATAxis : uint32_t
        {
            A_EDGE,
            B_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

        alignas(CACHE_LINE_ALIGNMENT) Vec2 vertsB_inA[MAX_POLYGON_VERTEX_COUNT];
        alignas(CACHE_LINE_ALIGNMENT) Vec2 normalsB_inA[MAX_POLYGON_VERTEX_COUNT];  // B edge normals rotated into A-local.

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const PolygonGeometry& localVertsAContainer = polyGeometryPtr[shapeA];
            const PolygonGeometry& localVertsBContainer = polyGeometryPtr[shapeB];

            const size_t countA = localVertsAContainer.size();
            const size_t countB = localVertsBContainer.size();

            const Vec2 rightA = { cosA, sinA };
            const Vec2 upA = { -sinA, cosA };

            const Vec2 centerDelta = positionB - positionA;
            const Vec2 bodyDeltaLocal = {
                glm::dot(centerDelta, rightA),
                glm::dot(centerDelta, upA)
            };

            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2* localVertsA = localVertsAContainer.vertices();
            const Vec2* normalsA = localVertsAContainer.normals();
            const Vec2* localVertsB = localVertsBContainer.vertices();
            const Vec2* normalsB = localVertsBContainer.normals();

            for (size_t i = 0; i < countB; i++)
            {
                const Vec2 v = localVertsB[i];
                vertsB_inA[i] = {
                    bodyDeltaLocal.x + relCos * v.x - relSin * v.y,
                    bodyDeltaLocal.y + relSin * v.x + relCos * v.y
                };
            }

            // Facing direction for SAT axis tie-breaking: A origin to B origin, in A-local space.
            const Vec2 centerDeltaLocal = bodyDeltaLocal;

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::A_EDGE;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            // Face-separation SAT. For a face with outward unit normal n, the owning polygon's
            // max extent along n is just dot(n, faceVertex), so only the OTHER polygon is projected
            // (O(nA*nB) per direction instead of re-projecting both shapes on every axis).
            // overlap = faceDist - min_other(dot(n, v)); negative => separating axis.
            auto consider = [&](Real overlap, Vec2 axisLocal, SATAxis axisType, uint32_t axisIndex)
                {
                    if (overlap > depth) return;

                    const Real facingSign = Real(axisType == SATAxis::B_EDGE) * Real(2) - Real(1);
                    const Real axisFacing = facingSign * glm::dot(axisLocal, centerDeltaLocal);

                    if (overlap < depth || axisFacing < bestAxisFacing)
                    {
                        depth = overlap;
                        // axisLocal is that shape's own outward edge normal: A's points from A to B
                        // already; B's must be negated to match.
                        normalLocal = (axisType == SATAxis::A_EDGE) ? axisLocal : -axisLocal;

                        bestAxisType = axisType;
                        bestAxisIndex = axisIndex;
                        bestAxisFacing = axisFacing;
                    }
                };

            for (uint32_t i = 0; i < uint32_t(countA); i++)
            {
                const Vec2 axisLocal = normalsA[i];

                Real minB = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countB; j++)
                {
                    const Real p = glm::dot(vertsB_inA[j], axisLocal);
                    minB = std::fmin(minB, p);
                }

                const Real overlap = glm::dot(localVertsA[i], axisLocal) - minB;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::A_EDGE, i);
            }

            for (uint32_t i = 0; i < uint32_t(countB); i++)
            {
                const Vec2 nB = normalsB[i];
                const Vec2 axisLocal = { relCos * nB.x - relSin * nB.y, relSin * nB.x + relCos * nB.y };
                normalsB_inA[i] = axisLocal;

                Real minA = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countA; j++)
                {
                    const Real p = glm::dot(localVertsA[j], axisLocal);
                    minA = std::fmin(minA, p);
                }

                const Real overlap = glm::dot(vertsB_inA[i], axisLocal) - minA;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::B_EDGE, i);
            }

            {
                const bool refIsA = bestAxisType == SATAxis::A_EDGE;

                const Vec2* refVertsPtr = refIsA ? localVertsA : vertsB_inA;
                const size_t refCount = refIsA ? countA : countB;

                const Vec2* incVertsPtr = refIsA ? vertsB_inA : localVertsA;
                const size_t incCount = refIsA ? countB : countA;

                Vec2 refNormalLocal = refIsA ? normalLocal : -normalLocal;
                Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;

                {
                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = nextIndex(i0, uint32_t(refCount));

                    refEdgeStart = refVertsPtr[i0];
                    refEdgeEnd = refVertsPtr[i1];
                    refFaceCenter = (refEdgeStart + refEdgeEnd) * HALF;

                    const Vec2 edge = refEdgeEnd - refEdgeStart;
                    const Real edgeLenSquared = glm::dot(edge, edge);

                    sideDir = edge * (Real(1) / std::sqrt(edgeLenSquared));
                }

                uint32_t incidentEdgeIndex = 0;
                Real minDot = std::numeric_limits<Real>::max();

                for (uint32_t i = 0; i < uint32_t(incCount); i++)
                {
                    const Vec2 nLocal = refIsA ? normalsB_inA[i] : normalsA[i];

                    const Real d = glm::dot(refNormalLocal, nLocal);
                    if (d < minDot)
                    {
                        minDot = d;
                        incidentEdgeIndex = i;
                    }
                }

                Vec2 incEdgeStart = incVertsPtr[incidentEdgeIndex];
                Vec2 incEdgeEnd = incVertsPtr[nextIndex(incidentEdgeIndex, uint32_t(incCount))];

                Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStart, incEdgeEnd };
                if (clipSegment(clipped[0], clipped[1], refEdgeStart, sideDir)) goto nextPair;
                if (clipSegment(clipped[0], clipped[1], refEdgeEnd, -sideDir)) goto nextPair;

                const Real refPlaneDist = glm::dot(refFaceCenter, refNormalLocal);

                const uint32_t bodyTag = refIsA ? REFERENCE_BODY_TAG : 0u;
                const uint32_t id1 = bodyTag | incidentEdgeIndex;
                const uint32_t id2 = bodyTag | nextIndex(incidentEdgeIndex, uint32_t(incCount));

                Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactCount = 0;

                const Vec2 normalWorld = rightA * normalLocal.x + upA * normalLocal.y;

                for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionA + rightA * clipped[i].x + upA * clipped[i].y;
                        contactIds[contactCount] = (i == 0) ? id1 : id2;
                        contactCount++;
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normalWorld,
                    depth,
                    contactCount,
                    contactPoints[0],
                    contactPoints[1],
                    contactIds[0],
                    contactIds[1]
                );
            }
        nextPair:
            continue;
        }
    }

    void NarrowPhaseCollisionDetector::collisionPolygonCapsule(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const PolygonGeometry& localPolygonVertices = polyGeometryPtr[shapeA];

            CapsuleManifold manifold;
            if (!collideCapsuleConvex<false>(
                localPolygonVertices.vertices(), localPolygonVertices.normals(), uint32_t(localPolygonVertices.size()), Vec2(),
                Vec2(positionXPtr[indexA], positionYPtr[indexA]), Vec2(rotationCosPtr[indexA], rotationSinPtr[indexA]),
                Vec2(positionXPtr[indexB], positionYPtr[indexB]), Vec2(rotationCosPtr[indexB], rotationSinPtr[indexB]),
                halfLengthPtr[shapeB], capsuleRadiusPtr[shapeB], manifold)) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                manifold.normal,
                manifold.depth,
                manifold.contactCount,
                manifold.points[0],
                manifold.points[1],
                manifold.ids[0],
                manifold.ids[1]
            );
        }
    }

    void NarrowPhaseCollisionDetector::collisionCapsuleCapsule(
        const std::vector<ObjectPair>& pairs,
        std::vector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Capsule-capsule collision");

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT capsuleRadiusPtr = capsules.radius;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfLengthA = halfLengthPtr[shapeA];
            const Real halfLengthB = halfLengthPtr[shapeB];
            const Real radiusA = capsuleRadiusPtr[shapeA];
            const Real radiusB = capsuleRadiusPtr[shapeB];

            const Vec2 centerDelta = positionB - positionA;
            const Real radiusSum = radiusA + radiusB;

            // Bounding circle rejection (skips the AABB corner cases from the broad phase).
            const Real reach = halfLengthA + halfLengthB + radiusSum;
            if (glm::dot(centerDelta, centerDelta) >= reach * reach) continue;

            const Vec2 axisA = { rotationCosPtr[indexA], rotationSinPtr[indexA] };
            const Vec2 axisB = { rotationCosPtr[indexB], rotationSinPtr[indexB] };

            // Closest points of the two core segments, parametrized from their centers.
            const Real b = glm::dot(axisA, axisB);
            const Real centerAlongA = glm::dot(centerDelta, axisA);
            const Real c = -centerAlongA;
            const Real f = -glm::dot(axisB, centerDelta);
            const Real sinSquared = Real(1) - b * b;
            const bool parallel = sinSquared < CAPSULE_PARALLEL_SIN_SQUARED;

            // Overlap of the segments along axisA (meaningful only when parallel).
            const Real extentB = std::fabs(b) * halfLengthB;
            const Real overlapLo = std::fmax(-halfLengthA, centerAlongA - extentB);
            const Real overlapHi = std::fmin(halfLengthA, centerAlongA + extentB);
            const bool hasOverlap = parallel && overlapHi - overlapLo >= ZERO_DIVISION_BOUNDARY;

            Real s, t;
            if (hasOverlap)
            {
                s = HALF * (overlapLo + overlapHi);
                t = clampValue(b * s + f, -halfLengthB, halfLengthB);
            }
            else
            {
                s = parallel ? Real(0) : clampValue((b * f - c) / sinSquared, -halfLengthA, halfLengthA);
                t = b * s + f;
                if (t < -halfLengthB || t > halfLengthB)
                {
                    t = clampValue(t, -halfLengthB, halfLengthB);
                    s = clampValue(b * t - c, -halfLengthA, halfLengthA);
                }
            }

            const Vec2 pointA = positionA + axisA * s;
            const Vec2 pointB = positionB + axisB * t;
            const Vec2 delta = pointB - pointA;
            const Real squaredDistance = glm::dot(delta, delta);
            if (squaredDistance >= radiusSum * radiusSum) continue;

            Vec2 normal;
            Real depth;
            if (squaredDistance >= ZERO_DIVISION_BOUNDARY_SQUARED)
            {
                const Real distance = std::sqrt(squaredDistance);
                normal = delta * (Real(1) / distance);
                depth = radiusSum - distance;
            }
            else [[unlikely]]
            {
                // Core segments intersect: separate along the cheaper of the two side axes.
                const Vec2 sideA = { -axisA.y, axisA.x };
                const Vec2 sideB = { -axisB.y, axisB.x };
                const Real offsetA = glm::dot(centerDelta, sideA);
                const Real offsetB = glm::dot(centerDelta, sideB);
                const Real depthA = radiusSum + std::fabs(glm::dot(axisB, sideA)) * halfLengthB - std::fabs(offsetA);
                const Real depthB = radiusSum + std::fabs(glm::dot(axisA, sideB)) * halfLengthA - std::fabs(offsetB);

                if (depthA <= depthB)
                {
                    normal = sideA;
                    flipSignIfNegative(normal, offsetA);
                    depth = depthA;
                }
                else
                {
                    normal = sideB;
                    flipSignIfNegative(normal, offsetB);
                    depth = depthB;
                }
            }

            Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD] = { Vec2(), Vec2() };
            uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD] = { 0, 0 };
            uint32_t contactCount = 0;

            if (hasOverlap)
            {
                // Side by side: one contact at each end of the overlap, if actually within reach.
                const Vec2 sideB = { -axisB.y, axisB.x };
                const Real ends[2] = { overlapLo, overlapHi };
                Real minGap = std::numeric_limits<Real>::max();

                for (uint32_t k = 0; k < MAX_CONTACTS_PER_MANIFOLD; k++)
                {
                    const Vec2 p = positionA + axisA * ends[k];
                    const Real gap = std::fabs(glm::dot(p - positionB, sideB));
                    if (gap >= radiusSum) continue;

                    contactPoints[contactCount] = p + normal * radiusA;
                    contactIds[contactCount] = k;
                    contactCount++;
                    minGap = std::fmin(minGap, gap);
                }
                if (contactCount != 0) depth = radiusSum - minGap;
            }

            if (contactCount == 0)
            {
                contactPoints[0] = pointA + normal * radiusA;
                contactIds[0] = 2;
                contactCount = 1;
            }

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                contactCount,
                contactPoints[0],
                contactPoints[1],
                contactIds[0],
                contactIds[1]
            );
        }
    }
}