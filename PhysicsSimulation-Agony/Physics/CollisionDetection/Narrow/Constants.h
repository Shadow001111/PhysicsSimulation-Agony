#pragma once
#include "Physics/Types.h"

namespace PS_AGONY
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