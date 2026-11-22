#pragma once
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/ConstexprMath.h"

namespace PS_AGONY2D
{
    // Numeric tolerances.
    constexpr Real ZERO_DIVISION_BOUNDARY = 1e-4;
    constexpr Real ZERO_DIVISION_BOUNDARY_SQUARED = 1e-8;
    constexpr Real SAT_EPSILON = 1e-4;
    constexpr Real HALF = Real(0.5);

    // Fallback normal for coincident circle centers.
    constexpr Real DEFAULT_NORMAL_X = 1;
    constexpr Real DEFAULT_NORMAL_Y = 0;

    // Threading.
    constexpr size_t PAIRS_PER_WORKER = 3000;
    constexpr size_t MULTITHREAD_PAIR_THRESHOLD = PAIRS_PER_WORKER * 2;
    constexpr size_t LOAD_BALANCING_FACTOR = 4;

    // Manifolds and contact ids.
    constexpr size_t MAX_CONTACTS_PER_MANIFOLD = 2;
    constexpr uint32_t REFERENCE_BODY_TAG = 0x80000000u;
    constexpr uint32_t INVALID_CONTACT_ID = std::numeric_limits<uint32_t>::max();
    constexpr ObjectDenseIndex INVALID_COLLIDER_INDEX = std::numeric_limits<ObjectDenseIndex>::max();

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
    // TODO: Must depend on their length.
    constexpr Real CAPSULE_PARALLEL_SIN_SQUARED = Real(1e-3);    // Below this, capsule axes count as parallel (2 contacts).

    // Contact reuse: a manifold whose colliders barely moved since it was computed is reused as it is.
    constexpr bool CONTACT_REUSE_ENABLED = true;
    constexpr bool CONTACT_REUSE_PRINT_STATS = false; // Prints the reuse statistics once per narrow phase.
    constexpr Real CONTACT_REUSE_LINEAR_TOLERANCE = Real(5e-3); // 5 cm.
    constexpr Real CONTACT_REUSE_LINEAR_TOLERANCE_SQUARED = CONTACT_REUSE_LINEAR_TOLERANCE * CONTACT_REUSE_LINEAR_TOLERANCE;
    constexpr Real CONTACT_REUSE_ANGULAR_COS_TOLERANCE = ConstexprMath::cos(ConstexprMath::to_radians(0.1f)); // Cosine of tolerance.
    constexpr Real CONTACT_REUSE_INVALID_POSITION = Real(1e15); // Snapshot position that never passes the linear check.

    // Box as a convex polygon: counter-clockwise, normal i belongs to edge i -> i + 1.
    alignas(std::hardware_destructive_interference_size) const Vec2 BOX_NORMALS[4] = {
        Vec2(Real(0), Real(-1)),
        Vec2(Real(1), Real(0)),
        Vec2(Real(0), Real(1)),
        Vec2(Real(-1), Real(0))
    };
}