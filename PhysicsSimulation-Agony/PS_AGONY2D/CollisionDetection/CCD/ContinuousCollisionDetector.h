#pragma once
#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Limits.h"
#include "PS_AGONY2D/CollisionDetection/Broad/BvhTree.h"

namespace PS_AGONY2D
{
	// Continuous collision: fast DYNAMIC colliders vs STATIC colliders only.
	//
	// Translation only:
	//   Each fast collider is represented by a "core" circle at the collider origin (radius = CORE_FRACTION * min extent).
	//   The core is swept from its previous to its current position against the static BVH. On the earliest hit the body
	//   is moved back along its own displacement to that time of impact. Velocity and rotation are kept: the narrow phase
	//   then sees a normal (overlapping) contact and the existing solver resolves it in the same step.
	//
	// Pipeline contract (per physics step):
	//   1. snapshotPoses()  after bodies were woken/reordered, BEFORE integration.
	//   2. resolve()        after collider world positions + static/dynamic trees are valid, BEFORE the broad phase pairs.
	//                       Returns true when any body moved: caller must rebuild dynamic AABBs and refit the dynamic tree.
	//
	// Limits: no rotational CCD, no dynamic-vs-dynamic. Collider origin must lie inside its shape (polygons whose origin
	// is outside are skipped).
	//
	// Useful note: Stateless/pipeline. Inside containers can reuse memory from shared buffers.
	class ContinuousCollisionDetector
	{
	public:
		struct Settings
		{
			Real fastFactor = 1.0; // Fast when displacement > FAST_FACTOR * min extent.
			Real coreFraction = 0.5; // Core radius / min extent. < 1: sliding on a surface never "hits".
		};

		static constexpr Real MIN_DISPLACEMENT_SQ = Real(1e-8); // Skip (almost) stationary bodies before any other work.

		Settings settings;
	public:
		void snapshotPoses(const BodySoA& bodies, const ColliderSoA& colliders);

		bool resolve(
			BodySoA& bodies,
			ColliderSoA& colliders,
			const CircleSoA& circles,
			const BoxSoA& boxes,
			const PolygonSoA& polygons,
			const CapsuleSoA& capsules,
			const BroadPhaseInternal::BvhTree& staticTree
		);

		size_t getMemoryUsage() const;
	private:
		struct StaticSweepData;

		// Sweeps the cores of all fast dynamic colliders of one shape type against the static tree.
		void processShapeRange(
			IndexRange shapeRange,
			const ObjectDenseIndex* ECSTASY_RESTRICT colliderIndexPtr,
			const BodySoA& bodies,
			const ColliderSoA& colliders,
			const StaticSweepData& data
		);

		// Orientation before integration.
		CacheLineAlignedVector<Real> prevComX;
		CacheLineAlignedVector<Real> prevComY;
		CacheLineAlignedVector<Real> prevColliderX;
		CacheLineAlignedVector<Real> prevColliderY;

		CacheLineAlignedVector<ObjectDenseIndex> hitBodies; // Per step scratch.
		CacheLineAlignedVector<Real> bodyHitTime; // Indexed by body. Invariant: all 1 between calls.
	};
}