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

		static constexpr bool COLLECT_STATS = true;

		struct DebugData
		{
			struct SweepStats
			{
				uint32_t total;
				uint32_t moved;
				uint32_t fastBody;
				uint32_t fastCollider;
				uint32_t hits;
				uint32_t nodesVisited;
				uint32_t leavesVisited;
				uint32_t aabbCandidates;
			};

			struct CastStats
			{
				uint32_t calls;
				uint32_t misses;
			};

			struct PolygonStats
			{
				uint32_t rejectedAtEdge[MAX_POLYGON_VERTEX_COUNT];
				uint32_t rejectedAfterLoop;
				uint32_t startedInside;
				uint32_t faceHit;
				uint32_t cornerCast;
			};

			SweepStats sweep[size_t(ColliderShapeType::COUNT)];
			CastStats cast[size_t(ColliderShapeType::COUNT)];
			PolygonStats polygon;
		};

		Settings settings;
	private:
		DebugData debugData;
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

		void resetDebugData() noexcept { debugData = DebugData{}; }
		const DebugData& getDebugData() const noexcept { return debugData; }
	private:
		struct StaticSweepData;

		// Sweeps the cores of all fast dynamic colliders of one shape type against the static tree.
		template<typename ShapeSoA>
		void processShapeRange(
			const ShapeSoA& soa,
			ColliderShapeType type,
			const BodySoA& bodies,
			const ColliderSoA& colliders,
			const StaticSweepData& data
		);

		// Orientation before integration.
		CacheLineAlignedVector<Real> prevOffsetX;
		CacheLineAlignedVector<Real> prevOffsetY;
		CacheLineAlignedVector<Real> prevColliderX;
		CacheLineAlignedVector<Real> prevColliderY;

		CacheLineAlignedVector<ObjectDenseIndex> hitBodies; // Per step scratch.
		CacheLineAlignedVector<Real> bodyHitTime; // Indexed by body. Invariant: all 1 between calls.
	};
}