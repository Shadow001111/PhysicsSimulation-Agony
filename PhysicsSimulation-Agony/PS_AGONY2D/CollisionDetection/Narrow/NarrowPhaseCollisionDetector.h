#pragma once
#include "ContactCache.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/SymmetricMatrix.h"
#include "PS_AGONY2D/MemoryUsageTracking.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/ExecutionPolicy.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <span>

namespace PS_AGONY2D
{
	struct PersistentImpulses
	{
		Real normalImpulseAccumulator = 0;
		Real tangentImpulseAccumulator = 0;
	};

	// Persistent manifold between two colliders, identified by handles (stable under swap-remove).
	// Collision methods fill it through the index constructor: the collider storage then holds dense indices
	// (indexA/indexB), and processPairs replaces them with handles before the data leaves the narrow phase.
	struct BodyCollisionData
	{
		static_assert(sizeof(ObjectDenseIndex) == sizeof(uint32_t));

		union
		{
			struct { Handle colliderA, colliderB; };
			struct { ObjectDenseIndex indexA; uint32_t paddingA; ObjectDenseIndex indexB; uint32_t paddingB; };
		};
		Vec2 normal;
		Real penetration;
		uint32_t contactCount;
		Vec2 contactPoints[2];
		uint32_t contactIds[2];
		std::array<PersistentImpulses, 2> persistentContactData{};

		BodyCollisionData() noexcept : colliderA(), colliderB() {}

		BodyCollisionData(
			ObjectDenseIndex colliderIndexA, ObjectDenseIndex colliderIndexB,
			Vec2 normal,
			Real depth,
			uint32_t contactCount,
			Vec2 contactPoint1, Vec2 contactPoint2,
			uint32_t contactId1, uint32_t contactId2
		) :
			normal(normal), penetration(depth), contactCount(contactCount)
		{
			indexA = colliderIndexA;
			indexB = colliderIndexB;
			contactPoints[0] = contactPoint1;
			contactPoints[1] = contactPoint2;
			contactIds[0] = contactId1;
			contactIds[1] = contactId2;
		}
	};

	// Collider transforms a manifold was computed at. Stored in arrays index-aligned with the manifold arrays.
	struct ContactReuseData
	{
		Vec2 posA, posB;
		Vec2 rotA, rotB; // (cos, sin).
		Real penetration; // Manifold penetration at these transforms.
		Vec2 contactPoints[2]; // Manifold contact points at these transforms.
	};

	// Per-frame outcome of the contact reuse test of every pair that had a manifold in the previous frame or none.
	struct ContactReuseStats
	{
		uint32_t reused = 0;
		uint32_t notCached = 0;
		uint32_t handleMismatch = 0;
		uint32_t movedLinear = 0;
		uint32_t movedAngular = 0;

		void add(const ContactReuseStats& other) noexcept
		{
			reused += other.reused;
			notCached += other.notCached;
			handleMismatch += other.handleMismatch;
			movedLinear += other.movedLinear;
			movedAngular += other.movedAngular;
		}
	};

	class NarrowPhaseCollisionDetector
	{
		// Structures.

		static constexpr size_t BODY_TYPE_COUNT = static_cast<size_t>(ColliderShapeType::COUNT);

		struct alignas(64) ChunkData
		{
			size_t start = 0, end = 0;
			SymmetricMatrix<CacheLineAlignedVector<ObjectPair>, BODY_TYPE_COUNT> pairs; // Pairs of COLLIDER indices.
			CacheLineAlignedVector<BodyCollisionData> results;
			CacheLineAlignedVector<ContactReuseData> reuse;
			CacheLineAlignedVector<ObjectPair> remainingPairs; // Pairs left for the collision methods after contact reuse.
			ContactReuseStats reuseStats;
			alignas(64) std::atomic<bool> finished{ false };

			ChunkData() = default;
			~ChunkData() = default;
			ChunkData(const ChunkData&) = delete;
			ChunkData& operator=(const ChunkData&) = delete;

			ChunkData(ChunkData&& other) noexcept
			{
				start = other.start;
				end = other.end;
				pairs = std::move(other.pairs);
				results = std::move(other.results);
				reuse = std::move(other.reuse);
				remainingPairs = std::move(other.remainingPairs);
				reuseStats = other.reuseStats;
			}

			ChunkData& operator=(ChunkData&& other) noexcept
			{
				if (this != &other)
				{
					start = other.start;
					end = other.end;
					pairs = std::move(other.pairs);
					results = std::move(other.results);
					reuse = std::move(other.reuse);
					remainingPairs = std::move(other.remainingPairs);
					reuseStats = other.reuseStats;
				}
				return *this;
			}

			void clear()
			{
				for (auto& vec : pairs.getDirectAccess())
					vec.clear();
				results.clear();
				reuse.clear();
				remainingPairs.clear();
				reuseStats = {};
			}
		};

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(
			std::span<const ObjectPair>, CacheLineAlignedVector<BodyCollisionData>&
			);

		// Static fields.

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		// Fields.

		CacheLineAlignedVector<BodyCollisionData> currentCollisionArray;
		CacheLineAlignedVector<ChunkData> chunks;

		CacheLineAlignedVector<BodyCollisionData> previousCollisionArray;
		ContactCache contactCache;

		// Contact reuse snapshots. Always the same size and order as the manifold array they belong to
		// (current/previous, island and wake storage). Unused when CONTACT_REUSE_ENABLED is false.
		CacheLineAlignedVector<ContactReuseData> currentReuseArray;
		CacheLineAlignedVector<ContactReuseData> previousReuseArray;
		std::vector<std::vector<ContactReuseData>> islandReuse;
		std::vector<ContactReuseData> wakeStagingReuse;
		ContactReuseStats reuseStats;

		// Full manifolds of sleeping islands, indexed by sleep island id - 2. Kept intact (geometry, ids, impulses), so a
		// woken island can be solved immediately instead of waiting for the narrow phase to regenerate its contacts.
		// A woken island's vector is cleared but keeps its capacity.
		std::vector<std::vector<BodyCollisionData>> islandManifolds;

		// Wake scratch: manifolds of the woken islands, a table over their pair keys and a consumed mask.
		std::vector<BodyCollisionData> wakeStaging;
		std::vector<uint8_t> wakeConsumed;
		ContactCache wakeCache;

		// Scratch for takeContactsOfColliders, indexed by handle slot. All zeros between calls.
		std::vector<uint8_t> editedColliderMask;

		// 'previousCollisionArray' was compacted/appended since 'contactCache' was built. Rebuilt lazily in findCollisions.
		bool contactCacheDirty = false;

		// SoA data viewers.
		BodySoAViewer bodies;
		ColliderSoAViewer colliders;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		PolygonSoAViewer polygons;
		CapsuleSoAViewer capsules;
	public:
		// When true, output order is identical for single/multi-threaded runs (and any thread count).
		// Both paths use the same fixed-size chunking, merged in chunk order.
		// Requires the input pair order to be deterministic as well.
		static constexpr bool DETERMINISTIC = true;
		static constexpr size_t DETERMINISTIC_CHUNK_SIZE = 1024;// 2048;

		NarrowPhaseCollisionDetector() = default;
		~NarrowPhaseCollisionDetector() = default;
		NarrowPhaseCollisionDetector(const NarrowPhaseCollisionDetector&) = delete;
		NarrowPhaseCollisionDetector& operator=(const NarrowPhaseCollisionDetector&) = delete;
		NarrowPhaseCollisionDetector(NarrowPhaseCollisionDetector&&) = delete;
		NarrowPhaseCollisionDetector& operator=(NarrowPhaseCollisionDetector&&) = delete;

		void setDataViewers(
			const BodySoAViewer& bodies,
			const ColliderSoAViewer& colliders,
			const CircleSoAViewer& circles,
			const BoxSoAViewer& boxes,
			const PolygonSoAViewer& polygons,
			const CapsuleSoAViewer& capsules
		);

		std::span<BodyCollisionData> findCollisions(
			std::span<const ObjectPair> colliderPairs,
			ExecutionPolicy executionPolicy = ExecutionPolicy::Auto
		);

		// 'collidersToCheck' and the first element of each 'outColliding' pair are COLLIDER indices.
		void findCollisionsInCircle(
			std::span<const ObjectDenseIndex> collidersToCheck,
			Vec2 pos,
			Real radius,
			std::vector<std::pair<ObjectDenseIndex, Real>>& outColliding
		) const;

		void rebuildContactCache();

		// Moves every manifold touching a body of the Sleeping group from 'previousCollisionArray' to its sleep island's
		// storage (island id = bodies.sleepIslandId). Manifolds of deleted colliders are dropped.
		void sleepContactData(IndexRange sleepingBodies);

		// Restores the stored manifolds of the given islands (call after the narrow phase, before solving).
		// Pair already generated this frame: impulses are copied into it. Otherwise the stored manifold is appended
		// to 'previousCollisionArray' (may reallocate: re-fetch the span via getBodyCollisionData()).
		// Cost is proportional to the woken manifolds plus one light pass over 'previousCollisionArray'.
		void wakeContactData(std::span<const uint32_t> islandIds);

		// For colliders whose shape, transform or existence changed (their saved contacts are invalid).
		// Sleeping islands: manifolds touching the colliders are erased and the ids of the affected islands are appended
		// to 'outSleepIslandIds' (ascending, unique). The caller wakes them.
		// Awake manifolds are kept; the other collider of each touching manifold is appended to 'outAwakePartners'.
		void takeContactsOfColliders(
			std::span<const Handle> colliderHandles,
			std::vector<uint32_t>& outSleepIslandIds,
			std::vector<Handle>& outAwakePartners
		);

		void resetCollisionData();

		std::span<BodyCollisionData> getBodyCollisionData() noexcept { return previousCollisionArray; }
		std::span<const BodyCollisionData> getBodyCollisionData() const noexcept { return previousCollisionArray; }

		// Totals of the last findCollisions call.
		const ContactReuseStats& getContactReuseStats() const noexcept { return reuseStats; }

		void collectMemoryUsage(MemoryUsageTracking& memoryTracking, uint32_t parentNodeIndex) const;
	private:
		bool isAlive(const BodyCollisionData& d) const noexcept { return colliders.handles->isValid(d.colliderA) && colliders.handles->isValid(d.colliderB); }

		// Order-independent: the same collider pair gives the same key whatever the dense order is.
		static uint64_t makePairKey(Handle a, Handle b) noexcept
		{
			return (uint64_t(std::min(a.slot, b.slot)) << 32) | uint64_t(std::max(a.slot, b.slot));
		}

		void findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs);
		void findCollisionsMultiThreaded(std::span<const ObjectPair> colliderPairs, size_t maxWorkerCount);

		// Appends results to 'out' (snapshots to 'outReuse'): reused manifolds first, then the computed ones, converted to handles and warm-started.
		void processPairs(
			std::span<const ObjectPair> pairs,
			ChunkData& chunkData,
			CacheLineAlignedVector<BodyCollisionData>& out,
			CacheLineAlignedVector<ContactReuseData>& outReuse
		);

		// Appends the previous manifold (and snapshot) of every pair of the chunk that qualifies for contact reuse.
		// Returns the pairs that still need collision methods.
		std::span<const ObjectPair> reuseContacts(
			std::span<const ObjectPair> pairs,
			ChunkData& chunkData,
			CacheLineAlignedVector<BodyCollisionData>& out,
			CacheLineAlignedVector<ContactReuseData>& outReuse
		);

		// Converts dense collider indices to handles for results [firstResult, end) and, if the cache is not empty,
		// copies persistent impulses from the previous frame (read-only lookups, thread-safe).
		// Only contacts of pairs with the same collider order and matching contact ids are copied.
		// Also records the collider transforms of those results in 'reuse'.
		void finalizeResults(size_t firstResult, std::span<BodyCollisionData> results, CacheLineAlignedVector<ContactReuseData>& reuse);

		void collisionCircleCircle(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCircleCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionPolygonCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCapsuleCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
	};
}