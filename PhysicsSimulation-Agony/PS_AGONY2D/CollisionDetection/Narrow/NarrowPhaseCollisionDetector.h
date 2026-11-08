#pragma once
#include "ContactCache.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/SymmetricMatrix.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/ExecutionPolicy.h"

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

	// bodyA/bodyB are the OWNING bodies of colliderA/colliderB, resolved once at generation
	// time so downstream solvers (which only ever act on bodies) never re-resolve them.
	struct BodyCollisionData
	{
		ObjectDenseIndex colliderA, colliderB;
		Vec2 normal;
		Real penetration;
		uint32_t contactCount;
		Vec2 contactPoints[2];
		uint32_t contactIds[2];
		std::array<PersistentImpulses, 2> persistentContactData{};

		BodyCollisionData() = default;

		BodyCollisionData(
			ObjectDenseIndex colliderA, ObjectDenseIndex colliderB,
			Vec2 normal,
			Real depth,
			uint32_t contactCount,
			Vec2 contactPoint1, Vec2 contactPoint2,
			uint32_t contactId1, uint32_t contactId2
		) :
			colliderA(colliderA), colliderB(colliderB),
			normal(normal), penetration(depth), contactCount(contactCount)
		{
			contactPoints[0] = contactPoint1;
			contactPoints[1] = contactPoint2;
			contactIds[0] = contactId1;
			contactIds[1] = contactId2;
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
			}

			ChunkData& operator=(ChunkData&& other) noexcept
			{
				if (this != &other)
				{
					start = other.start;
					end = other.end;
					pairs = std::move(other.pairs);
					results = std::move(other.results);
				}
				return *this;
			}

			void clear()
			{
				for (auto& vec : pairs.getDirectAccess())
					vec.clear();
				results.clear();
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

		// Full manifolds touching sleeping colliders. Kept intact (geometry, ids, impulses), so a woken body
		// can be solved immediately instead of waiting for the narrow phase to regenerate the contact.
		CacheLineAlignedVector<BodyCollisionData> sleepingCollisionArray;
		ContactCache wakeCache; // Lazily built over 'previousCollisionArray' in wakeContactData.

		static constexpr uint8_t FLAG_IN_SET = 1; // Scratch: collider is in the current sleep/wake set.
		static constexpr uint8_t FLAG_ASLEEP = 2; // Persistent: collider is asleep (set by sleep, cleared by wake).
		std::vector<uint8_t> colliderFlags;

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
		static constexpr size_t DETERMINISTIC_CHUNK_SIZE = 2048;

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

		// Moves every manifold touching the given colliders from 'previousCollisionArray' to the sleeping array.
		void sleepContactData(std::span<const ObjectDenseIndex> colliderIndices);

		// Restores sleeping manifolds touching the given colliders (call after the narrow phase, before solving).
		// Pair already generated this frame: impulses are copied into it. Otherwise the stored manifold is appended
		// to 'previousCollisionArray' (may reallocate: re-fetch the span via getBodyCollisionData()).
		// Manifolds whose partner is still asleep stay stored (no islands).
		void wakeContactData(std::span<const ObjectDenseIndex> colliderIndices);

		// 'deletedColliders' is the collider swap-remove log (same shape as ObjectDeletion,
		// but every index in it is a collider index, not a body index).
		// TODO: Must fix/delete data in 'sleepPersistentContactData', to not leave data lying forever.
		void remapPersistentContactData(std::span<const ObjectDeletion> deletedColliders);

		void resetCollisionData();

		std::span<BodyCollisionData> getBodyCollisionData() noexcept { return previousCollisionArray; }
		std::span<const BodyCollisionData> getBodyCollisionData() const noexcept { return previousCollisionArray; }

		size_t getMemoryUsage() const;
	private:
		void markColliders(std::span<const ObjectDenseIndex> colliderIndices);
		void unmarkColliders(std::span<const ObjectDenseIndex> colliderIndices);
		bool isMarked(ObjectDenseIndex i) const noexcept { return size_t(i) < colliderFlags.size() && (colliderFlags[size_t(i)] & FLAG_IN_SET); }
		bool isAsleep(ObjectDenseIndex i) const noexcept { return size_t(i) < colliderFlags.size() && (colliderFlags[size_t(i)] & FLAG_ASLEEP); }

		void findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs);
		void findCollisionsMultiThreaded(std::span<const ObjectPair> colliderPairs, size_t maxWorkerCount);

		// Appends results to 'out' and applies warm-starting to the appended range only.
		void processPairs(std::span<const ObjectPair> pairs, ChunkData& chunkData, CacheLineAlignedVector<BodyCollisionData>& out);

		// Moves persistent contact data from previous frame (read-only lookups, thread-safe).
		// (Copies persistent impulses from previous to current frame, only matching contacts).
		void movePreviousContactData(size_t firstResult, std::span<BodyCollisionData> chunkCollisionData);

		void collisionCircleCircle	(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox		(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon	(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCircleCapsule	(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox		(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon	(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionBoxCapsule	(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionPolygonCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
		void collisionCapsuleCapsule(std::span<const ObjectPair> pairs, CacheLineAlignedVector<BodyCollisionData>& outCollisionData);
	};
}