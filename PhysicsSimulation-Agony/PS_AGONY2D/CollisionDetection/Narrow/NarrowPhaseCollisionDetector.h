#pragma once
#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/SymmetricMatrix.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/ExecutionPolicy.h"

#include <atomic>
#include <array>
#include <span>
#include <bit>
#include <cstring>
#include <algorithm>
#include <xmmintrin.h>
#include <robin_hood.h>

namespace PS_AGONY2D
{
	struct PersistentContactData
	{
		Real normalImpulseAccumulator = 0;
		Real tangentImpulseAccumulator = 0;
	};

	// bodyA/bodyB are the OWNING bodies of colliderA/colliderB, resolved once at generation
	// time so downstream solvers (which only ever act on bodies) never re-resolve them.
	struct BodyCollisionData
	{
		ColliderDenseIndex colliderA, colliderB;
		ObjectDenseIndex bodyA, bodyB;
		Vec2 normal;
		Real penetration;
		uint32_t contactCount;
		Vec2 contactPoints[2];
		uint32_t contactIds[2];
		std::array<PersistentContactData, 2> persistentContactData{};

		BodyCollisionData() = default;

		BodyCollisionData(
			ColliderDenseIndex colliderA, ColliderDenseIndex colliderB,
			ObjectDenseIndex bodyA, ObjectDenseIndex bodyB,
			Vec2 normal,
			Real depth,
			uint32_t contactCount,
			Vec2 contactPoint1, Vec2 contactPoint2,
			uint32_t contactId1, uint32_t contactId2
		) :
			colliderA(colliderA), colliderB(colliderB),
			bodyA(bodyA), bodyB(bodyB),
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

		static constexpr size_t BODY_TYPE_COUNT = static_cast<size_t>(BodyType::COUNT);

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

		struct ContactCache
		{
			// Open-addressing table: uint64 collider-pair key -> index into previousCollisionData.
			// Rebuilt once per frame, never erased from, load factor <= 0.5, linear probing.
			static constexpr uint64_t EMPTY = ~0ull;
			static constexpr uint32_t NONE = ~0u;

			struct Slot { uint64_t key; uint32_t val; uint32_t pad; };

			CacheLineAlignedVector<Slot> slots;
			size_t mask = 0;
			uint32_t shift = 63;
			size_t capacity = 0;

			static uint64_t makeKey(ColliderDenseIndex a, ColliderDenseIndex b) noexcept
			{
				static_assert(sizeof(ColliderDenseIndex) <= 4);
				return (uint64_t(a) << 32) | uint64_t(b);
			}

			size_t slotOf(uint64_t key) const noexcept
			{
				return size_t((key * 0x9E3779B97F4A7C15ull) >> shift);
			}

			bool empty() const noexcept { return capacity == 0; }

			void reset(size_t maxEntries)
			{
				if (maxEntries == 0) { capacity = 0; return; }
				const size_t c = std::bit_ceil(std::max<size_t>(maxEntries * 2, 16));
				if (slots.size() < c) slots.resize(c);
				std::memset(slots.data(), 0xFF, c * sizeof(Slot));
				capacity = c;
				mask = c - 1;
				shift = 64 - unsigned(std::countr_zero(c));
			}

			// Duplicate key: last write wins.
			void insert(uint64_t key, uint32_t val) noexcept
			{
				Slot* s = slots.data();
				size_t i = slotOf(key);
				while (true)
				{
					if (s[i].key == EMPTY || s[i].key == key) { s[i].key = key; s[i].val = val; return; }
					i = (i + 1) & mask;
				}
			}

			uint32_t find(uint64_t key) const noexcept
			{
				const Slot* s = slots.data();
				size_t i = slotOf(key);
				while (true)
				{
					if (s[i].key == key) return s[i].val;
					if (s[i].key == EMPTY) return NONE;
					i = (i + 1) & mask;
				}
			}

			void prefetch(uint64_t key) const noexcept
			{
				_mm_prefetch(reinterpret_cast<const char*>(slots.data() + slotOf(key)), _MM_HINT_T0);
			}

			size_t memoryUsage() const { return slots.capacity() * sizeof(Slot); }
		};

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(
			std::span<const ObjectPair>, CacheLineAlignedVector<BodyCollisionData>&
			);

		// Static fields.

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		// Fields.

		CacheLineAlignedVector<BodyCollisionData> allCollisionData;
		CacheLineAlignedVector<ChunkData> chunks;

		// Previous frame's results (owner of warm-start data) + lookup table into them.
		CacheLineAlignedVector<BodyCollisionData> previousCollisionData;
		ContactCache contactCache;

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

		void updatePersistentContactData();

		// 'deletedColliders' is the collider swap-remove log (same shape as ObjectDeletion,
		// but every index in it is a collider index, not a body index).
		void remapPersistentContactData(std::span<const ObjectDeletion> deletedColliders);

		void clearData();

		std::span<BodyCollisionData> getBodyCollisionData() noexcept { return allCollisionData; }
		std::span<const BodyCollisionData> getBodyCollisionData() const noexcept { return allCollisionData; }

		size_t getMemoryUsage() const;
	private:
		void rebuildContactCache();
		void findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs);
		void findCollisionsMultiThreaded(std::span<const ObjectPair> colliderPairs, size_t maxWorkerCount);

		// Appends results to 'out' and applies warm-starting to the appended range only.
		void processPairs(std::span<const ObjectPair> pairs, ChunkData& chunkData, CacheLineAlignedVector<BodyCollisionData>& out);

		void movePreviousContactData(size_t firstResult, std::span<BodyCollisionData> out);

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