#pragma once
#include "Physics/ObjectSoA.h"
#include "Physics/SymmetricMatrix.h"
#include "Physics/Types/Types.h"
#include "Physics/Types/ExecutionPolicy.h"

#include <atomic>
#include <array>
#include <span>
#include <robin_hood.h>

namespace PS_AGONY
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
		ColliderIndex colliderA, colliderB;
		ObjectIndex bodyA, bodyB;
		Vec2 normal;
		Real penetration;
		uint32_t contactCount;
		Vec2 contactPoints[2];
		uint32_t contactIds[2];
		std::array<PersistentContactData, 2> persistentContactData{};

		BodyCollisionData() = default;

		BodyCollisionData(
			ColliderIndex colliderA, ColliderIndex colliderB,
			ObjectIndex bodyA, ObjectIndex bodyB,
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

		struct CachedContactPair
		{
			uint32_t contactIds[2] = { uint32_t(-1), uint32_t(-1) };
			PersistentContactData contactData[2];
		};

		// Keyed by COLLIDER pair, not body pair: two bodies can now touch through more
		// than one simultaneous collider pair, and warm-starting must not conflate them.
		struct ColliderPairKey
		{
			ColliderIndex colliderA;
			ColliderIndex colliderB;

			bool operator==(const ColliderPairKey& other) const noexcept
			{
				return colliderA == other.colliderA && colliderB == other.colliderB;
			}
		};

		struct ColliderPairKeyHasher
		{
			size_t operator()(const ColliderPairKey& key) const noexcept
			{
				constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
				uint64_t h = (uint64_t)key.colliderA + addConst;
				h ^= (uint64_t)key.colliderB + addConst + (h << 6) + (h >> 2);
				return h;
			}
		};

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(
			std::span<const ObjectPair>, CacheLineAlignedVector<BodyCollisionData>&
			);

		// Static fields.

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		// Fields.

		CacheLineAlignedVector<BodyCollisionData> allCollisionData;
		CacheLineAlignedVector<ChunkData> chunks;

		robin_hood::unordered_flat_map<ColliderPairKey, CachedContactPair, ColliderPairKeyHasher> previousContactDataContainer;

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
			std::span<const ObjectIndex> collidersToCheck,
			Vec2 pos,
			Real radius,
			std::vector<std::pair<ObjectIndex, Real>>& outColliding
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
		void findCollisionsSingleThreaded(std::span<const ObjectPair> colliderPairs);
		void findCollisionsMultiThreaded(std::span<const ObjectPair> colliderPairs, size_t maxWorkerCount);

		// Appends results to 'out' and applies warm-starting to the appended range only.
		void processPairs(std::span<const ObjectPair> pairs, ChunkData& chunkData, CacheLineAlignedVector<BodyCollisionData>& out);

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