#pragma once
#include "GlmTypes.h"
#include "ObjectSoA.h"
#include "SoA/ColliderSoA.h"
#include "SymmetricMatrix.h"

#include <atomic>
#include <robin_hood.h>
#include <array>

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
		mutable std::array<PersistentContactData, 2> persistentContactData{};

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
			SymmetricMatrix<std::vector<ObjectPair>, BODY_TYPE_COUNT> pairs; // Pairs of COLLIDER indices.
			std::vector<BodyCollisionData> results;
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
			const std::vector<ObjectPair>&, std::vector<BodyCollisionData>&
			);

		// Static fields.

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		// Fields.

		std::vector<BodyCollisionData> allCollisionData;
		std::vector<ChunkData> chunks;

		robin_hood::unordered_flat_map<ColliderPairKey, CachedContactPair, ColliderPairKeyHasher> previousContactDataContainer;

		// SoA data viewers.
		BodySoAViewer bodies;
		ColliderSoAViewer colliders;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		PolygonSoAViewer polygons;
	public:
		static constexpr bool ENABLE_WARM_STARTING = true;

		enum class ExecutionPolicy
		{
			Standard,
			ForceSingleThreaded,
			ForceMultiThreaded
		};

		NarrowPhaseCollisionDetector();
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
			const PolygonSoAViewer& polygons
		);

		// 'colliderPairs' are COLLIDER index pairs (as produced by BroadPhaseCollisionDetector).
		const std::vector<BodyCollisionData>& findCollisions(const std::vector<ObjectPair>& colliderPairs, ExecutionPolicy executionPolicy = ExecutionPolicy::Standard);

		// 'collidersToCheck' and the first element of each 'outColliding' pair are COLLIDER indices.
		void findCollisionsInCircle(
			const std::vector<ObjectIndex>& collidersToCheck,
			Vec2 pos,
			Real radius,
			std::vector<std::pair<ObjectIndex, Real>>& outColliding
		) const;

		void updatePersistentContactData();

		// 'deletedColliders' is the collider swap-remove log (same shape as ObjectDeletion,
		// but every index in it is a collider index, not a body index).
		void remapPersistentContactData(const std::vector<ObjectDeletion>& deletedColliders);

		void clearData();

		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return allCollisionData; }

		size_t getMemoryUsage() const;
	private:
		void findCollisionsSingleThreaded(const std::vector<ObjectPair>& colliderPairs);
		void findCollisionsMultiThreaded(const std::vector<ObjectPair>& colliderPairs);

		void processPairs(const std::vector<ObjectPair>& pairs, ChunkData& chunkData);

		void collisionCircleCircle(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(const std::vector<ObjectPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
	};
}