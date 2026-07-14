#pragma once
#include "GlmTypes.h"
#include "ObjectSoA.h"
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

	struct BodyCollisionData
	{
		BodyIndex bodyA, bodyB;
		Vec2 normal;
		Real penetration;
		uint32_t contactCount;
		Vec2 contactPoints[2];
		uint32_t contactIds[2];
		mutable std::array<PersistentContactData, 2> persistentContactData{};

		BodyCollisionData() = default;

		BodyCollisionData(
			BodyIndex bodyA, BodyIndex bodyB,
			Vec2 normal,
			Real depth,
			uint32_t contactCount,
			Vec2 contactPoint1, Vec2 contactPoint2,
			uint32_t contactId1, uint32_t contactId2
		) :
			bodyA(bodyA), bodyB(bodyB), normal(normal), penetration(depth), contactCount(contactCount)
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
			SymmetricMatrix<std::vector<BodyPair>, BODY_TYPE_COUNT> pairs;
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

		struct BodyPairKey
		{
			BodyIndex bodyA;
			BodyIndex bodyB;

			bool operator==(const BodyPairKey& other) const noexcept
			{
				return bodyA == other.bodyA && bodyB == other.bodyB;
			}
		};

		struct BodyPairKeyHasher
		{
			size_t operator()(const BodyPairKey& key) const noexcept
			{
				constexpr uint64_t addConst = 0x9e3779b97f4a7c15;
				uint64_t h = (uint64_t)key.bodyA + addConst;
				h ^= (uint64_t)key.bodyB + addConst + (h << 6) + (h >> 2);
				return h;
			}
		};

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(
			const std::vector<BodyPair>&, std::vector<BodyCollisionData>&
			);

		// Static fields.

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		// Fields.

		std::vector<BodyCollisionData> allCollisionData;
		std::vector<ChunkData> chunks;

		robin_hood::unordered_flat_map<BodyPairKey, CachedContactPair, BodyPairKeyHasher> previousContactDataContainer;

		// SoA data viewers.
		BodySoAViewer bodies;
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
			const CircleSoAViewer& circles,
			const BoxSoAViewer& boxes,
			const PolygonSoAViewer& polygons
		);

		const std::vector<BodyCollisionData>& findCollisions(const std::vector<BodyPair>& bodyPairs, ExecutionPolicy executionPolicy = ExecutionPolicy::Standard);

		void updatePersistentContactData();
		void remapPersistentContactData(const std::vector<BodyDeletion>& deletions);

		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return allCollisionData; }

		size_t getMemoryUsage() const;
	private:
		void findCollisionsSingleThreaded(const std::vector<BodyPair>& bodyPairs);
		void findCollisionsMultiThreaded(const std::vector<BodyPair>& bodyPairs);

		void processPairs(const std::vector<BodyPair>& pairs, ChunkData& chunkData);

		void collisionCircleCircle(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
	};
}