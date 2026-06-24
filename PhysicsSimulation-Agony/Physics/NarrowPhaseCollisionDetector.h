#pragma once
#include "GlmTypes.h"
#include "BodySoAViewer.h"
#include "SymmetricMatrix.h"

namespace PS_AGONY
{
	struct BodyCollisionData
	{
		BodyIndex bodyA, bodyB;
		Vec2 normal;
		Real depth;
		Vec2 contacts[2];
		uint32_t contactCount;

		BodyCollisionData() = default;

		BodyCollisionData(
			BodyIndex bodyA, BodyIndex bodyB,
			Vec2 normal,
			Real depth,
			Vec2 contact1, Vec2 contact2,
			uint32_t contactCount
		) :
			bodyA(bodyA), bodyB(bodyB), normal(normal), depth(depth), contactCount(contactCount)
		{
			contacts[0] = contact1;
			contacts[1] = contact2;
		}
	};

	class NarrowPhaseCollisionDetector
	{
		static constexpr size_t BODY_TYPE_COUNT = static_cast<size_t>(BodyType::COUNT);

		struct ChunkData
		{
			SymmetricMatrix<std::vector<BodyPair>, BODY_TYPE_COUNT> pairs;
			std::vector<BodyCollisionData> results;

			void clear()
			{
				for (auto& vec : pairs.getDirectAccess())
					vec.clear();
				results.clear();
			}
		};

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(
			const std::vector<BodyPair>&, std::vector<BodyCollisionData>&
			);

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		std::vector<BodyCollisionData> allCollisionData;
		std::vector<ChunkData> chunks;

		// SoA data viewers.
		BodySoAViewer bodies;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;
	public:
		NarrowPhaseCollisionDetector();
		~NarrowPhaseCollisionDetector() = default;
		NarrowPhaseCollisionDetector(const NarrowPhaseCollisionDetector&) = delete;
		NarrowPhaseCollisionDetector& operator=(const NarrowPhaseCollisionDetector&) = delete;
		NarrowPhaseCollisionDetector(NarrowPhaseCollisionDetector&&) = delete;
		NarrowPhaseCollisionDetector& operator=(NarrowPhaseCollisionDetector&&) = delete;

		void setDataViewers(
			const BodySoAViewer& bodies,
			const CircleSoAViewer& circles,
			const BoxSoAViewer& boxes
		);

		const std::vector<BodyCollisionData>& findCollisions(const std::vector<BodyPair>& bodyPairs);

		const std::vector<BodyCollisionData>& getBodyCollisionData() const noexcept { return allCollisionData; }

		size_t getMemoryUsage() const;
	private:
		void findCollisionsSingleThreaded(const std::vector<BodyPair>& bodyPairs);
		void findCollisionsMultiThreaded(const std::vector<BodyPair>& bodyPairs);

		void processPairs(const std::vector<BodyPair>& pairs, size_t start, size_t end, ChunkData& chunkData);

		void collisionCircleCircle(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(const std::vector<BodyPair>& pairs, std::vector<BodyCollisionData>& outCollisionData);
	};
}