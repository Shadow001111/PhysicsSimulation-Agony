#pragma once
#include "GlmTypes.h"
#include "BodySoAViewer.h"
#include "SymmetricMatrix.h"
#include "Threading.h"

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
		static constexpr bool USE_THREADING = true;

		static constexpr size_t BODY_TYPE_COUNT = static_cast<size_t>(BodyType::COUNT);

		using CollisionFunc = void(NarrowPhaseCollisionDetector::*)(size_t, size_t, std::vector<BodyCollisionData>&);

		struct alignas(64) CacheAlignedCollisionDataVector
		{
			std::vector<BodyCollisionData> vector;
		};

		static const SymmetricMatrix<CollisionFunc, BODY_TYPE_COUNT> collisionFuncs;

		SymmetricMatrix<std::vector<BodyPair>, BODY_TYPE_COUNT> bodyPairVectorMatrix;

		SymmetricMatrix<std::vector<CacheAlignedCollisionDataVector>, BODY_TYPE_COUNT> chunkedResults;
		std::vector<Ecstasy::Threading::Task> tasks;

		std::vector<BodyCollisionData> allCollisionData;

		// SoA data viewers.
		BodySoAViewer bodies;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;
	public:
		NarrowPhaseCollisionDetector();
		~NarrowPhaseCollisionDetector() = default;
		NarrowPhaseCollisionDetector(const NarrowPhaseCollisionDetector&) = default;
		NarrowPhaseCollisionDetector& operator=(const NarrowPhaseCollisionDetector&) = default;
		NarrowPhaseCollisionDetector(NarrowPhaseCollisionDetector&&) = default;
		NarrowPhaseCollisionDetector& operator=(NarrowPhaseCollisionDetector&&) = default;

		void setDataViewers(
			const BodySoAViewer& bodies,
			const CircleSoAViewer& circles,
			const BoxSoAViewer& boxes
		);

		const std::vector<BodyCollisionData>& findCollisions(const std::vector<BodyPair>& bodyPairs);

		size_t getMemoryUsage() const;
	private:
		void findCollisionsSingleThreaded();
		void findCollisionsMultiThreaded();

		void collisionCircleCircle(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCircleBox(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
		void collisionCirclePolygon(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxBox(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
		void collisionBoxPolygon(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
		void collisionPolygonPolygon(size_t startIndex, size_t endIndex, std::vector<BodyCollisionData>& outCollisionData);
	};
}
