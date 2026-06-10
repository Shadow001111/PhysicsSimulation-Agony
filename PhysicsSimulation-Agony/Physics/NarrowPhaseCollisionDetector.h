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

		SymmetricMatrix<std::vector<BodyPair>*, BODY_TYPE_COUNT> bodyPairVectorMatrix;
		std::vector<BodyPair> circleCirclePairs;
		std::vector<BodyPair> circleBoxPairs;
		std::vector<BodyPair> boxBoxPairs;

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
		void collisionCircleCircle();
		void collisionCircleBox();
		void collisionCirclePolygon();
		void collisionBoxBox();
		void collisionBoxPolygon();
		void collisionPolygonPolygon();
	};
}
