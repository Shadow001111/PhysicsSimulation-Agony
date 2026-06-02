#pragma once
#include "GlmTypes.h"
#include "BodySoAViewer.h"

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
		using CollisionFunc = void (NarrowPhaseCollisionDetector::*)(BodyIndex, BodyIndex);

		static const CollisionFunc collisionFunctions[static_cast<size_t>(BodyType::COUNT)][static_cast<size_t>(BodyType::COUNT)];

		std::vector<BodyCollisionData> narrowCollisionData;

		// SoA data viewers.
		BodySoAViewer bodies;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;
	public:
		NarrowPhaseCollisionDetector() = default;
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
		void collisionCircleCircle(BodyIndex indexA, BodyIndex indexB);
		void collisionCircleBox(BodyIndex indexA, BodyIndex indexB);
		void collisionCirclePolygon(BodyIndex indexA, BodyIndex indexB);
		void collisionBoxBox(BodyIndex indexA, BodyIndex indexB);
		void collisionBoxPolygon(BodyIndex indexA, BodyIndex indexB);
		void collisionPolygonPolygon(BodyIndex indexA, BodyIndex indexB);
	};
}
