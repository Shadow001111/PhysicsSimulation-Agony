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
		Vec2 contact1, contact2;
	};

	class NarrowPhaseCollisionDetector
	{
	};
}
