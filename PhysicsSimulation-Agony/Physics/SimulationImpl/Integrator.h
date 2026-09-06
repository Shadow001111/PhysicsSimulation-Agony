#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/SoA/BodySoA.h"

namespace PS_AGONY
{
	class Integrator
	{
	public:
		struct IntegrationSettings
		{
			Real timeScale = 1.0;
			Vec2 gravity{ 0.0, -9.81 };
			Real angularVelocityDamping = 1.0; // Per second.
		};

		static void integrateVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

		static void integrateKinematics(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

		static void wrapRotation(BodySoA& bodies);

		static void computeRotationCosSin(BodySoA& bodies);
	};
}

