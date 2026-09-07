#pragma once
#include "Physics/GlmTypes.h"
#include "Physics/SoA/BodySoA.h"

namespace PS_AGONY::Integrator
{
	struct IntegrationSettings
	{
		Vec2 gravity{ 0.0, -9.81 };
		Real angularVelocityDamping = 1.0; // Per second.
	};

	void integrateVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void integrateKinematics(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void wrapRotation(BodySoA& bodies);

	void computeRotationCosSin(BodySoA& bodies);
}

