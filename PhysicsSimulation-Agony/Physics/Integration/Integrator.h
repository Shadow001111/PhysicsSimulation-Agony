#pragma once
#include "Physics/Types/Types.h"
#include "Physics/SoA/BodySoA.h"

namespace PS_AGONY2D::Integrator
{
	struct IntegrationSettings
	{
		Vec2 gravity{ 0.0, -9.81 };
		Real angularVelocityDamping = 1.0; // Per second.
	};

	void integrateVelocities_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);
	void integrateKinematics_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void integratePhase1_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);
	void integratePhase2_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void dampAngularVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void computeRotationCosSin(BodySoA& bodies);
}

