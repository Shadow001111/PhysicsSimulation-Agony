#pragma once
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/SoA/BodySoA.h"

namespace PS_AGONY2D::Integrator
{
	enum class IntegratorType : uint8_t
	{
		SemiImplicit,
		VelocityVerlet,
		COUNT
	};

	struct IntegrationSettings
	{
		Vec2 gravity{ 0.0, -9.81 };
		Real angularVelocityDamping = 1.0; // Per second.
		IntegratorType integratorType = IntegratorType::SemiImplicit;
	};

	void integrateVelocities_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);
	void integrateKinematics_SemiImplicit(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void integratePhase1_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);
	void integratePhase2_VelocityVerlet(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void dampAngularVelocities(BodySoA& bodies, Real deltaTime, const IntegrationSettings& settings);

	void computeRotationCosSin(BodySoA& bodies);

	inline const char* integratorTypeToString(IntegratorType d)
	{
		switch (d)
		{
		case IntegratorType::SemiImplicit:   return "SemiImplicit";
		case IntegratorType::VelocityVerlet: return "VelocityVerlet";
		}
	}
}

