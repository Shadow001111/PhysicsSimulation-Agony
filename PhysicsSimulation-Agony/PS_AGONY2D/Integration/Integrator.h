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

	struct Settings
	{
		Vec2 gravity{ 0.0, -9.81 };
		Real angularVelocityDamping = 1.0; // Per second.
		IntegratorType integratorType = IntegratorType::VelocityVerlet;
	};

	void integrateVelocities_SemiImplicit(BodySoA& bodies, IndexRange range, Real deltaTime, const Settings& settings);
	void integrateKinematics_SemiImplicit(BodySoA& bodies, IndexRange range, Real deltaTime, const Settings& settings);

	void integratePhase1_VelocityVerlet(BodySoA& bodies, IndexRange range, Real deltaTime, const Settings& settings);
	void integratePhase2_VelocityVerlet(BodySoA& bodies, IndexRange range, Real deltaTime, const Settings& settings);

	void dampAngularVelocities(BodySoA& bodies, IndexRange range, Real deltaTime, const Settings& settings);

	void computeRotationCosSin(BodySoA& bodies, IndexRange range);

	inline const char* integratorTypeToString(IntegratorType d)
	{
		switch (d)
		{
		case IntegratorType::SemiImplicit:   return "SemiImplicit";
		case IntegratorType::VelocityVerlet: return "VelocityVerlet";
		default:							 return nullptr;
		}
	}
}

