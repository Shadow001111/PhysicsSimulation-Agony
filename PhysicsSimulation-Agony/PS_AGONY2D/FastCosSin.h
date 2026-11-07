#pragma once
#include "Types/Types.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <utility>

namespace PS_AGONY2D::FastCosSin
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	// Requirment to use these functions: Angles/rotations must be in range [0; 2pi).

	void order5Array(
		const Real* ECSTASY_RESTRICT inAngleArray,
		Real* ECSTASY_RESTRICT outCosArray,
		Real* ECSTASY_RESTRICT outSinArray,
		size_t size);

	void order5Simd(
		const RealSimd& inAngle,
		RealSimd& outCos,
		RealSimd& outSin);

	// Returns { cos, sin }.
	std::pair<Real, Real> order5Scalar(const Real inAngle);
}
