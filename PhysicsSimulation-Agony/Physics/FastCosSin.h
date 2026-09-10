#pragma once
#include "Types.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <utility>

namespace PS_AGONY::FastCosSin
{
	using RealSimd = Ecstasy::Core::Simd<Real>;

	// Requirments to use these functions:
	// 1) Input pointers must be RealSimd aligned.
	// 2) Angles/rotations must be in range [0; 2pi)

	void order4Array(
		const Real* ECSTASY_RESTRICT inAngleArray,
		Real* ECSTASY_RESTRICT outCosArray,
		Real* ECSTASY_RESTRICT outSinArray,
		size_t size);

	void order4Simd(
		const RealSimd& inAngle,
		RealSimd& outCos,
		RealSimd& outSin);

	// Returns { cos, sin }.
	std::pair<Real, Real> order4Scalar(const Real inAngle);
}
