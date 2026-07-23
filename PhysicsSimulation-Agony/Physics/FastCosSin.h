#pragma once
#include "Types.h"

#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY::FastCosSin
{
	// Requirments to use these functions:
	// 1) Input pointers must be RealSimd aligned.
	// 2) Angles/rotations must be in range [0; 2pi)

	void order4CosSin(
		const Real* ECSTASY_RESTRICT inAngleArray,
		Real* ECSTASY_RESTRICT outCosArray,
		Real* ECSTASY_RESTRICT outSinArray,
		size_t size);
}
