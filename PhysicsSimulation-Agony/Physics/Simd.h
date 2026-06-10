#pragma once
#include "EcstasyCore/Simd.h"

namespace PS_AGONY
{
	template<typename T, size_t Bits = Ecstasy::kSimdBits>
	using Simd = Ecstasy::Simd<T, Bits>;
}