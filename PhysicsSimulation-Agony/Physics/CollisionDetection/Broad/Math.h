#pragma once
#include <cstdint>
#include <bit>

namespace PS_AGONY::BroadPhaseInternal
{
	inline constexpr uint64_t ceilLog2(uint64_t n, uint64_t groupSize = 1)
	{
		if (n <= groupSize || groupSize == 0) return 0;
		return std::bit_width((n - 1) / groupSize);
	}
}