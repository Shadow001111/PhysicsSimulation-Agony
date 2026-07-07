#pragma once
#include "ScratchBuffer.h"
#include "Types.h"

namespace PS_AGONY
{
	struct SharedScratchBuffers
	{
		ScratchBuffer<64> buffer0;

		size_t getMemoryUsage() const noexcept
		{
			size_t total = 0;
			total += buffer0.getCapacity();
			return total;
		}
	};
}