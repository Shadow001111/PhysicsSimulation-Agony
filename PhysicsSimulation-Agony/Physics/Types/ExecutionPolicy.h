#pragma once
#include <cstdint>

namespace PS_AGONY2D
{
	enum class ExecutionPolicy : uint8_t
	{
		Auto,
		ForceSingleThreaded,
		ForceMultiThreaded
	};
}