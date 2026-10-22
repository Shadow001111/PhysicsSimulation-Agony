#pragma once
#include <cstdint>

namespace PS_AGONY
{
	enum class ExecutionPolicy : uint8_t
	{
		Auto,
		ForceSingleThreaded,
		ForceMultiThreaded
	};
}