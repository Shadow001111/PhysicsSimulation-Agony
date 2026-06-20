#pragma once
#include <cstdint>

namespace Ecstasy::Threading
{
	void pinCurrentThreadToCpu(size_t cpuIndex);

	uint64_t getPcoreAffinityMask();
}