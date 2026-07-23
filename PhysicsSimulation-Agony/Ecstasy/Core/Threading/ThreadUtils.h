#pragma once
#include <cstdint>

namespace Ecstasy::Core::Threading
{
	void pinCurrentThreadToCpu(size_t cpuIndex);

	uint64_t getPcoreAffinityMask();

	void setThreadPriorityToHighest();
}