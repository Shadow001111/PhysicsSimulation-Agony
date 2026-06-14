#pragma once
#include "EcstasyCore/Threading/ThreadPool.h"

namespace PS_AGONY::Threading
{
	const uint32_t MAX_THREADS_ALLOWED = 0;

	using ThreadPool = Ecstasy::Threading::ThreadPool;

	ThreadPool& getGlobalThreadPool();
}
