#pragma once
#include "EcstasyCore/Threading/ThreadPool.h"

namespace PS_AGONY::Threading
{
	using ThreadPool = Ecstasy::Threading::ThreadPool;

	ThreadPool& getGlobalThreadPool();
}
