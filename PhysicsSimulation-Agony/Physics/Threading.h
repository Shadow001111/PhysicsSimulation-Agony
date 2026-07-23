#pragma once
#include "Ecstasy/Core/Threading/ThreadPool.h"

namespace PS_AGONY::Threading
{
	using ThreadPool = Ecstasy::Core::Threading::ThreadPool;

	ThreadPool& getGlobalThreadPool();
}
