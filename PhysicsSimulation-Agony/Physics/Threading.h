#pragma once
#include "Core/Threading/ThreadPool.h"

namespace PS_AGONY
{
	Core::Threading::ThreadPool& getGlobalThreadPool()
	{
		static Core::Threading::ThreadPool pool;
		return pool;
	}
}
