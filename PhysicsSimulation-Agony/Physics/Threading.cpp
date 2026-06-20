#include "Threading.h"

namespace PS_AGONY::Threading
{
	ThreadPool& getGlobalThreadPool()
	{
		static Ecstasy::Threading::ThreadPool pool(std::min<uint32_t>(
			std::thread::hardware_concurrency(),
			MAX_THREADS_ALLOWED
		));
		return pool;
	}
}
