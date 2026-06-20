#include "Threading.h"

namespace PS_AGONY::Threading
{
	ThreadPool& getGlobalThreadPool()
	{
		static Ecstasy::Threading::ThreadPool pool(
			std::thread::hardware_concurrency(), Ecstasy::Threading::ThreadPool::CoreMode::AnyCores
		);
		return pool;
	}
}
