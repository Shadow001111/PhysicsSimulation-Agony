#include "Threading.h"

namespace PS_AGONY::Threading
{
	ThreadPool& getGlobalThreadPool()
	{
		static Ecstasy::Core::Threading::ThreadPool pool(
			std::thread::hardware_concurrency(), Ecstasy::Core::Threading::ThreadPool::CoreMode::AnyCores
		);
		return pool;
	}
}
