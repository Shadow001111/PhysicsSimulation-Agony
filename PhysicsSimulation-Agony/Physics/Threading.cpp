#include "Threading.h"

namespace PS_AGONY
{
	Ecstasy::Threading::ThreadPool& getGlobalThreadPool()
	{
		static Ecstasy::Threading::ThreadPool pool;
		return pool;
	}
}
