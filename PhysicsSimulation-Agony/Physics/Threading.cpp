#include "Threading.h"

#include <atomic>
#include <cstdlib>
#include <iostream>

namespace PS_AGONY::Threading
{
	namespace
	{
		std::atomic<IThreadingBackend*> backendPtr{ nullptr };
	}

	void setThreadingBackend(IThreadingBackend* backend) noexcept
	{
		backendPtr.store(backend, std::memory_order_release);
	}

	IThreadingBackend& getThreadingBackend()
	{
		IThreadingBackend* backend = backendPtr.load(std::memory_order_acquire);
		if (!backend) [[unlikely]]
		{
			std::cerr << "[PS_AGONY][getThreadingBackend]: No threading backend set. Call Threading::setThreadingBackend() first.\n";
			std::abort();
		}
		return *backend;
	}
}