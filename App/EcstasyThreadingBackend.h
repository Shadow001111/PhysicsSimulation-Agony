#pragma once
#include "Physics/Threading.h"
#include "Ecstasy/Core/Threading/ThreadPool.h"

namespace PS_AGONY::Threading
{
	class EcstasyThreadingBackend final : public IThreadingBackend
	{
		Ecstasy::Core::Threading::ThreadPool& pool;
	public:
		explicit EcstasyThreadingBackend(Ecstasy::Core::Threading::ThreadPool& poolIn) noexcept : pool(poolIn) {}

		size_t getThreadCount() const noexcept override { return pool.getThreadCount(); }

		void launch(size_t taskCount, TaskFn fn, void* context, std::span<const uint32_t> targets = {}) override
		{
			if (taskCount == 0) return;

			static thread_local std::vector<Ecstasy::Core::Threading::Task> tasks;
			tasks.clear();
			tasks.reserve(taskCount);
			for (size_t i = 0; i < taskCount; i++)
			{
				tasks.emplace_back([fn, context, i]() { fn(context, i); });
			}
			pool.enqueueBulk(tasks, targets);
		}
	};
}
