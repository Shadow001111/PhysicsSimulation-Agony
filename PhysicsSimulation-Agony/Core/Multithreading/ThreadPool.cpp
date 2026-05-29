#include "ThreadPool.h"

#include "Core/FileLogger.h"
#include "Core/TracyProfiler.h"

ThreadPool::ThreadPool(int numThreads)
{
    if (numThreads == 0)
    {
        const int minThreads = 1;
        const int maxThreads = 999;

        numThreads = std::thread::hardware_concurrency();
        numThreads = std::clamp(numThreads - 1, minThreads, maxThreads);
    }

    workers.reserve(numThreads);
    for (size_t i = 0; i < numThreads; i++)
    {
		workers.emplace_back(&ThreadPool::workerThreadFunc, this, i);
    }
}

ThreadPool::~ThreadPool()
{
    shutdown();
}

void ThreadPool::enqueueBulk(std::vector<Task> tasks)
{
    TRACY_SCOPE_N("ThreadPool::enqueueBulk");

    // If the pool is stopped, reject the task
    if (context.stop.load(std::memory_order_relaxed))
    {
        throw std::runtime_error("enqueue on stopped ThreadPool");
    }

    // Push the tasks into the queue
    const size_t count = tasks.size();
    {
        std::lock_guard lock(context.queueMutex);
        for (auto& task : tasks)
            context.tasks.push(std::move(task));
    }
    context.newTaskCondition.notify_all();
    taskTotalCount.fetch_add(count, std::memory_order_relaxed);
}

void ThreadPool::shutdown()
{
    {
        std::unique_lock lock(context.queueMutex);

        context.completionCondition.wait(lock, [this]
            {
                return context.tasks.empty() && context.activeTaskCount.load(std::memory_order_relaxed) == 0;
            });

        context.stop.store(true, std::memory_order_release);
    }
    context.newTaskCondition.notify_all();

    for (auto& worker : workers)
    {
        worker.join();
    }

    workers.clear();
}

void ThreadPool::workerThreadFunc(size_t threadIndex)
{
    std::string threadName = std::to_string(threadIndex);
    tracy::SetThreadName(threadName.c_str());

    while (true)
    {
        Task task;

        // Try to get task
        {
            std::unique_lock lock(context.queueMutex);
            if (context.tasks.empty())
            {
                // No job found, wait for a task to appear or a shutdown
                context.newTaskCondition.wait(lock, [this] { return !context.tasks.empty() || context.stop.load(std::memory_order_relaxed); });

                if (context.stop.load(std::memory_order_relaxed)) [[unlikely]]
                {
                    break;
                }
            }

            task = std::move(context.tasks.front());
            context.tasks.pop();
            context.activeTaskCount.fetch_add(1, std::memory_order_relaxed);
        }

        if (task)
        {
            try
            {
                task();
            }
            catch (const std::exception& e)
            {
                FileLogger logger("log/warnings.txt");
                logger.add(std::string("Exception in thread pool worker thread: ") + e.what());
            }
            catch (...)
            {
                FileLogger logger("log/warnings.txt");
                logger.add("Unknown exception in thread pool worker thread");
            }

            const size_t oldTaskCount = context.activeTaskCount.fetch_sub(1, std::memory_order_relaxed);
            if (oldTaskCount <= 1)
            {
                std::lock_guard lock(context.queueMutex);
                if (context.tasks.empty())
                {
                    context.completionCondition.notify_all();
                }
            }
        }
    }
}

ThreadPool& ParallelUtils::getGlobalThreadPool()
{
    static ThreadPool pool;
    return pool;
}

std::pair<size_t, size_t> ParallelUtils::calculateBatchCountAndSize(size_t totalItems, size_t threadCount, size_t minBatchSize)
{
	const size_t maxBatchCount = (totalItems + minBatchSize - 1) / minBatchSize;
    const size_t targetBatchCount = threadCount * 8;
    const size_t batchCount = std::max(size_t(1), std::min(targetBatchCount, maxBatchCount));

    const size_t batchSize = (totalItems + batchCount - 1) / batchCount;
    const size_t actualBatchCount = (totalItems + batchSize - 1) / batchSize;

    return { actualBatchCount, batchSize };
}
