#include "ThreadPool.h"

#include "Core/TracyProfiler.h"

#include <string>

namespace Core::Threading
{
    ThreadPool::ThreadPool(size_t numThreads)
    {
        if (numThreads == 0)
        {
            const size_t minThreads = 1;
            size_t availableThreadCount = std::thread::hardware_concurrency();
            numThreads = std::max(availableThreadCount, minThreads);
        }

        workers = WorkerThreadContainer(numThreads);
        for (size_t i = 0; i < numThreads; i++)
        {
            WorkerThread& worker = workers[i];
            worker.thread = std::thread(&WorkerThread::run, &worker, this);
        }
    }

    ThreadPool::~ThreadPool()
    {
        shutdown();
    }

    void ThreadPool::enqueueBulk(std::vector<Task>& tasks)
    {
        TRACY_SCOPE_N("ThreadPool::enqueueBulk");

        const size_t taskCount = tasks.size();
        if (taskCount == 0) [[unlikely]] return;

        const size_t workerCount = workers.getThreadCount();

        // Choose a worker in round-robin order.
        size_t startIdx;
        {
            TRACY_SCOPE_N("Get next worker id");
            startIdx = nextWorker.fetch_add(taskCount, std::memory_order_relaxed);
        }

        // Distribute tasks to worker queues.
        for (size_t i = 0; i < taskCount; i++)
        {
            const size_t workerIdx = (startIdx + i) % workerCount;
            WorkerThread& worker = workers[workerIdx];

            {
                TRACY_SCOPE_N("Read stop");
                if (worker.stop.load(std::memory_order_relaxed)) [[unlikely]]
                    throw std::runtime_error("enqueueBulk on stopped ThreadPool");
            }

            // Spin until the queue has room for this task.
            {
                TRACY_SCOPE_N("Push");
                while (!worker.tasks.try_push(std::move(tasks[i]))) [[unlikely]]
                {
                    TRACY_SCOPE_N("Spin");
                    TracyMessage("Can't push!", 11);
                    std::this_thread::yield();
                }
            }
        }

        {
            TRACY_SCOPE_N("Add pending task count");
            pendingTaskCount.fetch_add(taskCount, std::memory_order_release);
        }
        {
            TRACY_SCOPE_N("Add work version");
            workVersion.fetch_add(1, std::memory_order_release);
        }
        {
            TRACY_SCOPE_N("Notify");
            workVersion.notify_all();
        }
    }

    void ThreadPool::shutdown()
    {
        // Wait for all pending tasks to complete.
        while (pendingTaskCount.load(std::memory_order_acquire) != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Signal all workers to stop.
        const size_t threadCount = workers.getThreadCount();
        for (size_t i = 0; i < threadCount; i++)
        {
            WorkerThread& worker = workers[i];
            worker.stop.store(true, std::memory_order_release);
        }

        // Wake any workers that are sleeping.
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_all();

        // Join all threads.
        for (size_t i = 0; i < threadCount; i++)
        {
            WorkerThread& worker = workers[i];
            if (worker.thread.joinable())
                worker.thread.join();
        }

        workers.destroy();
    }

    void ThreadPool::onTaskComplete()
    {
        const size_t remaining = pendingTaskCount.fetch_sub(1, std::memory_order_acq_rel) - 1;
        if (remaining == 0)
        {
            pendingTaskCount.notify_all();
        }
    }

    void WorkerThread::run(ThreadPool* pool)
    {
        #ifdef TRACY_ENABLE
        std::string threadName = "worker_" + std::to_string(index);
        tracy::SetThreadName(threadName.c_str());
        #endif

        const size_t workerCount = pool->getThreadCount();

        constexpr int SPIN_COUNT = 1000;

        while (!stop.load(std::memory_order_relaxed))
        {
            // Try to get task from queue.
            Task task = tasks.steal();
            if (task)
            {
                task();
                pool->onTaskComplete();
                continue;
            }

            // Try stealing from other threads.
            for (size_t i = 1; i < workerCount; i++)
            {
                size_t victimIdx = (index + i) % workerCount;
                ChaseLevQueue& victimQueue = pool->getWorkerQueue(victimIdx);
                task = victimQueue.steal();
                if (task)
                {
                    task();
                    pool->onTaskComplete();
                    goto done;
                }
            }

            // No work found.
            if (!task)
            {
                // Spin.
                for (int spin = 0; spin < SPIN_COUNT; spin++)
                {
                    if (stop.load(std::memory_order_relaxed))
                        goto done;
                    if (pool->getPendingTasks() > 0)
                        goto done;
                    std::this_thread::yield();
                }

                //Wait for new tasks (or stop).
                uint32_t currentVersion = pool->workVersion.load(std::memory_order_acquire);
                do
                {
                    // Wait until 'workVersion' != 'currentVersion'.
                    pool->workVersion.wait(currentVersion, std::memory_order_acquire);
                    currentVersion = pool->workVersion.load(std::memory_order_acquire);
                } while (!stop.load(std::memory_order_relaxed) && pool->getPendingTasks() == 0);
            }
            done:
        }
    }
} // namespace Core::Threading