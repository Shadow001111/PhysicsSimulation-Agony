#include "ThreadPool.h"

#include "../TracyProfiler.h"

#include <string>
#include <iostream>

#define NOMINMAX
#define NOGDI
#define NOCRYPT
#define NOUSER
#define NOKERNEL
#define NOMCX
#define NOSERVICE
#define NOCOMM
#define NOSOUND
#define NOHELP
#define NOMB
#define NOWH
#define NOGDI
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <windows.h>

namespace Ecstasy::Threading
{
    static std::string threadPriorityToString(int priority)
    {
        if (priority == THREAD_PRIORITY_ERROR_RETURN)
        {
            return "Error: " + std::to_string(GetLastError());
        }

        switch (priority)
        {
        case THREAD_PRIORITY_IDLE:               return "IDLE (-15)";
        case THREAD_PRIORITY_LOWEST:             return "LOWEST (-2)";
        case THREAD_PRIORITY_BELOW_NORMAL:       return "BELOW_NORMAL (-1)";
        case THREAD_PRIORITY_NORMAL:             return "NORMAL (0)";
        case THREAD_PRIORITY_ABOVE_NORMAL:       return "ABOVE_NORMAL (+1)";
        case THREAD_PRIORITY_HIGHEST:            return "HIGHEST (+2)";
        case THREAD_PRIORITY_TIME_CRITICAL:      return "TIME_CRITICAL (+15)";
        default:
            // Some unexpected value (should not happen with valid handles).
            return "Unknown (" + std::to_string(priority) + ")";
        }
    }


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

        Task* taskSource = tasks.data();
        const size_t taskCount = tasks.size();
        if (taskCount == 0) [[unlikely]] return;

        const size_t workerCount = workers.getThreadCount();
        if (workerCount == 0) [[unlikely]]
            throw std::runtime_error("enqueueBulk on ThreadPool with no workers");

        // Rotate the starting worker for load balancing across multiple bulk calls.
        const size_t startWorkerOffset = nextWorker.fetch_add(1, std::memory_order_relaxed) % workerCount;

        const size_t base = taskCount / workerCount;
        const size_t remainder = taskCount % workerCount;

        size_t taskOffset = 0; // Current position in the input tasks array

        // Assign contiguous blocks to workers in round-robin order starting from startWorkerOffset.
        for (size_t i = 0; i < workerCount; i++)
        {
            const size_t workerIdx = (startWorkerOffset + i) % workerCount;
            const size_t count = base + (i < remainder ? 1 : 0);
            if (count == 0) continue;

            WorkerThread& worker = workers[workerIdx];

            if (worker.stop.load(std::memory_order_relaxed)) [[unlikely]]
                throw std::runtime_error("enqueueBulk on stopped ThreadPool");

            // Push.
            worker.tasks.bulk_push(taskSource + taskOffset, count);

            taskOffset += count;
        }

        // Update global counters and notify waiting threads.
        pendingTaskCount.fetch_add(taskCount, std::memory_order_release);
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_all();
    }

    void ThreadPool::shutdown()
    {
        // Wait for all pending tasks to complete.
        {
            size_t count;
            while ((count = pendingTaskCount.load(std::memory_order_acquire)) != 0)
                pendingTaskCount.wait(count, std::memory_order_acquire);
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
        configureThread();

        const size_t workerCount = pool->getThreadCount();

        constexpr int MAX_SPIN_COUNT = 5000;

        TracyMessage("Worker start", 12);
        while (!stop.load(std::memory_order_relaxed))
        {
            // Possibly sleep now, so it won't sleep during executing task.
            std::this_thread::yield();

            // Try to get task from queue.
            Task task = tasks.steal();
            if (task)
            {
                try
                {
                    task();
                }
                catch (const std::exception& e)
                {
                    std::cerr << "Worker thread exception: " << e.what() << "\n";
                }
                pool->onTaskComplete();
                continue;
            }

            // Try stealing from other threads.
            {
                for (size_t i = 1; i < workerCount; i++)
                {
                    size_t victimIdx = (index + i) % workerCount;
                    ChaseLevQueue& victimQueue = pool->getWorkerQueue(victimIdx);
                    task = victimQueue.steal();
                    if (task)
                    {
                        try
                        {
                            task();
                        }
                        catch (const std::exception& e)
                        {
                            std::cerr << "Worker thread exception: " << e.what() << "\n";
                        }
                        pool->onTaskComplete();
                        break;
                    }
                }
            }

            // No work found.
            if (!task)
            {
                // Spin.
                //TracyMessage("Spin", 4);
                bool returnToStart = false;
                for (int spin = 0; spin < MAX_SPIN_COUNT; spin++)
                {
                    if (pool->getPendingTasks() > 0 || stop.load(std::memory_order_relaxed))
                    {
                        returnToStart = true;
                        break;
                    }
                    std::this_thread::yield();
                }
                if (returnToStart) [[likely]] continue;

                TracyMessage("Sleep", 5);

                //Wait for new tasks (or stop).
                uint32_t currentVersion = pool->workVersion.load(std::memory_order_acquire);
                do
                {
                    // Wait until 'workVersion' != 'currentVersion'.
                    pool->workVersion.wait(currentVersion, std::memory_order_acquire);
                    currentVersion = pool->workVersion.load(std::memory_order_acquire);
                } while (!stop.load(std::memory_order_relaxed) && pool->getPendingTasks() == 0);
                TracyMessage("Awake", 5);
            }
        }
    }

    void WorkerThread::configureThread()
    {
        // Tracy thread name.
    #ifdef TRACY_ENABLE
        std::string threadName = "worker_" + std::to_string(index);
        tracy::SetThreadName(threadName.c_str());
    #endif

        // Priority.
        {
            //auto handle = thread.native_handle();

            //SetThreadPriority(handle, THREAD_PRIORITY_NORMAL);
        }
    }


    void parallelForRange(ThreadPool& pool, size_t begin, size_t end, std::function<void(size_t, size_t)>&& func, size_t loadBalancingFactor)
    {
        static thread_local std::vector<Task> tasks;

        TRACY_SCOPE_N("Parallel for range");

        const size_t range = end - begin;
        if (range == 0) return;

        // Create more chunks than workers for better load balancing.
        const size_t numWorkers = pool.getThreadCount();
        const size_t maxChunkCount = numWorkers * loadBalancingFactor;
        const size_t chunkSize = (range + maxChunkCount - 1) / maxChunkCount;

        // Compute how many chunks we will actually enqueue.
        const size_t actualChunkCount = (range + chunkSize - 1) / chunkSize;

        std::latch latch(actualChunkCount);

        /*for (size_t chunkStart = begin; chunkStart < end; chunkStart += chunkSize)
        {
            const size_t chunkEnd = std::min(chunkStart + chunkSize, end);
            pool.enqueue([&func, chunkStart, chunkEnd, &latch]()
                {
                    func(chunkStart, chunkEnd);
                    latch.count_down();
                });
        }*/

        tasks.clear();
        {
            TRACY_SCOPE_N("Reserve place tasks");
            tasks.reserve(actualChunkCount);
        }
        {
            TRACY_SCOPE_N("Create tasks");
            for (size_t chunkStart = begin; chunkStart < end; chunkStart += chunkSize)
            {
                const size_t chunkEnd = std::min(chunkStart + chunkSize, end);
                tasks.emplace_back([&func, chunkStart, chunkEnd, &latch]()
                    {
                        func(chunkStart, chunkEnd);
                        latch.count_down();
                    });
            }
        }

        pool.enqueueBulk(tasks);

        latch.wait(); // Wait for all chunks to complete
    }


    ParallelForRangeExecutor::ParallelForRangeExecutor(ThreadPool& pool, size_t begin, size_t end, size_t loadBalancingFactor) :
        pool(pool),
        begin(begin),
        end(end)
    {
        const size_t range = end - begin;
        if (range == 0) return;

        auto chunkCountAndSize = getChunkCountAndSize(pool, range, loadBalancingFactor);

        chunkCount = chunkCountAndSize.first;
        chunkSize = chunkCountAndSize.second;
    }

    void ParallelForRangeExecutor::execute(std::function<void(size_t, size_t, size_t)>&& func)
    {
        TRACY_SCOPE_N("ParallelForRangeExecutor::execute");

        static thread_local std::vector<Task> tasks;

        std::latch latch(chunkCount);

        tasks.clear();
        tasks.reserve(chunkCount);
        {
            TRACY_SCOPE_N("Create tasks");
            size_t chunkId = 0;
            for (size_t chunkStart = begin; chunkStart < end; chunkStart += chunkSize)
            {
                const size_t chunkEnd = std::min(chunkStart + chunkSize, end);
                tasks.emplace_back([&func, chunkStart, chunkEnd, chunkId, &latch]()
                    {
                        func(chunkStart, chunkEnd, chunkId);
                        latch.count_down();
                    });
                chunkId++;
            }
        }

        pool.enqueueBulk(tasks);

        latch.wait(); // Wait for all chunks to complete
    }

    std::pair<size_t, size_t> ParallelForRangeExecutor::getChunkCountAndSize(ThreadPool& pool, size_t taskRange, size_t loadBalancingFactor)
    {
        const size_t numWorkers = pool.getThreadCount();
        const size_t maxChunkCount = numWorkers * loadBalancingFactor;

        const size_t chunkSize = (taskRange + maxChunkCount - 1) / maxChunkCount;
        const size_t chunkCount = (taskRange + chunkSize - 1) / chunkSize;
        return { chunkCount, chunkSize };
    }
}