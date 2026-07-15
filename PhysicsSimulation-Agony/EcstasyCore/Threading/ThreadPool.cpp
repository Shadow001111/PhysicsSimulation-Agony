#include "ThreadPool.h"
#include "ThreadUtils.h"

#include "../TracyProfiler.h"

#include <string>
#include <iostream>
#include <bit>
#include <immintrin.h>

namespace Ecstasy::Threading
{
    ThreadPool::ThreadPool(size_t numThreads, CoreMode coreMode)
    {
        if (numThreads == 0)
        {
            const size_t minThreads = 1;
            size_t availableThreadCount = std::thread::hardware_concurrency();
            numThreads = std::max<size_t>(availableThreadCount, minThreads);
        }

        // Note: For now, we are gonna clamp thread count here. Probably for ever.
        if (coreMode == CoreMode::PerfomanceCores)
        {
            auto pCoreMask = getPcoreAffinityMask();
            numThreads = std::min<size_t>(numThreads, std::popcount(pCoreMask));

            workers = WorkerThreadContainer(numThreads);
            for (size_t i = 0; i < numThreads; i++)
            {
                const int32_t pinIndex = std::countr_zero(pCoreMask);
                pCoreMask &= pCoreMask - 1;

                WorkerThread& worker = workers[i];
                worker.thread = std::thread(&WorkerThread::run, &worker, this, pinIndex);
            }
        }
        else
        {
            workers = WorkerThreadContainer(numThreads);
            for (size_t i = 0; i < numThreads; i++)
            {
                WorkerThread& worker = workers[i];
                worker.thread = std::thread(&WorkerThread::run, &worker, this, WorkerThread::INVALID_THREAD_PIN_INDEX);
            }
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
        const size_t base = taskCount / workerCount;
        const size_t remainder = taskCount % workerCount;

        const size_t startWorkerOffset = nextWorker.fetch_add(remainder, std::memory_order_relaxed) % workerCount;

        size_t taskOffset = 0; // Current position in the input tasks array

        // Assign contiguous blocks to workers in round-robin order starting from startWorkerOffset.
        {
            TRACY_SCOPE_N("Push tasks");
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
        }

        // Update global counters and notify waiting threads.
        queuedTaskCount.fetch_add(taskCount, std::memory_order_release);
        unfinishedTaskCount.fetch_add(taskCount, std::memory_order_release);

        workVersion.fetch_add(1, std::memory_order_release);
        {
            TRACY_SCOPE_N("Notify");
            workVersion.notify_all();
        }
    }

    void ThreadPool::shutdown()
    {
        // Wait for all pending tasks to complete.
        {
            size_t unfinishedCount;
            while (true)
            {
                unfinishedCount = unfinishedTaskCount.load(std::memory_order_acquire);
                if (unfinishedTaskCount == 0) break;
                unfinishedTaskCount.wait(unfinishedCount, std::memory_order_acquire);
            }
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

    void ThreadPool::onTaskClaimed()
    {
        queuedTaskCount.fetch_sub(1, std::memory_order_acq_rel);
    }

    void ThreadPool::onTaskComplete()
    {
        const size_t remaining = unfinishedTaskCount.fetch_sub(1, std::memory_order_acq_rel) - 1;
        if (remaining == 0)
        {
            unfinishedTaskCount.notify_all();
        }
    }


    void WorkerThread::run(ThreadPool* pool, int32_t pinIndex)
    {
        configureThread(pinIndex);

        const size_t workerCount = pool->getThreadCount();

        constexpr int MAX_SPIN_COUNT = 5000;

        //TracyMessage("Worker start", 12);
        while (!stop.load(std::memory_order_relaxed))
        {
            // Try to get task from queue.
            Task task;
            {
                //TRACY_SCOPE_NC("Try pop", Ecstasy::Color::Gray);
                task = tasks.steal();
            }
            if (task)
            {
                pool->onTaskClaimed();

                //TRACY_SCOPE_NC("Execute task", Ecstasy::Color::NavajoWhite);
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
                bool taskStolen = false;
                {
                    //TRACY_SCOPE_NC("Try steal", Ecstasy::Color::Gray);
                    for (size_t i = 1; i < workerCount; i++)
                    {
                        size_t victimIdx = (threadId + i) % workerCount;
                        ChaseLevQueue& victimQueue = pool->getWorkerQueue(victimIdx);
                        task = victimQueue.steal();
                        if (task)
                        {
                            pool->onTaskClaimed();
                            taskStolen = true;
                            break;
                        }
                    }
                }
                if (taskStolen)
                {
                    //TRACY_SCOPE_NC("Execute task", Ecstasy::Color::NavajoWhite);
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
            }

            // No work found.
            if (!task)
            {
                //TRACY_SCOPE_NC("Thread spin/sleep", Ecstasy::Color::Black);

                // Spin.
                bool returnToStart = false;
                {
                    for (int spin = 0; spin < MAX_SPIN_COUNT; spin++)
                    {
                        ECSTASY_SPIN_PAUSE();
                        if (pool->getQueuedTasks() > 0 || stop.load(std::memory_order_relaxed))
                        {
                            returnToStart = true;
                            break;
                        }
                    }
                }
                if (returnToStart) [[likely]] continue;

                // Wait for new tasks (or stop).
                {
                    uint32_t currentVersion = pool->workVersion.load(std::memory_order_acquire);
                    do
                    {
                        // Wait until 'workVersion' != 'currentVersion'.
                        pool->workVersion.wait(currentVersion, std::memory_order_acquire);
                        currentVersion = pool->workVersion.load(std::memory_order_acquire);
                    } while (pool->getQueuedTasks() == 0 && !stop.load(std::memory_order_relaxed));
                }
            }
        }
    }

    void WorkerThread::configureThread(int32_t pinIndex)
    {
        // Tracy thread name.
    #ifdef TRACY_ENABLE
        std::string threadName = "worker_" + std::to_string(threadId);
        tracy::SetThreadName(threadName.c_str());
    #endif

        // Pinning.
        if (pinIndex != INVALID_THREAD_PIN_INDEX)
        {
            pinCurrentThreadToCpu(pinIndex);
        }

        // Thread priority.
        setThreadPriorityToHighest();
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
        const size_t threadCount = pool.getThreadCount();
        return getChunkCountAndSize(threadCount, taskRange, loadBalancingFactor);
    }

    std::pair<size_t, size_t> ParallelForRangeExecutor::getChunkCountAndSize(size_t threadCount, size_t taskRange, size_t loadBalancingFactor)
    {
        const size_t maxChunkCount = threadCount * loadBalancingFactor;

        const size_t chunkSize = (taskRange + maxChunkCount - 1) / maxChunkCount;
        const size_t chunkCount = (taskRange + chunkSize - 1) / chunkSize;
        return { chunkCount, chunkSize };
    }
}