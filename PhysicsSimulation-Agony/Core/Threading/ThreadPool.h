#pragma once
#include "ChaseLevQueue.h"

#include <thread>
#include <future>
#include <atomic>
#include <latch>
#include <iostream>

#include "Core/TracyProfiler.h"

namespace Core::Threading
{
    class ThreadPool;  // forward declaration

    struct alignas(64) WorkerThread
    {
        ChaseLevQueue tasks{ 1024 * 64 };
        std::thread thread;
        size_t index;
        std::atomic<bool> stop{ false };

        WorkerThread(size_t idx) : index(idx) {}

        void run(ThreadPool* pool);
    };

    class WorkerThreadContainer
    {
        using Allocator = std::allocator<WorkerThread>;
        Allocator allocator;
        size_t threadCount = 0;
        WorkerThread* storage = nullptr;
    public:
        WorkerThreadContainer() noexcept = default;

        explicit WorkerThreadContainer(size_t count) :
            threadCount(count),
            storage(std::allocator_traits<Allocator>::allocate(allocator, count))
        {
            size_t index = 0;
            try
            {
                // Construct new workers.
                for (; index < count; index++)
                {
                    new (storage + index) WorkerThread(index);
                }
            }
            catch (...)
            {
                // Destroy already constructed workers and free memory.
                for (size_t i = 0; i < index; i++)
                {
                    storage[i].~WorkerThread();
                }
                std::allocator_traits<Allocator>::deallocate(allocator, storage, count);
                throw;
            }
        }

        // Non‑copyable, non‑movable - the container owns the storage.
        WorkerThreadContainer(const WorkerThreadContainer&) = delete;
        WorkerThreadContainer& operator=(const WorkerThreadContainer&) = delete;

        WorkerThreadContainer(WorkerThreadContainer&& other) :
            allocator(std::move(other.allocator)),
            threadCount(other.threadCount),
            storage(other.storage)
        {
            other.threadCount = 0;
            other.storage = nullptr;
        }

        WorkerThreadContainer& operator=(WorkerThreadContainer&& other)
        {
            if (this != &other)
            {
                // Destroy current workers and free storage.
                if (storage)
                {
                    destroy();
                }
                // Transfer ownership.
                threadCount = other.threadCount;
                storage = other.storage;
                allocator = std::move(other.allocator);
                other.threadCount = 0;
                other.storage = nullptr;
            }
            return *this;
        }

        ~WorkerThreadContainer()
        {
            destroy();
        }

        void destroy()
        {
            if (storage)
            {
                for (size_t i = 0; i < threadCount; i++)
                {
                    storage[i].~WorkerThread();
                }
                std::allocator_traits<Allocator>::deallocate(allocator, storage, threadCount);
                storage = nullptr;
            }
        }

        // Direct access to the workers.
        WorkerThread& operator[](size_t i) { return storage[i]; }
        const WorkerThread& operator[](size_t i) const { return storage[i]; }

        size_t getThreadCount() const noexcept { return threadCount; }
    };

    class ThreadPool
    {
        friend WorkerThread;

        alignas(64) std::atomic<uint32_t> workVersion{ 0 };

        alignas(64) WorkerThreadContainer workers;

        alignas(64) std::atomic<size_t> nextWorker{ 0 };

        alignas(64) std::atomic<size_t> pendingTaskCount{ 0 };


        void onTaskComplete();

        ChaseLevQueue& getWorkerQueue(size_t idx) { return workers[idx].tasks; }
    public:
        explicit ThreadPool(size_t numThreads = 0);
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        template<class F, class... Args>
        void enqueue(F&& f, Args&&... args);

        template<class F, class... Args>
        auto enqueueFuture(F&& f, Args&&... args) -> std::future<typename std::invoke_result_t<F, Args...>>;

        void enqueueBulk(std::vector<Task>& tasks);

        void shutdown();
        size_t getThreadCount() const noexcept { return workers.getThreadCount(); }
        size_t getPendingTasks() const noexcept { return pendingTaskCount.load(std::memory_order_relaxed); }
    };

    template<class F, class... Args>
    inline void ThreadPool::enqueue(F&& f, Args&&... args)
    {
        TRACY_SCOPE_N("ThreadPool::enqueue");

        // Choose a worker in round-robin order.
        size_t idx;
        {
            TRACY_SCOPE_N("Get next worker id");
            idx = nextWorker.fetch_add(1, std::memory_order_relaxed) % workers.getThreadCount();
        }
        WorkerThread& worker = workers[idx];

        {
            TRACY_SCOPE_N("Read stop");
            if (worker.stop.load(std::memory_order_relaxed)) [[unlikely]]
                throw std::runtime_error("enqueue on stopped ThreadPool");
        }
        
        Task task = [f = std::forward<F>(f),
            ...args = std::forward<Args>(args)]() mutable {
            std::invoke(std::move(f), std::move(args)...);
            };

        // The queue has a fixed capacity; spin until there is room.
        {
            TRACY_SCOPE_N("Push");
            while (!worker.tasks.try_push(std::move(task))) [[unlikely]]
            {
                TRACY_SCOPE_N("Spin");
                TracyMessage("Can't push!", 11);
                std::this_thread::yield();
            }
        }
        {
            TRACY_SCOPE_N("Add pending task count");
            pendingTaskCount.fetch_add(1, std::memory_order_relaxed);
        }
        {
            TRACY_SCOPE_N("Add work version");
            workVersion.fetch_add(1, std::memory_order_release);
        }
        {
            TRACY_SCOPE_N("Notify");
            workVersion.notify_one(); // TODO: Maybe add 'sleepingWorkerCount' and call only when its not zero.
        }
    }

    template<class F, class... Args>
    inline auto ThreadPool::enqueueFuture(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result_t<F, Args...>>
    {
        TRACY_SCOPE_N("ThreadPool::enqueueFuture");

        const size_t idx = nextWorker.fetch_add(1, std::memory_order_relaxed) % workers.getThreadCount();
        WorkerThread& worker = workers[idx];

        if (worker.stop.load(std::memory_order_relaxed)) [[unlikely]]
            throw std::runtime_error("enqueue on stopped ThreadPool");

        using return_type = typename std::invoke_result_t<F, Args...>;
        auto packaged = std::packaged_task<return_type()>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        std::future<return_type> result = packaged.get_future();

        Task task = [t = std::move(packaged)]() mutable { t(); };

        while (!worker.tasks.try_push(std::move(task)))
        {
            std::this_thread::yield();
        }

        pendingTaskCount.fetch_add(1, std::memory_order_relaxed);
        
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_one();
        return result;
    }



    // Parallel for loop using the thread pool. Splits the range [begin, end) into chunks and enqueues them.
    template<typename Func>
    void parallelFor(ThreadPool& pool, size_t begin, size_t end, Func&& func)
    {
        constexpr size_t LOAD_BALANCING_FACTOR = 4;

        const size_t range = end - begin;
        if (range == 0) return;

        // Create more chunks than workers for better load balancing.
        const size_t numWorkers = pool.getThreadCount();
        const size_t maxChunkCount = numWorkers * LOAD_BALANCING_FACTOR;
        const size_t chunkSize = (range + maxChunkCount - 1) / maxChunkCount;

        // Compute how many chunks we will actually enqueue.
        const size_t actualChunkCount = (range + chunkSize - 1) / chunkSize;

        std::latch latch(actualChunkCount);

        for (size_t chunkStart = begin; chunkStart < end; chunkStart += chunkSize)
        {
            const size_t chunkEnd = std::min(chunkStart + chunkSize, end);
            pool.enqueue([&func, chunkStart, chunkEnd, &latch]()
                {
                    for (size_t i = chunkStart; i < chunkEnd; i++)
                        func(i);
                    latch.count_down();
                });
        }

        latch.wait(); // Wait for all chunks to complete
    }
} // namespace Core::Threading