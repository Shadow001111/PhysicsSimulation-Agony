#pragma once
#include "ChaseLevQueue.h"

#include <thread>
#include <future>
#include <atomic>
#include <latch>
#include <functional>

#include "../TracyProfiler.h"

namespace Ecstasy::Threading
{
    class ThreadPool;  // forward declaration

    class alignas(64) WorkerThread
    {
    public:
        static constexpr int32_t INVALID_THREAD_PIN_INDEX = 0;

        ChaseLevQueue tasks{ 1024 * 64 };
        std::thread thread;
        size_t threadId;
        std::atomic<bool> stop{ false };

        WorkerThread(size_t idx) : threadId(idx) {}

        void run(ThreadPool* pool, int32_t pinIndex);
    private:
        void configureThread(int32_t pinIndex);
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

    // TODO: Must use multiproducer queue, because enqueue can come from different threads.
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
        enum class CoreMode
        {
            AnyCores,
            PerfomanceCores
            // EfficiencyCores maybe?
        };

        explicit ThreadPool(size_t numThreads = 0, CoreMode coreMode = CoreMode::AnyCores);
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        template<class F, class... Args>
        void enqueue(F&& f, Args&&... args);

        template<class F, class... Args>
        auto enqueueFuture(F&& f, Args&&... args) -> std::future<typename std::invoke_result_t<F, Args...>>;

        void enqueueBulk(std::vector<Task>& tasks);

        template<typename ResultType>
        inline std::vector<std::future<ResultType>> enqueueFutureBulk(std::vector<std::function<ResultType()>> funcs);

        void shutdown();
        size_t getThreadCount() const noexcept { return workers.getThreadCount(); }
        size_t getPendingTasks() const noexcept { return pendingTaskCount.load(std::memory_order_relaxed); }
    };

    template<class F, class... Args>
    inline void ThreadPool::enqueue(F&& f, Args&&... args)
    {
        TRACY_SCOPE_N("ThreadPool::enqueue");

        // Choose a worker in round-robin order.
        size_t idx = nextWorker.fetch_add(1, std::memory_order_relaxed) % workers.getThreadCount();
        WorkerThread& worker = workers[idx];

        if (worker.stop.load(std::memory_order_relaxed)) [[unlikely]]
            throw std::runtime_error("enqueue on stopped ThreadPool");
        
        Task task = [f = std::forward<F>(f),
            ...args = std::forward<Args>(args)]() mutable {
            std::invoke(std::move(f), std::move(args)...);
            };

        // Push task.
        worker.tasks.push(std::move(task));

        pendingTaskCount.fetch_add(1, std::memory_order_relaxed);
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_one();
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

        // Push task.
        worker.tasks.push(std::move(task));

        pendingTaskCount.fetch_add(1, std::memory_order_relaxed);
        
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_one();
        return result;
    }

    template<typename ResultType>
    inline std::vector<std::future<ResultType>> ThreadPool::enqueueFutureBulk(
        std::vector<std::function<ResultType()>> funcs)
    {
        TRACY_SCOPE_N("ThreadPool::enqueueFutureBulk");

        const size_t taskCount = funcs.size();
        if (taskCount == 0) return {};

        const size_t workerCount = workers.getThreadCount();
        if (workerCount == 0)
            throw std::runtime_error("enqueueFutureBulk on ThreadPool with no workers");

        const size_t startOffset = nextWorker.fetch_add(1, std::memory_order_relaxed) % workerCount;

        // Build packaged tasks first and harvest all futures before any moves occur.
        std::vector<std::packaged_task<ResultType()>> packagedTasks;
        std::vector<std::future<ResultType>> futures;
        packagedTasks.reserve(taskCount);
        futures.reserve(taskCount);
        for (auto& func : funcs)
        {
            packagedTasks.emplace_back(std::move(func));
            futures.push_back(packagedTasks.back().get_future());
        }

        // Distribute contiguous blocks to worker queues in round-robin order.
        const size_t base = taskCount / workerCount;
        const size_t remainder = taskCount % workerCount;

        size_t taskOffset = 0;
        for (size_t i = 0; i < workerCount; i++)
        {
            const size_t workerIdx = (startOffset + i) % workerCount;
            const size_t count = base + (i < remainder ? 1 : 0);
            if (count == 0) continue;

            WorkerThread& worker = workers[workerIdx];
            if (worker.stop.load(std::memory_order_relaxed))
                throw std::runtime_error("enqueueFutureBulk on stopped ThreadPool");

            // Wrap each packaged_task in a void Task by moving it into a lambda.
            // The packaged_task is move-only, so it must be captured by move.
            std::vector<Task> genericTasks;
            genericTasks.reserve(count);
            for (size_t t = 0; t < count; ++t)
            {
                genericTasks.emplace_back(
                    [pkg = std::move(packagedTasks[taskOffset + t])]() mutable {
                        pkg();
                    });
            }
            worker.tasks.bulk_push(genericTasks.data(), count);

            taskOffset += count;
        }

        pendingTaskCount.fetch_add(taskCount, std::memory_order_release);
        workVersion.fetch_add(1, std::memory_order_release);
        workVersion.notify_all();

        return futures;
    }

    class ParallelForRangeExecutor
    {
        //static thread_local std::vector<Task> tasks;

        ThreadPool& pool;
        size_t begin, end;

        size_t chunkCount = 0, chunkSize = 0;
    public:
        ParallelForRangeExecutor(ThreadPool& pool, size_t begin, size_t end, size_t loadBalancingFactor);

        void execute(std::function<void(size_t, size_t, size_t)>&& func);

        size_t getChunkCount() const noexcept { return chunkCount; }

        static std::pair<size_t, size_t> getChunkCountAndSize(ThreadPool& pool, size_t taskRange, size_t loadBalancingFactor);
        static std::pair<size_t, size_t> getChunkCountAndSize(size_t threadCount, size_t taskRange, size_t loadBalancingFactor);
    };
}