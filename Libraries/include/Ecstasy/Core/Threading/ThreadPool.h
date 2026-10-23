#pragma once
#include "ChaseLevQueue.h"

#include <thread>
#include <future>
#include <atomic>
#include <latch>
#include <span>
#include <functional>

#include "ThreadUtils.h"
#include <vector>
#include <stdexcept>

namespace Ecstasy::Core::Threading
{
    class ThreadPool;  // forward declaration

    // Passed to onThreadStart. Indices are unique among this pool's workers.
    struct WorkerInfo
    {
        size_t globalIndex = 0;                 // Worker index in the pool.
        CoreType type = CoreType::Unknown;
        size_t typeIndex = 0;                   // Index among this pool's workers of the same type, from 0.
        const CpuInfo* cpu = nullptr;           // nullptr if unpinned.
    };

    class alignas(64) WorkerThread
    {
    public:
        ChaseLevQueue tasks{ 1024 * 64 };
        std::thread thread;
        size_t threadId;
        WorkerInfo info;
        std::atomic<bool> stop{ false };
        alignas(64) std::atomic<uint32_t> wakeVersion{ 0 };

        WorkerThread(size_t idx) : threadId(idx) {}

        void run(ThreadPool* pool);
    private:
        void configureThread(ThreadPool* pool);
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

        // Non-copyable, non-movable - the container owns the storage.
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

    struct ThreadPoolConfig
    {
        size_t numThreads = 0;                      // 0 = one worker per selected CPU. Clamped to the selected CPU count.
        CoreSet coreSet = CoreSet::All;
        SmtMode smtMode = SmtMode::Logical;
        bool pin = true;                            // Pin worker i to one CPU of the ordered list.
        std::vector<uint32_t> affinity;             // Explicit CPU ids (group * 64 + index). Overrides coreSet/smtMode.
        std::function<void(const WorkerInfo&)> onThreadStart;  // Runs on each worker after pin/priority, before its loop.
        ThreadPriority priority = ThreadPriority::Normal;
        uint32_t spinCount = 5000;                  // Idle spins before a worker sleeps.
        size_t reservedCpus = 0;                    // Taken from the front of the ordered CPU list, not given to workers.
        // With SMT, the sibling of a reserved CPU may still run a worker and
        // shares execution units with main. Set reserveWholeCore to avoid that.
        // At least one CPU always stays with the workers.
        bool reserveWholeCore = false;              // Also exclude SMT siblings (same coreId) of reserved CPUs from workers.
    };

    // TODO: Must use multiproducer queue, because enqueue can come from different threads.
    class ThreadPool
    {
        friend WorkerThread;

        alignas(64) WorkerThreadContainer workers;
        alignas(64) std::atomic<size_t> nextWorker{ 0 };

        // Tasks that are still sitting in queues and can be stolen/picked up.
        alignas(64) std::atomic<size_t> queuedTaskCount{ 0 };

        // Tasks that have not finished yet (queued + running).
        alignas(64) std::atomic<size_t> unfinishedTaskCount{ 0 };

        // Read-only after construction. pinnedCpus[i] = CPU of worker i (empty = unpinned).
        alignas(64) std::vector<CpuInfo> pinnedCpus;
        std::vector<CpuInfo> reservedCpus;  // Read-only after construction. For the main thread; no worker uses these.
        std::function<void(const WorkerInfo&)> onThreadStart;
        ThreadPriority priority;
        uint32_t spinCount;

        void onTaskClaimed();
        void onTaskComplete();

        ChaseLevQueue& getWorkerQueue(size_t idx) { return workers[idx].tasks; }

        static void wake(WorkerThread& worker) noexcept
        {
            worker.wakeVersion.fetch_add(1, std::memory_order_release);
            worker.wakeVersion.notify_one();
        }

        // Empty targets = every worker.
        size_t workerAt(std::span<const uint32_t> targets, size_t slot) const noexcept
        {
            return targets.empty() ? slot : targets[slot] % workers.getThreadCount();
        }

        // Wakes only the workers that received tasks. Call after counters are updated.
        void wakeDistributed(std::span<const uint32_t> targets, size_t startSlot, size_t base, size_t remainder);
    public:
        explicit ThreadPool(const ThreadPoolConfig& config = {});
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        template<class F, class... Args>
        void enqueue(F&& f, Args&&... args);

        template<class F, class... Args>
        auto enqueueFuture(F&& f, Args&&... args) -> std::future<typename std::invoke_result_t<F, Args...>>;

        // 'targets': worker indices to distribute to and wake. Empty = all workers.
        void enqueueBulk(std::vector<Task>& tasks, std::span<const uint32_t> targets = {});

        template<typename ResultType>
        inline std::vector<std::future<ResultType>> enqueueFutureBulk(std::vector<std::function<ResultType()>> funcs, std::span<const uint32_t> targets = {});

        void shutdown();
        size_t getThreadCount() const noexcept { return workers.getThreadCount(); }
        size_t getQueuedTasks() const noexcept { return queuedTaskCount.load(std::memory_order_relaxed); }
        size_t getPendingTasks() const noexcept { return unfinishedTaskCount.load(std::memory_order_relaxed); }

        std::span<const CpuInfo> getPinnedCpus() const noexcept { return pinnedCpus; }
        std::span<const CpuInfo> getReservedCpus() const noexcept { return reservedCpus; }
        // Valid after construction (written before the worker threads start).
        const WorkerInfo& getWorkerInfo(size_t i) const noexcept { return workers[i].info; }
        void printTopology() const { printCpuTopology(pinnedCpus); }
    };

    template<class F, class... Args>
    inline void ThreadPool::enqueue(F&& f, Args&&... args)
    {

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

        queuedTaskCount.fetch_add(1, std::memory_order_release);
        unfinishedTaskCount.fetch_add(1, std::memory_order_release);

        wake(worker);
    }

    template<class F, class... Args>
    inline auto ThreadPool::enqueueFuture(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result_t<F, Args...>>
    {

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

        queuedTaskCount.fetch_add(1, std::memory_order_release);
        unfinishedTaskCount.fetch_add(1, std::memory_order_release);

        wake(worker);
        return result;
    }

    template<typename ResultType>
    inline std::vector<std::future<ResultType>> ThreadPool::enqueueFutureBulk(
        std::vector<std::function<ResultType()>> funcs, std::span<const uint32_t> targets)
    {

        const size_t taskCount = funcs.size();
        if (taskCount == 0) return {};

        const size_t workerCount = workers.getThreadCount();
        if (workerCount == 0)
            throw std::runtime_error("enqueueFutureBulk on ThreadPool with no workers");

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
        const size_t slotCount = targets.empty() ? workerCount : targets.size();
        const size_t base = taskCount / slotCount;
        const size_t remainder = taskCount % slotCount;

        const size_t startOffset = nextWorker.fetch_add(remainder, std::memory_order_relaxed) % slotCount;

        size_t taskOffset = 0;
        for (size_t i = 0; i < slotCount; i++)
        {
            const size_t workerIdx = workerAt(targets, (startOffset + i) % slotCount);
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

        queuedTaskCount.fetch_add(taskCount, std::memory_order_release);
        unfinishedTaskCount.fetch_add(taskCount, std::memory_order_release);

        wakeDistributed(targets, startOffset, base, remainder);

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