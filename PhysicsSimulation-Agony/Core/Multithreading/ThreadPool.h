#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <atomic>
#include <queue>
#include <latch>
#include <vector>

#include "Core/TracyProfiler.h"

#if defined(__cpp_lib_move_only_function) && __cpp_lib_move_only_function >= 202110L
#include <functional>
template <class F>
using move_only_function_impl = std::move_only_function<F>;
#else
#include "move_only_function.h"
template <class F>
using move_only_function_impl = move_only_function<F>;
#endif

class ThreadPool
{
public:
    using Task = move_only_function_impl<void()>;
private:
    using TaskQueue = std::queue<Task>;

    struct Context
    {
        TaskQueue tasks;
        TracyLockableN(std::mutex, queueMutex, "ThreadPool queue mutex");
        std::condition_variable_any newTaskCondition;
        std::condition_variable_any completionCondition;
        std::atomic<bool> stop{ false };
        std::atomic<size_t> activeTaskCount{ 0 };
    };

	Context context;
    std::vector<std::thread> workers;

    // Statistics
    std::atomic<size_t> taskTotalCount{ 0 };
public:
	ThreadPool(int numThreads = 0);
	~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    template<class F, class... Args>
    void enqueue(F&& f, Args&&... args);

	template<class F, class... Args>
    auto enqueueFuture(F&& f, Args&&... args) -> std::future<typename std::invoke_result_t<F, Args...>>;

    void enqueueBulk(std::vector<Task> tasks);

    void shutdown();

	//void waitForCompletion();
    size_t getThreadCount() const { return workers.size(); };
    size_t getTaskCount() const { return context.tasks.size(); }
    size_t getTaskTotalCount() const { return taskTotalCount.load(std::memory_order_relaxed); }
private:
	void workerThreadFunc(size_t threadIndex);
};

template<class F, class ...Args>
inline void ThreadPool::enqueue(F&& f, Args && ...args)
{
	TRACY_SCOPE_N("ThreadPool::enqueue");

    // If the pool is stopped, reject the task
    if (context.stop.load(std::memory_order_relaxed))
    {
        throw std::runtime_error("enqueue on stopped ThreadPool");
    }

    auto task = [f = std::forward<F>(f),
        ...args = std::forward<Args>(args)]() mutable {
        std::invoke(std::move(f), std::move(args)...);
        };

    // Push the task into the queue
    {
        std::lock_guard lock(context.queueMutex);
        context.tasks.push(std::move(task));
    }
    context.newTaskCondition.notify_one();
    taskTotalCount.fetch_add(1, std::memory_order_relaxed);
}

template<class F, class ...Args>
inline auto ThreadPool::enqueueFuture(F&& f, Args && ...args) -> std::future<typename std::invoke_result_t<F, Args...>>
{
    TRACY_SCOPE_N("ThreadPool::enqueueFuture");

    // If the pool is stopped, reject the task
    if (context.stop.load(std::memory_order_relaxed))
    {
        throw std::runtime_error("enqueue on stopped ThreadPool");
    }

    using return_type = typename std::invoke_result_t<F, Args...>;

    auto task = std::packaged_task<return_type()>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<return_type> res = task.get_future();
    
    {
        std::lock_guard lock(context.queueMutex);
        context.tasks.emplace([task = std::move(task)]() mutable { task(); });
    }
    context.newTaskCondition.notify_one();
    taskTotalCount.fetch_add(1, std::memory_order_relaxed);
    return res;
}

// Parallel execution utilities
class ParallelUtils
{
public:
    static ThreadPool& getGlobalThreadPool();

    template<typename Func>
    static void parallelFor(size_t start, size_t end, size_t minBatchSize, Func func);

    template<typename Container, typename Func>
    static void parallelForEach(Container& container, size_t minBatchSize, Func func);

    template<typename T, typename Func>
    static void parallelForEachInRange(T* array, size_t range, size_t minBatchSize, Func func);
private:
    static std::pair<size_t, size_t> calculateBatchCountAndSize(size_t totalItems, size_t threadCount, size_t minBatchSize);
};

template<typename Func>
void ParallelUtils::parallelFor(size_t start, size_t end, size_t minBatchSize, Func func)
{
    TRACY_SCOPE_N("ThreadPool::parallelFor");

    if (end <= start) return;

    const size_t totalItems = end - start;

    ThreadPool& pool = getGlobalThreadPool();

    const size_t numThreads = pool.getThreadCount();
	auto [batchCount, batchSize] = calculateBatchCountAndSize(totalItems, numThreads, minBatchSize);

    std::vector<ThreadPool::Task> tasks;
    tasks.reserve(batchCount);

    std::latch doneLatch(batchCount);

    for (size_t i = start; i < end; i += batchSize)
    {
        size_t batchEnd = std::min(i + batchSize, end);
        tasks.emplace_back([&, i, batchEnd]()
            {
                for (size_t j = i; j < batchEnd; j++)
                    func(j);
                doneLatch.count_down();
            });
    }

    pool.enqueueBulk(std::move(tasks));

    // Wait for all chunks to complete
	doneLatch.wait();
}

template<typename Container, typename Func>
void ParallelUtils::parallelForEach(Container& container, size_t minBatchSize, Func func)
{
    parallelFor(0, container.size(), minBatchSize, [&container, func](size_t i)
        {
            func(container[i]);
        });
}

template<typename T, typename Func>
void ParallelUtils::parallelForEachInRange(T* array, size_t range, size_t minBatchSize, Func func)
{
    parallelFor(0, range, minBatchSize, [&array, func](size_t i)
        {
            func(array[i]);
        });
}