#pragma once
#include <semaphore>
#include <atomic>

class NothingFence
{
public:
    NothingFence() = default;
    ~NothingFence() = default;
    NothingFence(const NothingFence&) = delete;
    NothingFence& operator=(const NothingFence&) = delete;
    NothingFence(NothingFence&&) = delete;
    NothingFence& operator=(NothingFence&&) = delete;

    void lock() {}
    void unlock() {}
    bool try_lock() { return true; }

    // Note: This is not thread-safe. It should be used only for quick checks, where it's not critical if it returns a slightly outdated value.
    [[nodiscard]] bool isLocked() const noexcept { return false; };
};

class AtomicWaitFence
{
    std::atomic<bool> processing = false;
public:
    AtomicWaitFence() = default;
    ~AtomicWaitFence();
    AtomicWaitFence(const AtomicWaitFence&) = delete;
    AtomicWaitFence& operator=(const AtomicWaitFence&) = delete;
    AtomicWaitFence(AtomicWaitFence&&) = delete;
    AtomicWaitFence& operator=(AtomicWaitFence&&) = delete;

    void lock();
    void unlock();
    bool try_lock();

    [[nodiscard]] bool isLocked() const noexcept { return processing.load(std::memory_order_acquire); };
};

class SemaphoreFence
{
    std::binary_semaphore sem{ 1 };   // 1 = available (not processing)
    std::atomic<bool> processing = false;
public:
    SemaphoreFence() = default;
    ~SemaphoreFence();
    SemaphoreFence(const SemaphoreFence&) = delete;
    SemaphoreFence& operator=(const SemaphoreFence&) = delete;
    SemaphoreFence(SemaphoreFence&&) = delete;
    SemaphoreFence& operator=(SemaphoreFence&&) = delete;

    void lock();
    void unlock();
    bool try_lock();

    [[nodiscard]] bool isLocked() const noexcept { return processing.load(std::memory_order_acquire); };
};

template <typename T>
class FenceGuard
{
    T& fence;
public:
    explicit FenceGuard(T& fence) :
        fence(fence)
    {
		fence.lock();
    }

    ~FenceGuard()
    {
		fence.unlock();
    }

    FenceGuard(const FenceGuard&) = delete;
    FenceGuard& operator=(const FenceGuard&) = delete;
    FenceGuard(FenceGuard&&) = delete;
    FenceGuard& operator=(FenceGuard&&) = delete;
};