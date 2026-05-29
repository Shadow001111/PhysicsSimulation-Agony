#include "ProcessingFence.h"


AtomicWaitFence::~AtomicWaitFence()
{
    processing.store(false, std::memory_order_release);
    processing.notify_all();
}

void AtomicWaitFence::lock()
{
    bool expected = false;
    // Try to flip false -> true; if it fails, sleep until value changes.
    while (!processing.compare_exchange_weak(
        expected, true,
        std::memory_order_acquire,
        std::memory_order_relaxed))
    {
        processing.wait(true, std::memory_order_relaxed);
        expected = false;
    }
}

void AtomicWaitFence::unlock()
{
    processing.store(false, std::memory_order_release);
    processing.notify_one();
}

bool AtomicWaitFence::try_lock()
{
    bool expected = false;
    return processing.compare_exchange_strong(
        expected, true,
        std::memory_order_acquire,
        std::memory_order_relaxed);
}


SemaphoreFence::~SemaphoreFence()
{
    unlock();
}

void SemaphoreFence::lock()
{
    sem.acquire();
    processing.store(true, std::memory_order_release);
}

void SemaphoreFence::unlock()
{
    processing.store(false, std::memory_order_release);
    sem.release();
}

bool SemaphoreFence::try_lock()
{
    bool acquired = sem.try_acquire();
    if (acquired) processing.store(true, std::memory_order_release);
    return acquired;
}