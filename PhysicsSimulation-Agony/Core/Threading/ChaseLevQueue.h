#pragma once
#include "Task.h"

#include "EcstasyCore/TracyProfiler.h"

#include <atomic>
#include <cstddef>
#include <utility>
#include <thread>

namespace Core::Threading
{
    class ChaseLevQueue
    {
        // Aligning by 64 to avoid false sharing of 'mTop' and 'mBottom' since they are being acessed by multiple threads.

        alignas(64) std::atomic<size_t> mTop = { 0 };
        const size_t mCapacity;
        const size_t mMask;
        Task* const mTasks = nullptr;
        std::atomic<bool>* const mBools = nullptr;
        alignas(64) std::atomic<size_t> mBottom = { 0 };
    private:
        static size_t roundUpToPowerOfTwo(size_t n) noexcept
        {
            size_t p = 1;
            while (p < n) p <<= 1;
            return p;
        }
    public:
        explicit ChaseLevQueue(size_t initialCapacity = 1024) :
            mTop(0),
            mCapacity(roundUpToPowerOfTwo(initialCapacity)),
            mMask(mCapacity - 1),
            mTasks(new Task[mCapacity]()),
            mBools(new std::atomic<bool>[mCapacity]()),
            mBottom(0)
        {}

        ~ChaseLevQueue()
        {
            delete[] mTasks;
            delete[] mBools;
        }

        ChaseLevQueue(const ChaseLevQueue&) = delete;
        ChaseLevQueue& operator=(const ChaseLevQueue&) = delete;
        ChaseLevQueue(ChaseLevQueue&&) = delete;
        ChaseLevQueue& operator=(ChaseLevQueue&&) = delete;

        // Producer.
        bool try_push(Task&& task)
        {
            TRACY_SCOPE_N("Queue push");

            size_t top, bottom;
            {
                TRACY_SCOPE_N("Read top and bottom");
                bottom = mBottom.load(std::memory_order_relaxed);
                top = mTop.load(std::memory_order_acquire);
            }
            {
                TRACY_SCOPE_N("Check size");

                const size_t size = bottom - top;
                if (size >= mCapacity)
                {
                    return false; // Queue is full.
                }
            }

            // Compute slot index.
            const size_t slotIndex = bottom & mMask;

            // Wait until the stealer has released the slot.
            {
                TRACY_SCOPE_N("Spin");
                while (mBools[slotIndex].load(std::memory_order_acquire))
                    std::this_thread::yield();
            }

            // Move task.
            {
                TRACY_SCOPE_N("Move task");
                mTasks[slotIndex] = std::move(task);
            }

            // Set occupied flag to true.
            {
                TRACY_SCOPE_N("Set flag");
                mBools[slotIndex].store(true, std::memory_order_release);
            }
            
            // Advance bottom.
            {
                TRACY_SCOPE_N("Advance bottom");
                mBottom.store(bottom + 1, std::memory_order_release);
            }
            return true;
        }

        // No 'pop' method.

        // Consumer.
        Task steal()
        {
            TRACY_SCOPE_N("Queue steal");
            while (true)
            {
                size_t top = mTop.load(std::memory_order_acquire);
                const size_t bottom = mBottom.load(std::memory_order_acquire);
                if (bottom <= top)
                {
                    return {}; // Empty.
                }

                // Check if occupied.
                const bool isOccupied = mBools[top & mMask].load(std::memory_order_acquire);
                if (!isOccupied)
                {
                    continue;
                }

                if (mTop.compare_exchange_strong(top, top + 1,
                    std::memory_order_seq_cst,
                    std::memory_order_relaxed))
                {
                    // Get task.
                    Task task = std::move(mTasks[top & mMask]);

                    // Set as unoccupied.
                    mBools[top & mMask].store(false, std::memory_order_release);

                    return task;
                }
                // CAS failed, retry.
            }
        }
    };
}