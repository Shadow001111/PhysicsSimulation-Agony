#pragma once
#include "Task.h"

#include "../TracyProfiler.h"

#include <atomic>
#include <cstddef>
#include <utility>
#include <thread>

namespace Ecstasy::Threading
{
    class ChaseLevQueue
    {
        struct alignas(64) Slot
        {
            Task task;
            std::atomic<bool> flag;
        };

        // Aligning by 64 to avoid false sharing of 'mTop' and 'mBottom' since they are being acessed by multiple threads.

        alignas(64) std::atomic<size_t> mTop = { 0 };
        const size_t mCapacity;
        const size_t mMask;
        Slot* const mSlots = nullptr;
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
            mSlots(new Slot[mCapacity]()),
            mBottom(0)
        {}

        ~ChaseLevQueue()
        {
            delete[] mSlots;
        }

        ChaseLevQueue(const ChaseLevQueue&) = delete;
        ChaseLevQueue& operator=(const ChaseLevQueue&) = delete;
        ChaseLevQueue(ChaseLevQueue&&) = delete;
        ChaseLevQueue& operator=(ChaseLevQueue&&) = delete;

        // Producer.
        bool try_push(Task&& task)
        {
            size_t top, bottom;
            {
                bottom = mBottom.load(std::memory_order_relaxed);
                top = mTop.load(std::memory_order_acquire);
            }
            {
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
                while (mSlots[slotIndex].flag.load(std::memory_order_acquire))
                    std::this_thread::yield();
            }

            // Move task.
            {
                mSlots[slotIndex].task = std::move(task);
            }

            // Set occupied flag to true.
            {
                mSlots[slotIndex].flag.store(true, std::memory_order_release);
            }
            
            // Advance bottom.
            {
                mBottom.store(bottom + 1, std::memory_order_release);
            }
            return true;
        }

        void push(Task&& task)
        {
            while (!try_push(std::move(task)))
            {
                TracyMessage("Queue can't push!", 17);
                std::this_thread::yield();
            }
        }

        // if all tasks won't get pushed, begin part of array can be moved, other not. Advance source by written tasks.
        size_t try_bulk_push(Task* taskSource, size_t taskCount)
        {
            if (taskCount == 0) [[unlikely]] return 0;

            size_t top, bottom;
            {
                bottom = mBottom.load(std::memory_order_relaxed);
                top = mTop.load(std::memory_order_acquire);
            }
            size_t allowedTaskCount;
            {
                const size_t size = bottom - top;
                if (size >= mCapacity)
                {
                    return 0; // Queue is full.
                }
                allowedTaskCount = std::min(mCapacity - size, taskCount);
            }

            {
                for (size_t i = 0; i < allowedTaskCount; i++)
                {
                    const size_t idx = (bottom + i) & mMask;
                    if (mSlots[idx].flag.load(std::memory_order_acquire))
                    {
                        // Slot is occupied - this should never happen.
                        return 0;
                    }
                }
            }

            // Move tasks and set flags to true.
            {
                for (size_t i = 0; i < allowedTaskCount; i++)
                {
                    const size_t slotIndex = (bottom + i) & mMask;

                    mSlots[slotIndex].task = std::move(taskSource[i]);
                    mSlots[slotIndex].flag.store(true, std::memory_order_release);
                }
            }

            // Advance bottom.
            {
                mBottom.store(bottom + allowedTaskCount, std::memory_order_release);
            }
            return allowedTaskCount;
        }

        void bulk_push(Task* taskSource, size_t taskCount)
        {
            size_t tasksLeft = taskCount;
            while (true)
            {
                const size_t written = try_bulk_push(taskSource, tasksLeft);
                tasksLeft -= written;
                if (tasksLeft == 0)
                {
                    break;
                }
                taskSource += written;

                TracyMessage("Queue can't bulk push all!", 26);
                std::this_thread::yield();
            }
        }

        // No 'pop' method.

        // Consumer.
        Task steal()
        {
            while (true)
            {
                size_t top = mTop.load(std::memory_order_acquire);
                const size_t bottom = mBottom.load(std::memory_order_acquire);
                if (bottom <= top)
                {
                    return {}; // Empty.
                }

                // Check if occupied.
                const bool isOccupied = mSlots[top & mMask].flag.load(std::memory_order_acquire);
                if (!isOccupied)
                {
                    continue;
                }

                if (mTop.compare_exchange_strong(top, top + 1,
                    std::memory_order_seq_cst,
                    std::memory_order_relaxed))
                {
                    // Get task.
                    Task task = std::move(mSlots[top & mMask].task);

                    // Set as unoccupied.
                    mSlots[top & mMask].flag.store(false, std::memory_order_release);

                    return task;
                }
                // CAS failed, retry.
            }
        }
    };
}