#pragma once
#include "DynamicArray.h"
#include <algorithm>

template<typename T, size_t BlockSize = 64>
class Dequeue
{
    static_assert(BlockSize > 0, "Block size must be greater than zero");
    static_assert((BlockSize& (BlockSize - 1)) == 0, "Block size must be a power of two");

    DynamicArray<T*> mBlocks;

    size_t mHeadBlockIndex = 0;
    size_t mHeadElementIndex = 0;

    size_t mTailBlockIndex = 0;
    size_t mTailElementIndex = 0;

    size_t mElementCount = 0;

    static T* allocate_block()
    {
        return static_cast<T*>(::operator new(BlockSize * sizeof(T)));
    }

    static void deallocate_block(T* block)
    {
        ::operator delete(block);
    }

    template<typename... Args>
    void construct_at(T* ptr, Args&&... args)
    {
        new (ptr) T(std::forward<Args>(args)...);
    }

    void destruct_at(T* ptr)
    {
        ptr->~T();
    }

    // Get actual array index with wraparound
    size_t wrap_index(size_t index) const
    {
        return index % mBlocks.size();
    }

    // Calculate number of blocks currently in use
    size_t blocks_in_use() const noexcept
    {
        if (mElementCount == 0) return 0;

        // Calculate how many blocks span from head to tail
        size_t headBlock = mHeadBlockIndex;
        size_t tailBlock = mTailBlockIndex;

        if (mTailElementIndex == 0)
        {
            tailBlock = (tailBlock == 0) ? mBlocks.size() - 1 : tailBlock - 1;
        }

        if (headBlock <= tailBlock)
        {
            return tailBlock - headBlock + 1;
        }
        else
        {
            // Wrapped around
            return (mBlocks.size() - headBlock) + tailBlock + 1;
        }
    }

    // Grow the block array when full
    void grow_blocks()
    {
        size_t oldSize = mBlocks.size();
        size_t newSize = oldSize * 2;
        if (newSize < 4) newSize = 4;

        DynamicArray<T*> newBlocks;
        newBlocks.resize(newSize);

        for (size_t i = 0; i < newSize; i++)
            newBlocks[i] = nullptr;

        size_t destIdx = newSize / 4;
        size_t srcIdx = mHeadBlockIndex;
        size_t numBlocksToCopy = blocks_in_use();

        for (size_t i = 0; i < numBlocksToCopy; i++)
        {
            size_t wrappedIdx = wrap_index(srcIdx++);
            newBlocks[destIdx++] = mBlocks[wrappedIdx];
            mBlocks[wrappedIdx] = nullptr;  // Mark as moved
        }

        // Fix: free old blocks that were NOT moved into newBlocks
        for (size_t i = 0; i < oldSize; i++)
        {
            if (mBlocks[i] != nullptr)
            {
                deallocate_block(mBlocks[i]);
                mBlocks[i] = nullptr;
            }
        }

        for (size_t i = 0; i < newSize; i++)
        {
            if (newBlocks[i] == nullptr)
                newBlocks[i] = allocate_block();
        }

        size_t newHeadBlock = newSize / 4;
        mTailBlockIndex = newHeadBlock + numBlocksToCopy - 1;
        if (mTailElementIndex == 0 && numBlocksToCopy > 0)
            mTailBlockIndex++;
        mHeadBlockIndex = newHeadBlock;

        mBlocks = std::move(newBlocks);
    }
public:
    Dequeue(size_t elementCount = 0)
    {
        if (elementCount > 0)
        {
            reserve(elementCount);
        }
    }

    ~Dequeue()
    {
        clear();

        for (T* block : mBlocks)
        {
            if (block != nullptr)
            {
                deallocate_block(block);
            }
        }
    }

    Dequeue(const Dequeue& other)
    {
        if (other.mElementCount == 0)
        {
            return;
        }

        // Calculate blocks needed
        size_t blocksNeeded = (other.mElementCount + BlockSize - 1) / BlockSize;
        size_t totalBlocks = std::max<size_t>(blocksNeeded * 2, 4);

        mBlocks.resize(totalBlocks);
        for (size_t i = 0; i < totalBlocks; i++)
        {
            mBlocks[i] = allocate_block();
        }

        // Start in the middle
        size_t startBlock = totalBlocks / 4;
        mHeadBlockIndex = startBlock;
        mHeadElementIndex = 0;
        mTailBlockIndex = startBlock;
        mTailElementIndex = 0;

        // Copy all elements
        size_t srcBlockIdx = other.mHeadBlockIndex;
        size_t srcElemIdx = other.mHeadElementIndex;

        for (size_t i = 0; i < other.mElementCount; i++)
        {
            T* srcPtr = other.mBlocks[other.wrap_index(srcBlockIdx)] + srcElemIdx;
            T* destPtr = mBlocks[mTailBlockIndex] + mTailElementIndex;

            construct_at(destPtr, *srcPtr);

            // Advance source (with wraparound)
            srcElemIdx++;
            if (srcElemIdx >= BlockSize)
            {
                srcElemIdx = 0;
                srcBlockIdx++;
            }

            // Advance destination (with wraparound)
            mTailElementIndex++;
            if (mTailElementIndex >= BlockSize)
            {
                mTailElementIndex = 0;
                mTailBlockIndex++;
                if (mTailBlockIndex >= mBlocks.size())
                {
                    mTailBlockIndex = 0;
                }
            }
        }

        mElementCount = other.mElementCount;
    }

    Dequeue& operator=(const Dequeue& other)
    {
        if (this != &other)
        {
            Dequeue temp(other);
            swap(temp);
        }
        return *this;
    }

    Dequeue(Dequeue&& other) noexcept :
        mBlocks(std::move(other.mBlocks)),
        mHeadBlockIndex(other.mHeadBlockIndex),
        mHeadElementIndex(other.mHeadElementIndex),
        mTailBlockIndex(other.mTailBlockIndex),
        mTailElementIndex(other.mTailElementIndex),
        mElementCount(other.mElementCount)
    {
        other.mElementCount = 0;
        other.mHeadBlockIndex = 0;
        other.mHeadElementIndex = 0;
        other.mTailBlockIndex = 0;
        other.mTailElementIndex = 0;
    }

    Dequeue& operator=(Dequeue&& other) noexcept
    {
        if (this != &other)
        {
            Dequeue temp(std::move(other));
            swap(temp);
        }
        return *this;
    }

    // Modifiers

    template<typename... Args>
    void emplace_front(Args&&... args)
    {
        if (mBlocks.empty())
        {
            mBlocks.resize(4);
            for (size_t i = 0; i < 4; i++)
            {
                mBlocks[i] = allocate_block();
            }

            // Start in the middle
            mHeadBlockIndex = 2;
            mHeadElementIndex = 0;
            mTailBlockIndex = 2;
            mTailElementIndex = 0;
        }
        else if (blocks_in_use() >= mBlocks.size())
        {
            grow_blocks();
        }

        // Move head backwards with wraparound
        if (mHeadElementIndex == 0)
        {
            mHeadBlockIndex = (mHeadBlockIndex == 0) ? mBlocks.size() - 1 : mHeadBlockIndex - 1;
            mHeadElementIndex = BlockSize;
        }
        mHeadElementIndex--;

        construct_at(mBlocks[mHeadBlockIndex] + mHeadElementIndex, std::forward<Args>(args)...);
        mElementCount++;
    }

    void push_front(const T& item)
    {
        emplace_front(item);
    }

    void push_front(T&& item)
    {
        emplace_front(std::move(item));
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        if (mBlocks.empty())
        {
            mBlocks.resize(4);
            for (size_t i = 0; i < 4; i++)
            {
                mBlocks[i] = allocate_block();
            }

            // Start in the middle
            mHeadBlockIndex = 2;
            mHeadElementIndex = 0;
            mTailBlockIndex = 2;
            mTailElementIndex = 0;
        }
        else if (blocks_in_use() >= mBlocks.size())
        {
            grow_blocks();
        }

        construct_at(mBlocks[mTailBlockIndex] + mTailElementIndex, std::forward<Args>(args)...);

        // Move tail forward with wraparound
        mTailElementIndex++;
        if (mTailElementIndex >= BlockSize)
        {
            mTailElementIndex = 0;
            mTailBlockIndex++;
            if (mTailBlockIndex >= mBlocks.size())
            {
                mTailBlockIndex = 0;
            }
        }

        mElementCount++;
    }

    void push_back(const T& item)
    {
        emplace_back(item);
    }

    void push_back(T&& item)
    {
        emplace_back(std::move(item));
    }

    void pop_front()
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("pop_front() called on empty Dequeue");
        }

        destruct_at(mBlocks[mHeadBlockIndex] + mHeadElementIndex);

        mHeadElementIndex++;
        if (mHeadElementIndex >= BlockSize)
        {
            mHeadElementIndex = 0;
            mHeadBlockIndex++;
            if (mHeadBlockIndex >= mBlocks.size())
            {
                mHeadBlockIndex = 0;
            }
        }

        mElementCount--;
    }

    void pop_back()
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("pop_back() called on empty Dequeue");
        }

        if (mTailElementIndex == 0)
        {
            mTailBlockIndex = (mTailBlockIndex == 0) ? mBlocks.size() - 1 : mTailBlockIndex - 1;
            mTailElementIndex = BlockSize;
        }
        mTailElementIndex--;

        destruct_at(mBlocks[mTailBlockIndex] + mTailElementIndex);

        mElementCount--;
    }

    void reserve(size_t elementCount)
    {
        if (elementCount == 0) return;

        size_t blocksNeeded = (elementCount + BlockSize - 1) / BlockSize;
        size_t totalBlocks = std::max<size_t>(blocksNeeded * 2, 4);

        if (totalBlocks > mBlocks.size())
        {
            size_t oldSize = mBlocks.size();
            mBlocks.resize(totalBlocks);

            for (size_t i = oldSize; i < totalBlocks; i++)
            {
                mBlocks[i] = allocate_block();
            }

            // Position in middle if empty
            if (mElementCount == 0)
            {
                size_t middle = totalBlocks / 4;
                mHeadBlockIndex = middle;
                mHeadElementIndex = 0;
                mTailBlockIndex = middle;
                mTailElementIndex = 0;
            }
        }
    }

    void clear()
    {
        if (mElementCount == 0) return;

        size_t blockIdx = mHeadBlockIndex;
        size_t elemIdx = mHeadElementIndex;

        for (size_t i = 0; i < mElementCount; i++)
        {
            destruct_at(mBlocks[blockIdx] + elemIdx);

            elemIdx++;
            if (elemIdx >= BlockSize)
            {
                elemIdx = 0;
                blockIdx++;
                if (blockIdx >= mBlocks.size())
                {
                    blockIdx = 0;
                }
            }
        }

        // Reset to middle position
        if (!mBlocks.empty())
        {
            size_t middle = mBlocks.size() / 2;
            mHeadBlockIndex = middle;
            mHeadElementIndex = 0;
            mTailBlockIndex = middle;
            mTailElementIndex = 0;
        }

        mElementCount = 0;
    }

    void swap(Dequeue& other)
    {
        mBlocks.swap(other.mBlocks);
        std::swap(mHeadBlockIndex, other.mHeadBlockIndex);
        std::swap(mHeadElementIndex, other.mHeadElementIndex);
        std::swap(mTailBlockIndex, other.mTailBlockIndex);
        std::swap(mTailElementIndex, other.mTailElementIndex);
        std::swap(mElementCount, other.mElementCount);
    }

    // Getters

    T& front()
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("front() called on empty Dequeue");
        }
        return mBlocks[mHeadBlockIndex][mHeadElementIndex];
    }

    const T& front() const
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("front() called on empty Dequeue");
        }
        return mBlocks[mHeadBlockIndex][mHeadElementIndex];
    }

    T& back()
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("back() called on empty Dequeue");
        }

        size_t lastBlockIdx = mTailBlockIndex;
        size_t lastElemIdx = mTailElementIndex;

        if (lastElemIdx == 0)
        {
            lastBlockIdx = (lastBlockIdx == 0) ? mBlocks.size() - 1 : lastBlockIdx - 1;
            lastElemIdx = BlockSize;
        }
        lastElemIdx--;

        return mBlocks[lastBlockIdx][lastElemIdx];
    }

    const T& back() const
    {
        if (mElementCount == 0)
        {
            throw std::out_of_range("back() called on empty Dequeue");
        }

        size_t lastBlockIdx = mTailBlockIndex;
        size_t lastElemIdx = mTailElementIndex;

        if (lastElemIdx == 0)
        {
            lastBlockIdx = (lastBlockIdx == 0) ? mBlocks.size() - 1 : lastBlockIdx - 1;
            lastElemIdx = BlockSize;
        }
        lastElemIdx--;

        return mBlocks[lastBlockIdx][lastElemIdx];
    }

    size_t size() const { return mElementCount; }
    bool empty() const { return mElementCount == 0; }
};