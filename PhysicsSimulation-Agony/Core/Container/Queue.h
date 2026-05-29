#pragma once
#pragma once
#include "Dequeue.h"

template<typename T, size_t BlockSize = 64>
class Queue
{
    Dequeue<T, BlockSize> mDequeue;
public:
    Queue(size_t initialCapacity = 0)
        : mDequeue(initialCapacity)
    {
    }

    // Copy constructor
    Queue(const Queue& other)
        : mDequeue(other.mDequeue)
    {
    }

    // Move constructor
    Queue(Queue&& other) noexcept
        : mDequeue(std::move(other.mDequeue))
    {
    }

    // Copy assignment
    Queue& operator=(const Queue& other)
    {
        if (this != &other)
        {
            mDequeue = other.mDequeue;
        }
        return *this;
    }

    // Move assignment
    Queue& operator=(Queue&& other) noexcept
    {
        if (this != &other)
        {
            mDequeue = std::move(other.mDequeue);
        }
        return *this;
    }

    // Element access
    T& front()
    {
        return mDequeue.front();
    }

    const T& front() const
    {
        return mDequeue.front();
    }

    T& back()
    {
        return mDequeue.back();
    }

    const T& back() const
    {
        return mDequeue.back();
    }

    // Capacity
    bool empty() const
    {
        return mDequeue.empty();
    }

    size_t size() const
    {
        return mDequeue.size();
    }

    // Modifiers
    template<typename... Args>
    void emplace(Args&&... args)
    {
        mDequeue.emplace_back(std::forward<Args>(args)...);
    }

    void push(const T& value)
    {
        mDequeue.push_back(value);
    }

    void push(T&& value)
    {
        mDequeue.push_back(std::move(value));
    }

    void pop()
    {
        mDequeue.pop_front();
    }

    void clear()
    {
        mDequeue.clear();
    }

    void reserve(size_t elementCount)
    {
        mDequeue.reserve(elementCount);
    }

    void swap(Queue& other)
    {
        mDequeue.swap(other.mDequeue);
    }
};