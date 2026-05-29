#pragma once
#include "Core/Container/DynamicArray.h"

template<typename T, size_t ItemsPerBlock>
class FixedArenaAllocator
{
    static_assert(ItemsPerBlock > 0, "ItemsPerBlock must be greater than 0");

    struct Block
    {
        T* memory;

        Block() :
            memory(
                static_cast<T*>(::operator new(ItemsPerBlock * sizeof(T)))
            )
        {
            if (!memory) throw std::bad_alloc();
        }

        ~Block()
        {
            ::operator delete(memory);
        }

        Block(const Block&) = delete;
        Block& operator=(const Block&) = delete;

        Block(Block&& other) noexcept : memory(other.memory)
        {
            other.memory = nullptr;
        }

        Block& operator=(Block&& other) noexcept
        {
            if (this != &other)
            {
                ::operator delete(memory);
                memory = other.memory;
                other.memory = nullptr;
            }
            return *this;
        }

        T* getSlot(size_t index) { return &memory[index]; }
    };

    DynamicArray<Block> blocks;
    size_t itemsUsedInCurrentBlock = 0;

    void allocateBlock()
    {
        blocks.emplace_back();
        itemsUsedInCurrentBlock = 0;
    }
public:
    FixedArenaAllocator()
    {
        blocks.reserve(10);
        allocateBlock();
    }
private:
    void destroyAll()
    {
        for (size_t blockIdx = 0; blockIdx < blocks.size(); blockIdx++)
        {
            Block& block = blocks[blockIdx];
            size_t itemsInThisBlock = (blockIdx == blocks.size() - 1)
                ? itemsUsedInCurrentBlock
                : ItemsPerBlock;

            for (size_t i = 0; i < itemsInThisBlock; i++)
            {
                block.getSlot(i)->~T();
            }
        }
    }
public:
    ~FixedArenaAllocator()
    {
        destroyAll();
    }

    FixedArenaAllocator(const FixedArenaAllocator&) = delete;
    FixedArenaAllocator& operator=(const FixedArenaAllocator&) = delete;

    FixedArenaAllocator(FixedArenaAllocator&& other) noexcept :
        blocks(std::move(other.blocks)) ,
        itemsUsedInCurrentBlock(other.itemsUsedInCurrentBlock)
    {
        other.itemsUsedInCurrentBlock = 0;
    }

    FixedArenaAllocator& operator=(FixedArenaAllocator&& other) noexcept
    {
        if (this != &other)
        {
            destroyAll();

            blocks = std::move(other.blocks);
            itemsUsedInCurrentBlock = other.itemsUsedInCurrentBlock;
            other.itemsUsedInCurrentBlock = 0;
        }
        return *this;
    }

    T* allocate()
    {
        if (itemsUsedInCurrentBlock >= ItemsPerBlock)
        {
            allocateBlock();
        }

        T* ptr = blocks.back().getSlot(itemsUsedInCurrentBlock++);
        return ptr;
    }

    template<typename... Args>
    T* create(Args&&... args)
    {
        T* ptr = allocate();
        new(ptr) T(std::forward<Args>(args)...);
        return ptr;
    }

    void reset()
    {
        destroyAll();

        blocks.clear();
        allocateBlock();
    }

    size_t capacity() const { return blocks.size() * ItemsPerBlock; }
    size_t size() const { return (blocks.size() - 1) * ItemsPerBlock + itemsUsedInCurrentBlock; }
    size_t itemsPerBlock() const { return ItemsPerBlock; }
};