#include "ImmutableBuffer.h"
#include <iostream>
#include <utility>

#define BUFFER_SAFETY_CHECKS false

ImmutableBuffer::~ImmutableBuffer()
{
    destroy();
}

ImmutableBuffer::ImmutableBuffer(ImmutableBuffer&& other) noexcept :
    target(std::exchange(other.target, 0)),
    id(std::exchange(other.id, 0)),
    flags(std::exchange(other.flags, 0)),
    capacity(std::exchange(other.capacity, 0)),
	persistentMappedPtr(std::exchange(other.persistentMappedPtr, nullptr))
{
}

ImmutableBuffer& ImmutableBuffer::operator=(ImmutableBuffer&& other) noexcept
{
    if (this != &other)
    {
        if (id) glDeleteBuffers(1, &id);

        target = std::exchange(other.target, 0);
        id = std::exchange(other.id, 0);
        flags = std::exchange(other.flags, 0);
        capacity = std::exchange(other.capacity, 0);
        persistentMappedPtr = std::exchange(other.persistentMappedPtr, nullptr);
    }
    return *this;
}

void ImmutableBuffer::create(GLenum target)
{
    this->target = target;
    this->flags = 0;
    this->capacity = 0;
    this->persistentMappedPtr = nullptr;

    if (id)
    {
        glDeleteBuffers(1, &id);
    }
    glCreateBuffers(1, &id);
}

void ImmutableBuffer::destroy()
{
    this->target = 0;
    this->flags = 0;
    this->capacity = 0;
    this->persistentMappedPtr = nullptr;

    if (id)
    {
        glDeleteBuffers(1, &id);
        id = 0;
    }
}

void ImmutableBuffer::allocateStorage(size_t size, GLbitfield flags, const void* data)
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][allocateStorage]: Buffer not created! Call create() first.\n";
        return;
	}
    if (size == 0)
    {
        std::cerr << "[ImmutableBuffer][allocateStorage]: Size must be greater than 0.\n";
        return;
	}
    if (capacity > 0)
    {
        std::cerr << "[ImmutableBuffer][allocateStorage]: Storage already allocated! Cannot resize immutable buffer.\n";
        return;
    }
#endif
    capacity = size;
    this->flags = flags;
    glNamedBufferStorage(id, capacity, data, flags);
}

void ImmutableBuffer::bind() const
{
    glBindBuffer(target, id);
}

void ImmutableBuffer::bind(GLenum target) const
{
    glBindBuffer(target, id);
}

void ImmutableBuffer::unbind() const
{
    glBindBuffer(target, 0);
}

void ImmutableBuffer::unbind(GLenum target)
{
    glBindBuffer(target, 0);
}

void ImmutableBuffer::bindBase(GLuint index) const
{
    glBindBufferBase(target, index, id);
}

void ImmutableBuffer::bindBase(GLenum target, GLuint index) const
{
    glBindBufferBase(target, index, id);
}

void ImmutableBuffer::swap(ImmutableBuffer& other) noexcept
{
    std::swap(target, other.target);
    std::swap(flags, other.flags);
    std::swap(id, other.id);
    std::swap(capacity, other.capacity);
    std::swap(persistentMappedPtr, other.persistentMappedPtr);
}

void ImmutableBuffer::write(const void* data, size_t dataSize, size_t offset) const
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][write]: Buffer not created! Call create() first.\n";
        return;
    }
    if (data == nullptr)
    {
        std::cerr << "[ImmutableBuffer][write]: 'data' is nullptr.\n";
        return;
    }
    if (dataSize == 0)
    {
        std::cerr << "[ImmutableBuffer][write]: Size must be greater than 0.\n";
        return;
	}
    if (offset + dataSize > capacity)
    {
		std::cerr << "[ImmutableBuffer][write]: Index out of bounds! Start: " << offset << ", Size: " << dataSize << ", Capacity: " << capacity << ".\n";
        return;
    }
#endif
    glNamedBufferSubData(id, offset, dataSize, data);
}

void ImmutableBuffer::writePersistentMapped(const void* data, size_t dataSize, size_t offset) const
{
    if (!persistentMappedPtr)
    {
#if BUFFER_SAFETY_CHECKS
		std::cerr << "[ImmutableBuffer][writePersistentMapped]: Buffer is not persistently mapped! Call mapPersistent() first.\n";
#endif
		return;
    }
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Buffer not created! Call create() first.\n";
        return;
    }
    if (data == nullptr)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: 'data' is nullptr.\n";
        return;
    }
    if (dataSize == 0)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Size must be greater than 0.\n";
        return;
    }
    if (offset + dataSize > capacity)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Index out of bounds! Start: " << offset << ", Size: " << dataSize << ", Capacity: " << capacity << ".\n";
        return;
    }
#endif
    std::memcpy(static_cast<char*>(persistentMappedPtr) + offset, data, dataSize);
}

void ImmutableBuffer::writePersistentMappedWithFallback(const void* data, size_t dataSize, size_t offset) const
{
    if (!persistentMappedPtr)
    {
		// Fallback to regular write if not persistently mapped
        write(data, dataSize, offset);
		return;
    }
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Buffer not created! Call create() first.\n";
        return;
    }
    if (data == nullptr)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: 'data' is nullptr.\n";
        return;
    }
    if (dataSize == 0)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Size must be greater than 0.\n";
        return;
	}
    if (offset + dataSize > capacity)
    {
        std::cerr << "[ImmutableBuffer][writePersistentMapped]: Index out of bounds! Start: " << offset << ", Size: " << dataSize << ", Capacity: " << capacity << ".\n";
        return;
    }
#endif
	std::memcpy(static_cast<char*>(persistentMappedPtr) + offset, data, dataSize);
}

// TODO: Add more checks for flags
void ImmutableBuffer::copyRangeFrom(const ImmutableBuffer& src, size_t srcOffset, size_t dstOffset, size_t size) const
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][copyRangeFrom]: Destination buffer not created! Call create() first.\n";
        return;
    }
    if (src.getID() == 0)
    {
        std::cerr << "[ImmutableBuffer][copyRangeFrom]: Source buffer not created! Call create() first.\n";
        return;
    }
    if (size == 0)
    {
        std::cerr << "[ImmutableBuffer][copyRangeFrom]: Size must be greater than 0.\n";
		return;
    }
    if (srcOffset + size > src.capacity || dstOffset + size > capacity)
    {
        std::cerr << "[ImmutableBuffer][copyRangeFrom]: Range exceeds buffer capacity.\n";
        return;
    }
#endif
    glCopyNamedBufferSubData(src.getID(), id,
        static_cast<GLintptr>(srcOffset),
        static_cast<GLintptr>(dstOffset),
        static_cast<GLsizeiptr>(size));
}

void ImmutableBuffer::clearData(GLenum internalFormat, GLenum format, GLenum type, const void* data) const
{
    glClearNamedBufferData(id, internalFormat, format, type, data);
}

void* ImmutableBuffer::map(GLenum access)
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][map]: Buffer not created! Call create() first.\n";
        return nullptr;
    }
    if (persistentMappedPtr)
    {
        std::cerr << "[ImmutableBuffer][map]: Buffer is already persistently mapped! Unmap first before mapping again.\n";
        return nullptr;
	}
    if (!isMappable())
    {
        std::cerr << "[ImmutableBuffer][map]: Buffer was not created with mappable flags! Cannot map.\n";
        return nullptr;
    }
#endif
    void* ptr = glMapNamedBuffer(id, access);
#if BUFFER_SAFETY_CHECKS
    if (ptr == nullptr)
    {
        std::cerr << "[ImmutableBuffer][ImmutableBuffer]: Failed to map buffer.\n";
    }
#endif
    return ptr;
}

void* ImmutableBuffer::mapRange(GLbitfield access, GLsizeiptr size, GLintptr offset)
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Buffer not created! Call create() first.\n";
        return nullptr;
    }
    if (persistentMappedPtr)
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Buffer is already persistently mapped! Unmap first before mapping again.\n";
        return nullptr;
    }
    if (size <= 0)
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Size must be greater than 0.\n";
        return nullptr;
    }
    if (offset + size > capacity)
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Index out of bounds! Start: " << offset << ", Size: " << size << ", Capacity: " << capacity << ".\n";
        return nullptr;
    }
    if (!isMappable())
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Buffer was not created with mappable flags! Cannot map.\n";
        return nullptr;
    }
#endif
    void* ptr = glMapNamedBufferRange(id, offset, size, access);
#if BUFFER_SAFETY_CHECKS
    if (!ptr)
    {
        std::cerr << "[ImmutableBuffer][mapRange]: Failed to map buffer.\n";
    }
#endif
    return ptr;
}

void* ImmutableBuffer::mapPersistentRange(GLbitfield access, GLsizeiptr size, GLintptr offset)
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][mapPersistentRange]: Buffer not created! Call create() first.\n";
        return nullptr;
    }
    if (size <= 0)
    {
        std::cerr << "[ImmutableBuffer][mapPersistentRange]: Size must be greater than 0.\n";
        return nullptr;
	}
    if (size > capacity)
    {
        std::cerr << "[ImmutableBuffer][mapPersistentRangeRange]: Size exceeds buffer capacity! Size: " << size << ", Capacity: " << capacity << ".\n";
        return nullptr;
    }
    if (persistentMappedPtr)
    {
        std::cerr << "[ImmutableBuffer][mapPersistentRangeRange]: Buffer is already persistently mapped! Unmap first before mapping again.\n";
        return nullptr;
	}
    if (!isMappablePersistently())
    {
		std::cerr << "[ImmutableBuffer][mapPersistentRange]: Buffer was not created with mappable flags! Cannot map persistently.\n";
        return nullptr;
    }
#endif
    persistentMappedPtr = glMapNamedBufferRange(id, 0, size, access | GL_MAP_PERSISTENT_BIT);
    if (!persistentMappedPtr)
    {
#if BUFFER_SAFETY_CHECKS
        std::cerr << "[ImmutableBuffer][mapPersistentRange]: Failed to map buffer.\n";
#endif
    }
    return persistentMappedPtr;
}

void* ImmutableBuffer::mapPersistent(GLbitfield access)
{
	return mapPersistentRange(access, capacity, 0);
}

void ImmutableBuffer::unmap()
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][unmap]: Buffer not created! Call create() first.\n";
        return;
    }
#endif
    GLboolean result = glUnmapNamedBuffer(id);
#if BUFFER_SAFETY_CHECKS
    if (result == GL_FALSE)
    {
        std::cerr << "[ImmutableBuffer][unmap]: Failed to unmap buffer.\n";
    }
#endif
}

void ImmutableBuffer::flushMappedRange(GLsizeiptr size, GLintptr offset)
{
#if BUFFER_SAFETY_CHECKS
    if (id == 0)
    {
        std::cerr << "[ImmutableBuffer][flushMappedRange]: Buffer not created! Call create() first.\n";
        return;
    }
    if (size <= 0)
    {
        std::cerr << "[ImmutableBuffer][flushMappedRange]: Size must be greater than 0.\n";
		return;
    }
    if (offset + size > capacity)
    {
        std::cerr << "[ImmutableBuffer][flushMappedRange]: Index out of bounds! Start: " << offset << ", Size: " << size << ", Capacity: " << capacity << ".\n";
        return;
    }
#endif
    glFlushMappedNamedBufferRange(id, offset, size);
}