#pragma once
#include <glad/glad.h>

class ImmutableBuffer
{
    GLenum target = 0;
    GLuint id = 0;
    GLbitfield flags = 0;
    size_t capacity = 0;
    void* persistentMappedPtr = nullptr;
public:
    ImmutableBuffer() = default;
    ~ImmutableBuffer();

    ImmutableBuffer(const ImmutableBuffer& other) = delete;
    ImmutableBuffer& operator=(const ImmutableBuffer& other) = delete;

    ImmutableBuffer(ImmutableBuffer&& other) noexcept;
    ImmutableBuffer& operator=(ImmutableBuffer&& other) noexcept;

    void create(GLenum target);
    void destroy();

    void allocateStorage(size_t size, GLbitfield flags, const void* data = nullptr);

    void bind() const;
    void bind(GLenum target) const;
    void unbind() const;
    static void unbind(GLenum target);

    void bindBase(GLuint index) const;
    void bindBase(GLenum target, GLuint index) const;

    void swap(ImmutableBuffer& other) noexcept;

    void write(const void* data, size_t dataSize, size_t offset = 0) const;
    void writePersistentMapped(const void* data, size_t dataSize, size_t offset = 0) const;
    void writePersistentMappedWithFallback(const void* data, size_t dataSize, size_t offset = 0) const;
    void copyRangeFrom(const ImmutableBuffer& src, size_t srcOffset, size_t dstOffset, size_t size) const;
    void clearData(GLenum internalFormat, GLenum format, GLenum type, const void* data) const;

    void* map(GLenum access);
    void* mapRange(GLbitfield access, GLsizeiptr size, GLintptr offset);
    void* mapPersistent(GLbitfield access);
    void* mapPersistentRange(GLbitfield access, GLsizeiptr size, GLintptr offset);

    void unmap();
    void flushMappedRange(GLsizeiptr size, GLintptr offset);

    [[nodiscard]] GLuint getID() const noexcept { return id; }
    [[nodiscard]] size_t getCapacity() const noexcept { return capacity; }
    [[nodiscard]] GLbitfield getFlags() const noexcept { return flags; }
    [[nodiscard]] bool isMappable() const noexcept { return (flags & (GL_MAP_READ_BIT | GL_MAP_WRITE_BIT)) != 0; }
    [[nodiscard]] bool isMappablePersistently() const noexcept { return (flags & (GL_MAP_READ_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT)) != 0; }
    [[nodiscard]] void* getPersistentMappedPtr() const noexcept { return persistentMappedPtr; };
};