#pragma once
#include <glad/glad.h>

class VertexArray
{
    GLuint id = 0;
public:
    VertexArray() = default;
    ~VertexArray();

    VertexArray(const VertexArray& other) = delete;
    VertexArray& operator=(const VertexArray& other) = delete;

    VertexArray(VertexArray&& other) noexcept;
    VertexArray& operator=(VertexArray&& other) noexcept;

    void create();
    void destroy();

    void bind() const;
    static void unbind();

    void setFloatAttribute(
        GLuint attributeIndex, GLint componentCount, GLsizei stride, GLuint bindingIndex = 0,
        GLenum type = GL_FLOAT, GLboolean normalized = GL_FALSE);

    void setIntAttribute(
        GLuint attributeIndex, GLint componentCount, GLsizei stride, GLuint bindingIndex = 0,
        GLenum type = GL_INT);

    void enableAttribute(GLuint index);
    void disableAttribute(GLuint index);
    void setAttributeDivisor(GLuint index, GLuint divisor);

    void bindVertexBuffer(GLuint bindingIndex, GLuint bufferId, GLintptr offset, GLsizei stride);
    void bindElementBuffer(GLuint bufferId);

    [[nodiscard]] GLuint getID() const { return id; };
};