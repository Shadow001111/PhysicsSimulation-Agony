#pragma once
#include "Texture.h"
#include "robin_hood.h"
#include <string>
#include <optional>

class FrameBuffer
{
    enum class AttachmentType
    {
        COLOR,
        DEPTH,
        STENCIL,
        DEPTH_STENCIL,
        STANDALONE_TEXTURE
    };

    struct Attachment
    {
        Texture texture;
        AttachmentType type = AttachmentType::COLOR;
        GLenum attachmentPoint = -1;
        float resolutionFactor = 1.0f;

        Attachment(const Attachment&) = delete;
        Attachment& operator=(const Attachment&) = delete;

        Attachment() = default;

        Attachment(Attachment&& other) noexcept;
        Attachment& operator=(Attachment&& other) noexcept;
    };

    GLuint id = 0;
    int width = 0, height = 0;

    robin_hood::unordered_flat_map<std::string, Attachment> attachments;
    robin_hood::unordered_flat_map<std::string, int> drawBufferIndices;
public:
    FrameBuffer() = default;
    ~FrameBuffer();

    // Delete copy constructor/assignment
    FrameBuffer(const FrameBuffer&) = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;

    // Move constructor/assignment
    FrameBuffer(FrameBuffer&& other) noexcept;
    FrameBuffer& operator=(FrameBuffer&& other) noexcept;

    // Creation
    void create(int width, int height);
    void destroy();

    // Attachment management
    void createColorAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless = false);

    void createDepthAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless = false);

    void createStencilAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless = false);

    void createDepthStencilAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless = false);

    void createStandaloneTextureAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, float resolutionFactor = 1.0f, bool bindless = false);

    // Remove attachments
    void removeAttachment(const std::string& name);

    // FBO operations
    void bind() const;
    void bind(GLenum target) const;

    static void unbind();
    static void unbind(GLenum target);

    void setDrawBuffers(const std::vector<std::string>& attachmentNames);
	void setReadBuffer(const std::string& attachmentName) const;
    void resize(int newWidth, int newHeight);

    // Set read and draw buffers before calling
	void blitTo(const FrameBuffer& dstFBO, GLbitfield mask = GL_COLOR_BUFFER_BIT, GLenum filter = GL_NEAREST) const;
    // Set read and draw buffers before calling
    void blitToDefaultFramebuffer(int dstWidth, int dstHeight, GLbitfield mask = GL_COLOR_BUFFER_BIT, GLenum filter = GL_NEAREST) const;

    void clearAttachment(const std::string& name, const float* clearValue) const;
    void clearDrawBuffer(const std::string& name, const float* clearValue) const;

    // Texture access
    [[nodiscard]] std::optional<Texture*> getTexture(const std::string& name);
    [[nodiscard]] std::optional<const Texture*> getTexture(const std::string& name) const;
    [[nodiscard]] bool hasTexture(const std::string& name) const;

    // Getters
    [[nodiscard]] bool isComplete() const;
    [[nodiscard]] GLuint getID() const { return id; }
    [[nodiscard]] int getWidth() const { return width; }
    [[nodiscard]] int getHeight() const { return height; }
private:
    [[nodiscard]] GLenum getAttachmentPoint(AttachmentType type);
    void createAndAttachTexture(Attachment& attachment, GLenum internalFormat, const Texture::Parameters& params) const;
    void createStandaloneTexture(Attachment& attachment, GLenum internalFormat, const Texture::Parameters& params) const;
	void makeTextureBindlessIfNeeded(Attachment& attachment, bool bindless);
};