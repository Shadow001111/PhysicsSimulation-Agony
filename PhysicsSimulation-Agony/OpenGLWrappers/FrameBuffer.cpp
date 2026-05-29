#include "FrameBuffer.h"

#include <utility>
#include <iostream>
#include <tracy/Tracy.hpp>

FrameBuffer::~FrameBuffer()
{
    destroy();
}

FrameBuffer::FrameBuffer(FrameBuffer&& other) noexcept :
    id(std::exchange(other.id, 0)),
    width(std::exchange(other.width, 0)),
    height(std::exchange(other.height, 0)),
    attachments(std::move(other.attachments)),
    drawBufferIndices(std::move(drawBufferIndices))
{
}

FrameBuffer& FrameBuffer::operator=(FrameBuffer&& other) noexcept
{
    if (this != &other)
    {
        if (id) destroy();

        id = std::exchange(other.id, 0);
        width = std::exchange(other.width, 0);
        height = std::exchange(other.height, 0);
        attachments = std::move(other.attachments);
        drawBufferIndices = std::move(drawBufferIndices);
    }
    return *this;
}

void FrameBuffer::create(int width, int height)
{
    // Clean up
    if (id) destroy();

    // Create new frame buffer
    glCreateFramebuffers(1, &id);

    // Set new resolution
    this->width = width;
    this->height = height;
}

void FrameBuffer::destroy()
{
    if (id)
    {
        for (auto& pair : attachments)
        {
            auto& attachment = pair.second;
            // Detach from framebuffer
            if (attachment.attachmentPoint != -1)
            {
                glNamedFramebufferTexture(id, attachment.attachmentPoint, 0, 0);
            }
        }
        attachments.clear();

        glDeleteFramebuffers(1, &id);
        id = 0;

        drawBufferIndices.clear();
    }
}

void FrameBuffer::createColorAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless)
{
    Attachment attachment;
    attachment.type = AttachmentType::COLOR;
    attachment.attachmentPoint = getAttachmentPoint(AttachmentType::COLOR);

    createAndAttachTexture(attachment, internalFormat, params);
	makeTextureBindlessIfNeeded(attachment, bindless);

    attachments[name] = std::move(attachment);
}

void FrameBuffer::createDepthAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless)
{
    Attachment attachment;
    attachment.type = AttachmentType::DEPTH;
    attachment.attachmentPoint = GL_DEPTH_ATTACHMENT; // TODO: Keep track of depth attachment (by pointer), so there will be single depth attachment.

    createAndAttachTexture(attachment, internalFormat, params);
    makeTextureBindlessIfNeeded(attachment, bindless);

    attachments[name] = std::move(attachment);
}

void FrameBuffer::createStencilAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless)
{
    Attachment attachment;
    attachment.type = AttachmentType::STENCIL;
    attachment.attachmentPoint = GL_STENCIL_ATTACHMENT;

    createAndAttachTexture(attachment, internalFormat, params);
    makeTextureBindlessIfNeeded(attachment, bindless);

    attachments[name] = std::move(attachment);
}

void FrameBuffer::createDepthStencilAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, bool bindless)
{
    Attachment attachment;
    attachment.type = AttachmentType::DEPTH_STENCIL;
    attachment.attachmentPoint = GL_DEPTH_STENCIL_ATTACHMENT;

    createAndAttachTexture(attachment, internalFormat, params);
    makeTextureBindlessIfNeeded(attachment, bindless);

    attachments[name] = std::move(attachment);
}

void FrameBuffer::createStandaloneTextureAttachment(const std::string& name, GLenum internalFormat, const Texture::Parameters& params, float resolutionFactor, bool bindless)
{
    Attachment attachment;
    attachment.type = AttachmentType::STANDALONE_TEXTURE;
    attachment.attachmentPoint = -1;
    attachment.resolutionFactor = resolutionFactor;

    createStandaloneTexture(attachment, internalFormat, params);
    makeTextureBindlessIfNeeded(attachment, bindless);

    attachments[name] = std::move(attachment);
}

void FrameBuffer::removeAttachment(const std::string& name)
{
    auto it = attachments.find(name);
    if (it != attachments.end())
    {
        // Detach from framebuffer
        if (it->second.attachmentPoint != -1)
        {
            glNamedFramebufferTexture(id, it->second.attachmentPoint, 0, 0);
        }
        attachments.erase(it);
    }
}

void FrameBuffer::bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, id);
}

void FrameBuffer::bind(GLenum target) const
{
    glBindFramebuffer(target, id);
}

void FrameBuffer::unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::unbind(GLenum target)
{
    glBindFramebuffer(target, 0);
}

void FrameBuffer::setDrawBuffers(const std::vector<std::string>& attachmentNames)
{
    ZoneScopedN("Set draw buffers");

    drawBufferIndices.clear();

    if (attachmentNames.empty())
    {
        // If no attachments specified, disable drawing
        glNamedFramebufferDrawBuffer(id, GL_NONE);
        glNamedFramebufferReadBuffer(id, GL_NONE);
        return;
    }

    std::vector<GLenum> drawBufferAttachmentPoints;
    drawBufferAttachmentPoints.reserve(attachmentNames.size());

    for (const auto& name : attachmentNames)
    {
        auto it = attachments.find(name);
        if (it != attachments.end())
        {
            // Only COLOR attachments can be draw buffers
            if (it->second.type == AttachmentType::COLOR)
            {
                int index = static_cast<int>(drawBufferAttachmentPoints.size());
                drawBufferIndices[name] = index;

                drawBufferAttachmentPoints.push_back(it->second.attachmentPoint);
            }
            else
            {
                std::cerr << "[FrameBuffer][setDrawBuffers]: Attachment '" << name
                    << "' is not a COLOR attachment (type: " << static_cast<int>(it->second.type)
                    << "). Skipping.\n";
            }
        }
        else
        {
            std::cerr << "[FrameBuffer][setDrawBuffers]: Attachment '" << name
                << "' not found. Skipping.\n";
        }
    }

    if (drawBufferAttachmentPoints.empty())
    {
        // No valid color attachments found
        glNamedFramebufferDrawBuffer(id, GL_NONE);
        glNamedFramebufferReadBuffer(id, GL_NONE);
    }
    else
    {
        glNamedFramebufferDrawBuffers(id, static_cast<GLsizei>(drawBufferAttachmentPoints.size()), drawBufferAttachmentPoints.data());
    }
}

void FrameBuffer::setReadBuffer(const std::string& attachmentName) const
{
    ZoneScopedN("Set read buffers");

    auto it = attachments.find(attachmentName);
    if (it != attachments.end())
    {
        // Only COLOR attachments can be read buffers
        if (it->second.type == AttachmentType::COLOR)
        {
            glNamedFramebufferReadBuffer(id, it->second.attachmentPoint);
        }
        else
        {
            std::cerr << "[FrameBuffer][setReadBuffer]: Attachment '" << attachmentName
                << "' is not a COLOR attachment (type: " << static_cast<int>(it->second.type)
                << "). Cannot set as read buffer.\n";
        }
    }
    else
    {
        glNamedFramebufferReadBuffer(id, GL_NONE);
        std::cerr << "[FrameBuffer][setReadBuffer]: Attachment '" << attachmentName
            << "' not found. Cannot set as read buffer.\n";
	}
}

void FrameBuffer::resize(int newWidth, int newHeight)
{
    if (newWidth == width && newHeight == height)
    {
        return;
    }
    if (newWidth <= 0 || newHeight <= 0)
    {
        std::cerr << "[FrameBuffer][resize]: Invalid franebuffer dimensions: width=" << newWidth << ", height=" << newHeight << "\n";
        return;
    }

    width = newWidth;
    height = newHeight;

    for (auto& [name, attachment] : attachments)
    {
        if (attachment.type == AttachmentType::STANDALONE_TEXTURE)
        {
            int tWidth = static_cast<int>(width * attachment.resolutionFactor);
            int tHeight = static_cast<int>(height * attachment.resolutionFactor);
            attachment.texture.recreate2D(tWidth, tHeight);
        }
        else
        {
            attachment.texture.recreate2D(width, height);
            glNamedFramebufferTexture(id, attachment.attachmentPoint, attachment.texture.getID(), 0);
        }
    }
}

void FrameBuffer::blitTo(const FrameBuffer& dstFBO, GLbitfield mask, GLenum filter) const
{
    glBlitNamedFramebuffer(
        id, dstFBO.id,
        0, 0, width, height,
        0, 0, dstFBO.width, dstFBO.height,
        mask, filter);
}

void FrameBuffer::blitToDefaultFramebuffer(int dstWidth, int dstHeight, GLbitfield mask, GLenum filter) const
{
    glBlitNamedFramebuffer(
        id, 0,
        0, 0, width, height,
        0, 0, dstWidth, dstHeight,
        mask, filter);
}

void FrameBuffer::clearAttachment(const std::string& name, const float* clearValue) const
{
    ZoneScopedN("Clear attachment");

    auto it = attachments.find(name);
    if (it == attachments.end())
    {
        std::cerr << "[FrameBuffer][clearAttachment]: Attachment '" << name << "' not found in attachments for clearing\n";
        return;
    }

    const auto& attachment = it->second;
    const auto& texture = attachment.texture;

    switch (attachment.type)
    {
    case AttachmentType::COLOR:
    case AttachmentType::STANDALONE_TEXTURE:
        glClearTexImage(texture.getID(), 0, texture.getFormatFromInternalFormat(), GL_FLOAT, clearValue);
        break;
    case AttachmentType::DEPTH:
        glClearNamedFramebufferfv(id, GL_DEPTH, 0, clearValue);
        break;
    case AttachmentType::STENCIL:
    {
        GLint stencilValue = static_cast<GLint>(clearValue[0]);
        glClearNamedFramebufferiv(id, GL_STENCIL, 0, &stencilValue);
        break;
    }
    case AttachmentType::DEPTH_STENCIL:
        glClearNamedFramebufferfi(id, GL_DEPTH_STENCIL, 0, clearValue[0], static_cast<GLint>(clearValue[1]));
        break;
    }
}

void FrameBuffer::clearDrawBuffer(const std::string& name, const float* clearValue) const
{
    ZoneScopedN("Clear draw buffers");

    auto it = drawBufferIndices.find(name);
    if (it != drawBufferIndices.end())
    {
        glClearNamedFramebufferfv(id, GL_COLOR, it->second, clearValue);
    }
    else
    {
        std::cerr << "[FrameBuffer][clearDrawBuffer]: Attachment '"
            << name << "' not found in draw buffers for clearing\n";
    }
}

std::optional<Texture*> FrameBuffer::getTexture(const std::string& name)
{
    auto it = attachments.find(name);
    if (it != attachments.end()) return &it->second.texture;
    return std::nullopt;
}

std::optional<const Texture*> FrameBuffer::getTexture(const std::string& name) const
{
    auto it = attachments.find(name);
    if (it != attachments.end()) return &it->second.texture;
    return std::nullopt;
}

bool FrameBuffer::hasTexture(const std::string& name) const
{
    return attachments.find(name) != attachments.end();
}

bool FrameBuffer::isComplete() const
{
    GLenum status = glCheckNamedFramebufferStatus(id, GL_FRAMEBUFFER);
    if (status == GL_FRAMEBUFFER_COMPLETE)
    {
        return true;
    }

    std::string errorMsg;
    switch (status)
    {
    case GL_FRAMEBUFFER_UNDEFINED:
        errorMsg = "GL_FRAMEBUFFER_UNDEFINED";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
        break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
        errorMsg = "GL_FRAMEBUFFER_UNSUPPORTED";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
        errorMsg = "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
        break;
    default:
        errorMsg = "Unknown error: " + std::to_string(status);
        break;
    }
    std::cerr << "[FrameBuffer][isComplete]: Framebuffer is not complete: " << errorMsg << "\n";
    return false;
}

GLenum FrameBuffer::getAttachmentPoint(AttachmentType type)
{
    if (type != AttachmentType::COLOR)
    {
        switch (type)
        {
        case AttachmentType::DEPTH: return GL_DEPTH_ATTACHMENT;
        case AttachmentType::STENCIL: return GL_STENCIL_ATTACHMENT;
        case AttachmentType::DEPTH_STENCIL: return GL_DEPTH_STENCIL_ATTACHMENT;
        default: return -1;
        }
    }

    constexpr int ATTACHMENT_POINT_COUNT = 32;
    bool usedAttachments[ATTACHMENT_POINT_COUNT];
    std::fill_n(usedAttachments, sizeof(usedAttachments), false);

    for (const auto& [name, attachment] : attachments)
    {
        if (attachment.type == AttachmentType::COLOR)
        {
            int index = attachment.attachmentPoint - GL_COLOR_ATTACHMENT0;
            if (index >= 0 && index < ATTACHMENT_POINT_COUNT)
                usedAttachments[index] = true;
        }
    }

    // Find first unused attachment point
    for (int i = 0; i < ATTACHMENT_POINT_COUNT; i++)
    {
        if (!usedAttachments[i])
        {
            return GL_COLOR_ATTACHMENT0 + i;
        }
    }

    std::cerr << "[FrameBuffer][getAttachmentPoint]: No available color attachment points\n";
    return GL_COLOR_ATTACHMENT0;
}

void FrameBuffer::createAndAttachTexture(Attachment& attachment, GLenum internalFormat, const Texture::Parameters& params) const
{
    attachment.texture.create2D(width, height, internalFormat);
    attachment.texture.setParameters(params);

    if (attachment.attachmentPoint != -1)
    {
        glNamedFramebufferTexture(id, attachment.attachmentPoint, attachment.texture.getID(), 0);
    }
}

void FrameBuffer::createStandaloneTexture(Attachment& attachment, GLenum internalFormat, const Texture::Parameters& params) const
{
    int tWidth = static_cast<int>(width * attachment.resolutionFactor);
    int tHeight = static_cast<int>(height * attachment.resolutionFactor);
    attachment.texture.create2D(tWidth, tHeight, internalFormat);
    attachment.texture.setParameters(params);
}

void FrameBuffer::makeTextureBindlessIfNeeded(Attachment& attachment, bool bindless)
{
    if (bindless && Texture::getExtensions().bindless)
    {
        attachment.texture.initHandle();
        attachment.texture.makeResident();
	}
}

FrameBuffer::Attachment::Attachment(Attachment&& other) noexcept :
    texture(std::move(other.texture)),
    type(other.type),
    attachmentPoint(other.attachmentPoint)
{
    other.type = AttachmentType::COLOR;
    other.attachmentPoint = -1;
}

FrameBuffer::Attachment& FrameBuffer::Attachment::operator=(Attachment&& other) noexcept
{
    if (this != &other)
    {
        texture = std::move(other.texture);

        type = other.type;
        attachmentPoint = other.attachmentPoint;

        other.type = AttachmentType::COLOR;
        other.attachmentPoint = -1;
    }
    return *this;
}
