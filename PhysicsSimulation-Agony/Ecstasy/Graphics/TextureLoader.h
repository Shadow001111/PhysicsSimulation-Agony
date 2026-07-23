#pragma once
#include "Ecstasy/OpenGL/Texture.h"
#include <string>
#include <vector>
#include <filesystem>

namespace Ecstasy::Graphics::TextureLoader
{
    struct TextureLoadParams
    {
        int desiredChannels = 4;
        bool createMipmaps = false;

        // Set to anything other than NONE to request GPU-compressed storage.
        // AUTO lets the system pick the best format available on the hardware.
        // Compression is skipped silently for texture types that do not support
        // it (GL_TEXTURE_3D) or when the chosen format is unsupported.
        OpenGL::TextureCompression::Format compression = OpenGL::TextureCompression::Format::NONE;

        // Only relevant for BC6H / HDR workflows.
        bool isHDR = false;
    };

    void createTexture2DFromImage(
        OpenGL::Texture& texture,
        const std::filesystem::path& texturePath,
        const TextureLoadParams& params = TextureLoadParams()
    );

    void createTextureArrayFromImages(
        OpenGL::Texture& texture,
        const std::filesystem::path& texturesFolderPath,
        const std::vector<std::string>& textureNames,
        const TextureLoadParams& params = TextureLoadParams()
    );

    void createTexture3DFromFloatData(
        OpenGL::Texture& texture,
        const std::vector<float>& data,
        int width, int height, int depth,
        const TextureLoadParams& params = TextureLoadParams()
    );
}