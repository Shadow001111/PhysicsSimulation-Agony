#pragma once
#include "OpenGLWrappers/Texture.h"
#include <string>
#include <vector>
#include <filesystem>

namespace TextureLoader
{
    struct TextureLoadParams
    {
        int desiredChannels = 4;
        bool createMipmaps = false;

        // Set to anything other than NONE to request GPU-compressed storage.
        // AUTO lets the system pick the best format available on the hardware.
        // Compression is skipped silently for texture types that do not support
        // it (GL_TEXTURE_3D) or when the chosen format is unsupported.
        TextureCompression::Format compression = TextureCompression::Format::NONE;

        // Only relevant for BC6H / HDR workflows.
        bool isHDR = false;
    };

    void createTexture2DFromImage(
        Texture& texture,
        const std::filesystem::path& texturePath,
        const TextureLoadParams& params = TextureLoadParams()
    );

    void createTextureArrayFromImages(
        Texture& texture,
        const std::filesystem::path& texturesFolderPath,
        const std::vector<std::string>& textureNames,
        const TextureLoadParams& params = TextureLoadParams()
    );

    void createTexture3DFromFloatData(
        Texture& texture,
        const std::vector<float>& data,
        int width, int height, int depth,
        const TextureLoadParams& params = TextureLoadParams()
    );
}