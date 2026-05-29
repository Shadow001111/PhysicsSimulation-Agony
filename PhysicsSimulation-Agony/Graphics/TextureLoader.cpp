#include "TextureLoader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include <iostream>
#include <cmath>

namespace fs = std::filesystem;

namespace TextureLoader
{
    namespace
    {
        // Generates a magenta/black checkerboard as a placeholder for missing textures.
        std::vector<unsigned char> createUndefinedTexture(int width, int height, int channels)
        {
            std::vector<unsigned char> data(width * height * channels, 0);
            const int halfWidth  = width  >> 1;
            const int halfHeight = height >> 1;

            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    const bool hxy = (x > halfWidth) ^ (y > halfHeight);
                    const unsigned char val = hxy ? 255 : 0;
                    unsigned char* p = data.data() + (y * width + x) * channels;

                    p[0] = val;
                    if (channels >= 2) p[1] = (channels == 2) ? 255 : 0; // G=0 or second-channel fill
                    if (channels >= 3) p[2] = val;
                    if (channels >= 4) p[3] = 255;
                }
            }
            return data;
        }

        // Converts an integer channel count to the TextureCompression::Channels enum.
        TextureCompression::Channels toCompressionChannels(int channels)
        {
            switch (channels)
            {
            case 1:  return TextureCompression::Channels::R;
            case 2:  return TextureCompression::Channels::RG;
            case 3:  return TextureCompression::Channels::RGB;
            default: return TextureCompression::Channels::RGBA;
            }
        }

        int calculateMipmapLevels(int width, int height, bool createMipmaps)
        {
            if (!createMipmaps) return 1;
            return 1 + static_cast<int>(std::log2(std::max(width, height)));
        }

        int calculateMipmapLevels(int width, int height, int depth, bool createMipmaps)
        {
            if (!createMipmaps) return 1;
            return 1 + static_cast<int>(std::log2(std::max({ width, height, depth })));
        }

        bool validateChannels(int channels)
        {
            if (channels < 1 || channels > 4)
            {
                std::cerr << "[TextureLoader]: Only 1-4 channels are supported. Got: " << channels << "\n";
                return false;
            }
            return true;
        }

        // Loads an image from disk, converting to desiredChannels via stbi's built-in
        // channel conversion. Returns true on success.
        bool loadAndProcessImage(
            const fs::path& path,
            int desiredChannels,
            std::vector<unsigned char>& outputData,
            int& width, int& height)
        {
            if (!fs::exists(path) || !fs::is_regular_file(path))
            {
                std::cerr << "[TextureLoader]: Texture file not found: " << path << "\n";
                return false;
            }

            stbi_set_flip_vertically_on_load(true);

            int channels = 0;
            unsigned char* data = stbi_load(
                path.string().c_str(), &width, &height, &channels, desiredChannels);

            if (!data)
            {
                std::cerr << "[TextureLoader]: Failed to load texture: " << path
                    << " (" << stbi_failure_reason() << ")\n";
                return false;
            }

            outputData.assign(data, data + width * height * desiredChannels);
            stbi_image_free(data);
            return true;
        }
    } // anonymous namespace

    // -----------------------------------------------------------------------
    //  Public API
    // -----------------------------------------------------------------------

    void createTexture2DFromImage(
        Texture& texture,
        const fs::path& texturePath,
        const TextureLoadParams& params)
    {
        if (!validateChannels(params.desiredChannels)) return;

        std::vector<unsigned char> imageData;
        int width = 0, height = 0;
        const bool loaded = loadAndProcessImage(
            texturePath, params.desiredChannels, imageData, width, height);

        const GLenum fmt = TextureCompression::resolveInternalFormat(
            params.compression,
            toCompressionChannels(params.desiredChannels),
            params.isHDR);

        if (!loaded)
        {
            // Fallback: small magenta checkerboard
            constexpr int defaultSize = 16;
            
            const int mips = calculateMipmapLevels(defaultSize, defaultSize, params.createMipmaps);

            imageData = createUndefinedTexture(defaultSize, defaultSize, params.desiredChannels);
            texture.create2D(defaultSize, defaultSize, fmt, mips);
            texture.uploadSubData2D(
                imageData.data(), 0, 0, defaultSize, defaultSize, GL_UNSIGNED_BYTE, 0);
            if (mips > 1) texture.generateMipmaps();
            return;
        }

        const int mips = calculateMipmapLevels(width, height, params.createMipmaps);

        // When the internal format is compressed, OpenGL accepts ordinary pixel data via
        // glTextureSubImage* and compresses it on the driver — no special path needed.
        texture.create2D(width, height, fmt, mips);
        texture.uploadSubData2D(imageData.data(), 0, 0, width, height, GL_UNSIGNED_BYTE, 0);
        if (mips > 1) texture.generateMipmaps();
    }

    void createTextureArrayFromImages(
        Texture& texture,
        const fs::path& texturesFolderPath,
        const std::vector<std::string>& textureNames,
        const TextureLoadParams& params)
    {
        if (!validateChannels(params.desiredChannels)) return;

        if (!fs::exists(texturesFolderPath))
        {
            std::cerr << "[TextureLoader]: Textures folder not found: " << texturesFolderPath << "\n";
            return;
        }

        const size_t layerCount = textureNames.size();
        if (layerCount == 0) return;

        // Determine shared dimensions from the largest image in the set
        int sharedWidth = 0, sharedHeight = 0;
        for (const auto& name : textureNames)
        {
            const fs::path fullPath = texturesFolderPath / (name + ".png");
            int w, h, ch;
            if (stbi_info(fullPath.string().c_str(), &w, &h, &ch))
            {
                sharedWidth  = std::max(sharedWidth,  w);
                sharedHeight = std::max(sharedHeight, h);
            }
        }

        if (sharedWidth == 0 || sharedHeight == 0)
            sharedWidth = sharedHeight = 16;

        const GLenum fmt = TextureCompression::resolveInternalFormat(
            params.compression,
            toCompressionChannels(params.desiredChannels),
            params.isHDR);
        const int mips = calculateMipmapLevels(sharedWidth, sharedHeight, params.createMipmaps);

        texture.create2DArray(sharedWidth, sharedHeight, static_cast<uint16_t>(layerCount), fmt, mips);

        const std::vector<unsigned char> fallback =
            createUndefinedTexture(sharedWidth, sharedHeight, params.desiredChannels);

        std::vector<unsigned char> imageData;

        for (size_t i = 0; i < layerCount; ++i)
        {
            const fs::path fullPath = texturesFolderPath / (textureNames[i] + ".png");

            int width = 0, height = 0;
            const bool loaded = loadAndProcessImage(
                fullPath, params.desiredChannels, imageData, width, height);

            const unsigned char* uploadData = fallback.data();

            if (loaded && width == sharedWidth && height == sharedHeight)
            {
                uploadData = imageData.data();
            }
            else if (loaded)
            {
                std::cerr << "[TextureLoader]: Layer " << i << " (\"" << textureNames[i]
                    << "\") has mismatched dimensions (" << width << "x" << height
                    << " vs " << sharedWidth << "x" << sharedHeight
                    << "). Using fallback texture.\n";
            }

            texture.uploadSubData2DArray(
                uploadData,
                0, 0, static_cast<uint16_t>(i),
                static_cast<uint16_t>(sharedWidth),
                static_cast<uint16_t>(sharedHeight),
                GL_UNSIGNED_BYTE, 0);
        }

        if (mips > 1) texture.generateMipmaps();
    }

    void createTexture3DFromFloatData(
        Texture& texture,
        const std::vector<float>& data,
        int width, int height, int depth,
        const TextureLoadParams& params)
    {
        if (!validateChannels(params.desiredChannels)) return;

        const int expectedSize = width * height * depth * params.desiredChannels;
        if (static_cast<int>(data.size()) < expectedSize)
        {
            std::cerr << "[TextureLoader][createTexture3DFromFloatData]: "
                "Provided data size (" << data.size() << ") is smaller than expected ("
                << expectedSize << ").\n";
            return;
        }

        // Block-compressed formats are not valid for GL_TEXTURE_3D in OpenGL.
        if (params.compression != TextureCompression::Format::NONE)
        {
            std::cerr << "[TextureLoader][createTexture3DFromFloatData]: "
                "Block-compressed formats are not supported for 3D textures. "
                "Ignoring compression setting.\n";
        }

        GLenum internalFormat;
        switch (params.desiredChannels)
        {
        case 1:  internalFormat = GL_R32F;    break;
        case 2:  internalFormat = GL_RG32F;   break;
        case 3:  internalFormat = GL_RGB32F;  break;
        default: internalFormat = GL_RGBA32F; break;
        }

        const int mips = calculateMipmapLevels(width, height, depth, params.createMipmaps);

        texture.create3D(width, height, depth, internalFormat, mips);
        texture.uploadSubData3D(data.data(), 0, 0, 0, width, height, depth, GL_FLOAT, 0);
        if (mips > 1) texture.generateMipmaps();
    }
}