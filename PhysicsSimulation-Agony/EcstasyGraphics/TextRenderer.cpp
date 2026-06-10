#include "TextRenderer.h"

#include <glm/gtc/matrix_transform.hpp>

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/FileStream.h"
#include "EcstasyCore/Decoding/UTFDecoder.h"


#include <iostream>

struct GlPixelAligmentRAII
{
    GLint oldAlignment;

    GlPixelAligmentRAII(int newAlignment)
    {
        glGetIntegerv(GL_UNPACK_ALIGNMENT, &oldAlignment);
        glPixelStorei(GL_UNPACK_ALIGNMENT, newAlignment);
    }

    ~GlPixelAligmentRAII()
    {
        glPixelStorei(GL_UNPACK_ALIGNMENT, oldAlignment);
    }
};


class FreeTypeLibrary
{
    FT_Library lib = nullptr;
    FT_Face face = nullptr;
public:
    bool initLibrary()
    {
        TRACY_SCOPE_N("Init FreeType library");

        if (FT_Init_FreeType(&lib))
        {
            return false;
        }

        return true;
    }

    bool initFace(const char* fontPath)
    {
        TRACY_SCOPE_N("Init FreeType face");

        if (FT_New_Face(lib, fontPath, 0, &face))
        {
            return false;
        }

        return true;
    }

    ~FreeTypeLibrary()
    {
        TRACY_SCOPE_N("Free FreeType resources");

        if (face)
        {
            FT_Done_Face(face);
            face = nullptr;
        }

        if (lib)
        {
            FT_Done_FreeType(lib);
            lib = nullptr;
        }
    }

    FT_Face getFace() noexcept { return face; }
};


Glyph::Glyph(uint32_t textureID, const glm::ivec2& size, const glm::ivec2& bearing, GLuint advance) :
    textureID(textureID), size(size), bearing(bearing), advance(advance)
{
}


Font::Font(Font&& other) noexcept :
    glyphs(std::move(other.glyphs)),
    fontSize(other.fontSize),
    textureArray(std::move(other.textureArray))
{
}

Font& Font::operator=(Font&& other) noexcept
{
    if (this != &other)
    {
        glyphs = std::move(other.glyphs);
        fontSize = other.fontSize;
        textureArray = std::move(other.textureArray);
    }
    return *this;
}


TextRenderer& TextRenderer::getInstance()
{
    static TextRenderer textRenderer;
    return textRenderer;
}

uint32_t TextRenderer::decodeStdString(const void* byteBuffer, size_t bufferLength, size_t& index)
{
    if (index >= bufferLength) return Ecstasy::Decoding::INVALID_UTF_CODEPOINT;
    const uint8_t* text_ = static_cast<const uint8_t*>(byteBuffer);
    return text_[index++];
}

uint32_t TextRenderer::decodeUTF8(const void* byteBuffer, size_t bufferLength, size_t& index)
{
    return Ecstasy::Decoding::decodeUTF8CodePoint(static_cast<const char8_t*>(byteBuffer), bufferLength, index);
}

uint32_t TextRenderer::decodeUTF16(const void* byteBuffer, size_t bufferLength, size_t& index)
{
    return Ecstasy::Decoding::decodeUTF16CodePoint(static_cast<const char16_t*>(byteBuffer), bufferLength, index);
}

uint32_t TextRenderer::decodeUTF32(const void* byteBuffer, size_t bufferLength, size_t& index)
{
    return Ecstasy::Decoding::decodeUTF32CodePoint(static_cast<const char32_t*>(byteBuffer), bufferLength, index);
}

TextRenderer::TextRenderer() :
    projectionMatrix(glm::identity<glm::mat4>())
{
    // Shaders
    std::vector<Shader::ShaderSource> textShaderSources =
    {
        {GL_VERTEX_SHADER, "res/Shaders/text.vert"},
        {GL_FRAGMENT_SHADER, "res/Shaders/text.frag"}
    };
    textShader.create(textShaderSources);
    textShader.setInt("glyphTextureArray", 0);
    textShader.setMat4("projection", projectionMatrix);

    // Buffers
    glm::vec2 vertices[4] = // CCW order
    {
        { 0.0f, 0.0f },
        { 1.0f, 0.0f },
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    // Create VAO
    textVAO.create();

    // Create VBO
    textVBO.create();
    textVBO.allocateStorage(sizeof(vertices), 0, vertices);

    // Bind VBO to VAO
    textVAO.bindVertexBuffer(0, textVBO.getID(), 0, sizeof(vertices[0]));

    textVAO.enableAttribute(0);
    textVAO.setFloatAttribute(0, 2, 0, 0);
}

void TextRenderer::flushGlyphs()
{
    if (glyphInstanceCount == 0)
    {
        return;
    }

    // Move glyph data to GPU
    textInstanceVBO.write(glyphInstances.get(), glyphInstanceCount * sizeof(GlyphInstance));

    // Draw glyphs
    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, glyphInstanceCount);

    // Reset glyphs count
    glyphInstanceCount = 0;
}

void TextRenderer::pushGlyph(const GlyphInstance& glyph)
{
    glyphInstances[glyphInstanceCount++] = glyph;
    if (glyphInstanceCount >= glyphInstanceBatchSize)
    {
        flushGlyphs();
    }
}

void TextRenderer::createInstanceVBO(size_t glyphCount)
{
    // Create instance VBO
    textInstanceVBO.create();
    textInstanceVBO.allocateStorage(sizeof(GlyphInstance) * glyphCount, GL_DYNAMIC_STORAGE_BIT);

    // Bind instance VBO to VAO
    textVAO.bindVertexBuffer(1, textInstanceVBO.getID(), 0, sizeof(GlyphInstance));

    textVAO.enableAttribute(1);
    textVAO.setFloatAttribute(1, 4, 0, 1);
    textVAO.setAttributeDivisor(1, 1);

    textVAO.enableAttribute(2);
    textVAO.setFloatAttribute(2, 2, 4 * sizeof(float), 1);
    textVAO.setAttributeDivisor(2, 1);

    textVAO.enableAttribute(3);
    textVAO.setIntAttribute(3, 1, 6 * sizeof(float), 1);
    textVAO.setAttributeDivisor(3, 1);
}

std::vector<uint8_t> TextRenderer::loadGlyphs(FT_Face face, Font& font)
{
    TRACY_SCOPE_N("Load glyphs");

    // Pre-pass
    uint32_t glyphDataCount = 0;
    font.glyphTextureCount = 0;

    FT_UInt gindex;
    FT_ULong charcode = FT_Get_First_Char(face, &gindex);
    while (gindex != 0)
    {
        glyphDataCount++;
        if (charcode > ' ') font.glyphTextureCount++;
        charcode = FT_Get_Next_Char(face, charcode, &gindex);
    }
    font.glyphs.reserve(glyphDataCount);

    // Create texture array
    {
        float scale = (float)font.fontSize / (float)face->units_per_EM;
        int width = (int)std::ceil((face->bbox.xMax - face->bbox.xMin) * scale);
        int height = (int)std::ceil((face->bbox.yMax - face->bbox.yMin) * scale);
    
        font.textureArray.create2DArrayCompressed(
            width, height, font.glyphTextureCount,
            TextureCompression::Channels::R,
            TextureCompression::Format::AUTO
        );
    }

    const size_t arrayWidth = font.textureArray.getWidth();
    const size_t arrayHeight = font.textureArray.getHeight();
    const size_t arrayDepth = font.glyphTextureCount;

    const size_t layerSizeInBytes = arrayWidth * arrayHeight;
    std::vector<uint8_t> textureData(layerSizeInBytes * arrayDepth, 0);

    // Main pass
    uint32_t textureIdCounter = 0;
    charcode = FT_Get_First_Char(face, &gindex);
    while (gindex != 0)
    {
        {
            TRACY_SCOPE_N("Load char");
            if (FT_Load_Char(face, charcode, FT_LOAD_RENDER)) [[unlikely]]
            {
                std::cerr << "[TextRenderer]: Failed to load glyph: '" << charcode << "'.\n";
                charcode = FT_Get_Next_Char(face, charcode, &gindex);
                continue;
            }
        }

        uint32_t glyphTextureId = Glyph::INVALID_GLYPH_TEXTUREE_ID;
        if (charcode > ' ')
        {
            TRACY_SCOPE_N("Process char");

            glyphTextureId = textureIdCounter++;

            const FT_Bitmap& bitmap = face->glyph->bitmap;

            const int srcW = (int)bitmap.width;
            const int srcH = (int)bitmap.rows;
            const int pitch = std::abs(bitmap.pitch);

            uint8_t* destSlice = textureData.data() + glyphTextureId * layerSizeInBytes;
            const uint8_t* srcBuffer = bitmap.buffer;

            for (int row = 0; row < srcH; row++)
            {
                uint8_t* destRow = destSlice + (row * arrayWidth);
                const uint8_t* srcRow = srcBuffer + (row * pitch);

                if (bitmap.pixel_mode == FT_PIXEL_MODE_MONO)
                {
                    // 1-bit packed -> expand to 8-bit
                    for (int col = 0; col < srcW; col++)
                    {
                        destRow[col] = (srcRow[col >> 3] & (0x80 >> (col & 7))) ? 255 : 0;
                    }
                }
                else
                {
                    // Assume 8-bit grayscale (FT_PIXEL_MODE_GRAY)
                    std::memcpy(destRow, srcRow, srcW);
                }
            }
        }

        Glyph glyph =
        {
            glyphTextureId,
            glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
            glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
            (GLuint)face->glyph->advance.x
        };

        font.glyphs.emplace(charcode, glyph);

        {
            TRACY_SCOPE_N("Next char");
            charcode = FT_Get_Next_Char(face, charcode, &gindex);
        }
    }

    {
        TRACY_SCOPE_N("Upload data to gpu");
        font.textureArray.uploadSubData3D(
            textureData.data(),
            0, 0, 0,
            arrayWidth, arrayHeight, arrayDepth,
            GL_UNSIGNED_BYTE
        );
    }

    return textureData;
}

bool TextRenderer::saveFontCache(const std::string& cachePath, const Font& font, const std::vector<uint8_t>& textureData)
{
    TRACY_SCOPE_N("Save font cache");

    std::error_code ec;
    std::filesystem::create_directories("cache", ec);
    if (ec)
    {
        std::cerr << "[TextRenderer][saveFontCache]: Failed to create cache folder\n";
        return false;
    }

    Ecstasy::FileStream file(cachePath, Ecstasy::FileStream::Mode::Write);
    if (!file)
    {
        std::cerr << "[TextRenderer][saveFontCache]: Failed to create/open file\n";
        return false;
    }

    CacheHeader header;
    header.fontSize = static_cast<uint32_t>(font.fontSize);
    header.textureArrayDims = font.getTextureArrayDims();
    header.glyphCount = font.glyphs.size();
    header.glyphTextureCount = font.glyphTextureCount;

    file.writeObjects(&header);

    // Write glyph map
    for (const auto& [codepoint, glyph] : font.glyphs)
    {
        CacheGlyphEntry entry = { codepoint, glyph };
        file.writeObjects(&entry);
    }

    // Save textures
    file.writeBytes(textureData.data(), textureData.size());

    return true;
}

bool TextRenderer::loadFontCache(const std::string& cachePath, Font& font)
{
    TRACY_SCOPE_N("Load font cache");

    Ecstasy::FileStream file(cachePath, Ecstasy::FileStream::Mode::Read);
    if (!file) return false;

    CacheHeader header;
    file.readObjects(&header);

    if (header.magic != 0x464F4E54 || header.version != 1) return false;

    font.fontSize = header.fontSize;
    font.glyphTextureCount = header.glyphTextureCount;

    // Load glyph Map
    font.glyphs.reserve(header.glyphCount);
    for (uint32_t i = 0; i < header.glyphCount; i++)
    {
        CacheGlyphEntry entry;
        file.readObjects(&entry);
        font.glyphs.emplace(entry.codepoint, entry.glyph);
    }

    // Load Texture Data and upload to GPU in one go
    size_t textureSize = header.textureArrayDims.x * header.textureArrayDims.y * header.glyphTextureCount;
    std::vector<uint8_t> pixels(textureSize);
    file.readBytes(pixels.data(), textureSize);

    font.textureArray.create2DArrayCompressed(
        header.textureArrayDims.x, header.textureArrayDims.y, header.glyphTextureCount,
        TextureCompression::Channels::R,
        TextureCompression::Format::AUTO
    );

    // Upload data to texture array
    font.textureArray.uploadSubData3D(
        pixels.data(),
        0, 0, 0,
        header.textureArrayDims.x, header.textureArrayDims.y, header.glyphTextureCount,
        GL_UNSIGNED_BYTE
    );

    return true;
}

void TextRenderer::finalizeFontTexture(Font& font)
{
    const Texture::Parameters defaultParams{
        .minFilter = GL_NEAREST,
        .magFilter = GL_NEAREST,
        .wrapS = GL_CLAMP_TO_EDGE,
        .wrapT = GL_CLAMP_TO_EDGE
    };
    font.textureArray.setParameters(defaultParams);

    if (Texture::getExtensions().bindless)
    {
        font.textureArray.initHandle();
        font.textureArray.makeResident();
    }
}

void TextRenderer::renderTextInternal(const void* text, size_t textLength, UTFDecoderFunction decoder, const float x, const float y, float rowHeight,
    const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds)
{
    TRACY_SCOPE_N("Render text");

    TextRenderer& inst = getInstance();

    // Check buffer size
    if (inst.glyphInstanceBatchSize == 0)
    {
        std::cerr << "[TextRenderer][renderText]: Glyph instance batch size is zero\n";
        return;
    }

    // Check if font is set
    const Font* font = inst.currentFont;
    if (!font)
    {
        std::cerr << "[TextRenderer][renderText]: Current font is not set\n";
        return;
    }

    // Get some data
    const auto& glyphs = font->glyphs;
    float scale = rowHeight / font->fontSize;
    const glm::vec2 invMaxGlyphSize = 1.0f / glm::vec2(font->getTextureArrayDims());

    // Decode text
    static std::vector<uint32_t> codepoints;
    codepoints.clear();
    codepoints.reserve(textLength);

    size_t index = 0;
    float currentLineWidth = 0.0f;
    float maxLineWidth = 0.0f;
    int lineCount = 1;

    constexpr uint32_t replaceCodepoint = '?';
    const auto replaceCodepointIT = glyphs.find(replaceCodepoint);
    const bool hasReplacement = replaceCodepointIT != glyphs.end();

    {
        TRACY_SCOPE_N("Decode text");
        while (index < textLength)
        {
            uint32_t codepoint = decoder(text, textLength, index);
            if (codepoint == Ecstasy::Decoding::INVALID_UTF_CODEPOINT) break;
            else if (codepoint == '\n')
            {
                // Update max line width for current line
                maxLineWidth = std::fmax(maxLineWidth, currentLineWidth);
                currentLineWidth = 0.0f;
                lineCount++;
                codepoints.push_back(codepoint);
                continue;
            }

            // Check if glyph exists, replace if not
            auto it = glyphs.find(codepoint);
            if (it == glyphs.end())
            {
                if (!hasReplacement) continue;
                codepoint = replaceCodepoint;
                it = replaceCodepointIT;
            }

            currentLineWidth += (it->second.advance >> 6) * scale;

            codepoints.push_back(codepoint);
        }
        maxLineWidth = std::fmax(maxLineWidth, currentLineWidth);
    }

    // Apply bounds scaling
    float textWidth = maxLineWidth;
    float textHeight = lineCount * rowHeight;

    if ((bounds.x > 0.0f && textWidth > bounds.x) || (bounds.y > 0.0f && textHeight > bounds.y))
    {
        float widthScale = (bounds.x > 0.0f) ? bounds.x / textWidth : 1.0f;
        float heightScale = (bounds.y > 0.0f) ? bounds.y / textHeight : 1.0f;
        float fitScale = std::fmin(widthScale, heightScale);

        scale *= fitScale;
        rowHeight *= fitScale;
        textWidth *= fitScale;
        textHeight *= fitScale;
    }

    // Calculate alignment offset
    glm::vec2 alignmentOffset(0.0f);

    if (alignment != TextAlignment::TopLeft)
    {
        // Horizontal alignment
        switch (alignment)
        {
        case TextAlignment::TopCenter:
        case TextAlignment::MiddleCenter:
        case TextAlignment::BottomCenter:
            alignmentOffset.x = -textWidth * 0.5f;
            break;
        case TextAlignment::TopRight:
        case TextAlignment::MiddleRight:
        case TextAlignment::BottomRight:
            alignmentOffset.x = -textWidth;
            break;
        default:
            break;
        }

        // Vertical alignment
        switch (alignment)
        {
        case TextAlignment::MiddleLeft:
        case TextAlignment::MiddleCenter:
        case TextAlignment::MiddleRight:
            alignmentOffset.y = textHeight * 0.5f;
            break;
        case TextAlignment::BottomLeft:
        case TextAlignment::BottomCenter:
        case TextAlignment::BottomRight:
            alignmentOffset.y = textHeight;
            break;
        default:
            break;
        }
    }

    float currentX = x + alignmentOffset.x;
    float currentY = y + alignmentOffset.y - rowHeight;

    // Shader
    const auto& textShader = inst.textShader;
    textShader.setVec3("textColor", color.x, color.y, color.z);

    // Render glyphs
    const float startX = currentX;

    {
        TRACY_SCOPE_N("Render glyphs");
        for (uint32_t codepoint : codepoints)
        {
            if (codepoint == '\n')
            {
                currentY -= rowHeight;
                currentX = startX;
                continue;
            }

            auto it = glyphs.find(codepoint);
            if (it == glyphs.end()) [[unlikely]]
            {
                continue; // Should not happen since we filtered already
            }

            const Glyph& glyph = it->second;
            if (glyph.textureID != Glyph::INVALID_GLYPH_TEXTUREE_ID)
            {
                float xpos = currentX + glyph.bearing.x * scale;
                float ypos = currentY - (glyph.size.y - glyph.bearing.y) * scale;
                float w = glyph.size.x * scale;
                float h = glyph.size.y * scale;

                GlyphInstance glyphInstance = {
                    glm::vec4(xpos, ypos, w, h),
                    glm::vec2(glyph.size.x * invMaxGlyphSize.x, glyph.size.y * invMaxGlyphSize.y),
                    glyph.textureID
                };

                inst.pushGlyph(glyphInstance);
            }

            currentX += (glyph.advance >> 6) * scale;
        }

        inst.flushGlyphs();
    }
}

void TextRenderer::updateProjectionMatrixInternal()
{
    TextRenderer& inst = getInstance();
    auto& proj = inst.projectionMatrix;
    const auto& textShader = inst.textShader;

    proj = glm::ortho(inst.left, inst.right, inst.bottom, inst.top);
    textShader.setMat4("projection", proj);
}

void TextRenderer::init()
{
    TRACY_SCOPE_N("Init text renderer");

    getInstance();
}

bool TextRenderer::loadFont(const std::string& fontName, GLuint fontSize)
{
    TRACY_SCOPE_N("Load font");

    TextRenderer& inst = getInstance();
    auto& fonts = inst.fonts;

    // Test if font exists
    if (fonts.find(fontName) != fonts.end())
    {
        std::cerr << "[TextRenderer][loadFont]: Font '" << fontName << "' already exists\n";
        return false;
    }

    GlPixelAligmentRAII pal(1);

    // Try loading from cache first
    const std::string cachePath = "cache/" + fontName + "_" + std::to_string(fontSize) + ".fcache";
    Font font;
    font.fontSize = fontSize;
    if (inst.loadFontCache(cachePath, font))
    {
        std::cout << "[TextRenderer]: Loaded '" << fontName << "' from cache.\n";
    
        finalizeFontTexture(font);
    
        inst.fonts.emplace(fontName, std::move(font));
        return true;
    }

    // Init FreeType
    FreeTypeLibrary freeTypeLibrary;
    if (!freeTypeLibrary.initLibrary())
    {
        std::cerr << "[TextRenderer][loadFont]: Failed to init FreeType Library\n";
        return false;
    }

    const std::string fontPath = "res/fonts/" + fontName + ".ttf";

    if (!freeTypeLibrary.initFace(fontPath.c_str()))
    {
        std::cerr << "[TextRenderer][loadFont]: Failed to load font: " << fontPath << "\n";
        return false;
    }

    auto face = freeTypeLibrary.getFace();
    FT_Set_Pixel_Sizes(face, 0, fontSize);

    // Load glyphs
    std::vector<uint8_t> textureData = loadGlyphs(face, font);

	//
    finalizeFontTexture(font);

	// Save the cache for next time
    if (inst.saveFontCache(cachePath, font, textureData))
    {
        std::cout << "[TextRenderer][loadFont]: Saved '" << fontName << "' to cache.\n";
    }

	// Store font
    fonts.emplace(fontName, std::move(font));

    return true;
}

void TextRenderer::setGlyphInstanceBatchSize(size_t size)
{
    TextRenderer& inst = getInstance();
    inst.glyphInstanceBatchSize = size;
    if (size == 0)
    {
        inst.glyphInstances.reset();
        inst.textInstanceVBO.destroy();
    }
    else
    {
        inst.glyphInstances = std::unique_ptr<GlyphInstance[]>(new GlyphInstance[size]);
        inst.createInstanceVBO(size);
    }
}

void TextRenderer::setCurrentFont(const std::string& fontName)
{
    TextRenderer& inst = getInstance();
    auto& fonts = inst.fonts;

    auto it = fonts.find(fontName);
    if (it == fonts.end())
    {
        std::cerr << "[TextRenderer]: Font '" << fontName << "' isn't loaded.\n";
        return;
    }

    Font* previousFont = inst.currentFont;
    inst.currentFont = &it->second;


    // Pass texture handle to shader
    if (Texture::getExtensions().bindless)
    {
        inst.textShader.setHandleui64ARB("glyphTextureArray", inst.currentFont->textureArray.getHandle());
    }
    
	// Make previous font texture non-resident if needed
    if (previousFont && previousFont != inst.currentFont && Texture::getExtensions().bindless)
    {
        previousFont->textureArray.makeNonResident();
    }
}

void TextRenderer::startTextRendering()
{
    TextRenderer& inst = getInstance();

    const Font* font = inst.currentFont;
    if (!font)
    {
        std::cerr << "[TextRenderer][renderText]: Current font is not set\n";
        return;
    }

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    inst.textVAO.bind();
	if (!Texture::getExtensions().bindless)
    {
        font->textureArray.bindUnit(0);
    }

    const auto& textShader = inst.textShader;
    textShader.use();
}

void TextRenderer::renderText(const std::string& text, float x, float y, float rowHeight,
    const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds)
{
    renderTextInternal(text.c_str(), text.length(), decodeStdString, x, y, rowHeight, color, alignment, bounds);
}

void TextRenderer::renderText(const std::u8string& text, float x, float y, float rowHeight,
    const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds)
{
    renderTextInternal(text.c_str(), text.length(), decodeUTF8, x, y, rowHeight, color, alignment, bounds);
}

void TextRenderer::renderText(const std::u16string& text, float x, float y, float rowHeight,
    const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds)
{
    renderTextInternal(text.c_str(), text.length(), decodeUTF16, x, y, rowHeight, color, alignment, bounds);
}

void TextRenderer::renderText(const std::u32string& text, float x, float y, float rowHeight,
    const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds)
{
    renderTextInternal(text.c_str(), text.length(), decodeUTF32, x, y, rowHeight, color, alignment, bounds);
}

void TextRenderer::setPixelCoordinateSpace(int width, int height)
{
    setCustomCoordinateSpace(
        0.0f, static_cast<float>(width),
        0.0f, static_cast<float>(height)
    );
}

void TextRenderer::setCustomCoordinateSpace(float left, float right, float bottom, float top)
{
    TextRenderer& inst = getInstance();

    if (left >= right || bottom >= top)
    {
        std::cerr << "[TextRenderer][setCustomCoordinateSpace]: Invalid bounds\n";
        return;
    }

    inst.left = left;
    inst.right = right;
    inst.bottom = bottom;
    inst.top = top;

    inst.updateProjectionMatrixInternal();
}
