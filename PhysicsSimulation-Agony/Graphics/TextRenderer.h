#pragma once
#include <glm/glm.hpp>
#include <memory>
#include <functional>

#include "OpenGLWrappers/Shader.h"
#include "OpenGLWrappers/ImmutableBuffer.h"
#include "OpenGLWrappers/VertexArray.h"
#include "OpenGLWrappers/Texture.h"

#include <ft2build.h>
#include FT_FREETYPE_H

struct Glyph
{
    static constexpr uint32_t INVALID_GLYPH_TEXTUREE_ID = -1;

    uint32_t textureID = INVALID_GLYPH_TEXTUREE_ID;
    glm::ivec2 size;    // Glyph size
    glm::ivec2 bearing; // Offset from baseline
    GLuint advance;     // Horizontal offset to next glyph

    Glyph() = default;
    Glyph(uint32_t textureID, const glm::ivec2& size, const glm::ivec2& bearing, GLuint advance);
    ~Glyph() = default;
};

struct Font
{
    robin_hood::unordered_flat_map<uint32_t, Glyph> glyphs;
    float fontSize = 0.0f;
    uint32_t glyphTextureCount = 0;

    Texture textureArray;

    Font() = default;
    ~Font() = default;

    Font(const Font& other) = delete;
    Font& operator=(const Font& other) = delete;

    Font(Font&& other) noexcept;
    Font& operator=(Font&& other) noexcept;

    glm::ivec2 getTextureArrayDims() const noexcept { return { textureArray.getWidth(), textureArray.getHeight() }; }
};

struct GlyphInstance
{
    glm::vec4 positionAndSize;
    glm::vec2 texSize;
    uint32_t textureID;
};

class TextRenderer
{
    struct CacheHeader
    {
        uint32_t magic = 0x464F4E54; // "FONT"
        uint32_t version = 1;
        uint32_t fontSize;
        glm::ivec2 textureArrayDims;
        uint32_t glyphCount;         // Count for font.glyphs (data)
        uint32_t glyphTextureCount;  // Count for texture array layers
    };

    struct CacheGlyphEntry
    {
        uint32_t codepoint;
        Glyph glyph;
    };
public:
    enum class TextAlignment
    {
        TopLeft,
        TopCenter,
        TopRight,
        MiddleLeft,
        MiddleCenter,
        MiddleRight,
        BottomLeft,
        BottomCenter,
        BottomRight
    };
private:
    using UTFDecoderFunction = std::function<uint32_t(const void*, size_t, size_t&)>;

    robin_hood::unordered_flat_map<std::string, Font> fonts;
    Font* currentFont = nullptr;

    VertexArray textVAO;
    ImmutableBuffer textVBO;
    ImmutableBuffer textInstanceVBO;

    Shader textShader;
    glm::mat4 projectionMatrix;

    size_t glyphInstanceBatchSize = 0;
    std::unique_ptr<GlyphInstance[]> glyphInstances;
    size_t glyphInstanceCount = 0;

    float left = 0.0f;
    float right = 1.0f;   // Default width
    float bottom = 0.0f;
    float top = 1.0f;

    TextRenderer();
    ~TextRenderer() = default;

    TextRenderer(const TextRenderer& other) = delete;
    TextRenderer& operator=(const TextRenderer& other) = delete;

    TextRenderer(TextRenderer&& other) = delete;
    TextRenderer& operator=(TextRenderer&& other) = delete;

    static TextRenderer& getInstance();

    static uint32_t decodeStdString(const void* byteBuffer, size_t bufferLength, size_t& index);
    static uint32_t decodeUTF8(const void* byteBuffer, size_t bufferLength, size_t& index);
    static uint32_t decodeUTF16(const void* byteBuffer, size_t bufferLength, size_t& index);
    static uint32_t decodeUTF32(const void* byteBuffer, size_t bufferLength, size_t& index);

    void flushGlyphs();
    void pushGlyph(const GlyphInstance& glyph);

    void createInstanceVBO(size_t glyphCount);

    static std::vector<uint8_t> loadGlyphs(FT_Face face, Font& font);

    static bool saveFontCache(const std::string& cachePath, const Font& font, const std::vector<uint8_t>& textureData);
    static bool loadFontCache(const std::string& cachePath, Font& font);

    static void finalizeFontTexture(Font& font);

    static void renderTextInternal(const void* text, size_t textLength, UTFDecoderFunction decoder, const float x, const float y, float rowHeight,
        const glm::vec3& color, TextAlignment alignment, const glm::vec2& bounds);

    void updateProjectionMatrixInternal();
public:
    static void init();

    static bool loadFont(const std::string& fontName, GLuint fontSize);

    static void setGlyphInstanceBatchSize(size_t size);

    static void setCurrentFont(const std::string& fontName);

    static void startTextRendering();

    static void renderText(const std::string& text, float x, float y, float rowHeight,
        const glm::vec3& color, TextAlignment alignment = TextAlignment::TopLeft, const glm::vec2& bounds = { 0.0f, 0.0f });

    static void renderText(const std::u8string& text, float x, float y, float rowHeight,
        const glm::vec3& color, TextAlignment alignment = TextAlignment::TopLeft, const glm::vec2& bounds = { 0.0f, 0.0f });

    static void renderText(const std::u16string& text, float x, float y, float rowHeight,
        const glm::vec3& color, TextAlignment alignment = TextAlignment::TopLeft, const glm::vec2& bounds = { 0.0f, 0.0f });

    static void renderText(const std::u32string& text, float x, float y, float rowHeight,
        const glm::vec3& color, TextAlignment alignment = TextAlignment::TopLeft, const glm::vec2& bounds = { 0.0f, 0.0f });

    static void setPixelCoordinateSpace(int width, int height);
    static void setCustomCoordinateSpace(float left, float right, float bottom, float top);
};