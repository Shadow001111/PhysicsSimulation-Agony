#include "UTFDecoder.h"

std::vector<uint32_t> UTFDecoder::decodeUTF8(const std::u8string& str)
{
    std::vector<uint32_t> result;
    result.reserve(str.length());

    size_t index = 0;
    const char8_t* buffer = str.data();
    size_t length = str.length();

    while (index < length)
    {
        result.push_back(decodeUTF8CodePoint(buffer, length, index));
    }

    return result;
}

std::vector<uint32_t> UTFDecoder::decodeUTF16(const std::u16string& str)
{
    std::vector<uint32_t> result;
    result.reserve(str.length());

    size_t index = 0;
    const char16_t* buffer = str.data();
    size_t length = str.length();

    while (index < length)
    {
        result.push_back(decodeUTF16CodePoint(buffer, length, index));
    }

    return result;
}

std::vector<uint32_t> UTFDecoder::decodeUTF32(const std::u32string& str)
{
    const uint32_t* begin = reinterpret_cast<const uint32_t*>(str.data());
    const uint32_t* end = begin + str.length();

    return std::vector<uint32_t>(begin, end);
}

uint32_t UTFDecoder::decodeUTF8CodePoint(const char8_t* buffer, size_t length, size_t& index)
{
    if (index >= length) return INVALID_CODEPOINT;

    uint8_t c = static_cast<uint8_t>(buffer[index]);

    // 1-byte ASCII (0xxxxxxx)
    if (c < 0x80)
    {
        return static_cast<uint8_t>(buffer[index++]);
    }

    // 2-byte sequence
    if ((c & 0xE0) == 0xC0)
    {
        if (index + 1 >= length)
        {
            index = length;
            return INVALID_CODEPOINT;
        }
        uint32_t cp = ((c & 0x1F) << 6) | (static_cast<uint8_t>(buffer[index + 1]) & 0x3F);
        index += 2;
        return cp < 0x80 ? INVALID_CODEPOINT : cp;
    }

    // 3-byte sequence
    if ((c & 0xF0) == 0xE0)
    {
        if (index + 2 >= length)
        {
            index = length;
            return INVALID_CODEPOINT;
        }
        uint32_t cp = ((c & 0x0F) << 12) | ((static_cast<uint8_t>(buffer[index + 1]) & 0x3F) << 6) |
            (static_cast<uint8_t>(buffer[index + 2]) & 0x3F);
        index += 3;
        return cp < 0x800 ? INVALID_CODEPOINT : cp;
    }

    // 4-byte sequence
    if ((c & 0xF8) == 0xF0)
    {
        if (index + 3 >= length)
        {
            index = length;
            return INVALID_CODEPOINT;
        }
        uint32_t cp = ((c & 0x07) << 18) | ((static_cast<uint8_t>(buffer[index + 1]) & 0x3F) << 12) |
            ((static_cast<uint8_t>(buffer[index + 2]) & 0x3F) << 6) |
            (static_cast<uint8_t>(buffer[index + 3]) & 0x3F);
        index += 4;
        return (cp < 0x10000 || cp > 0x10FFFF) ? INVALID_CODEPOINT : cp;
    }

    // Invalid byte
    index++;
    return INVALID_CODEPOINT;
}

uint32_t UTFDecoder::decodeUTF16CodePoint(const char16_t* buffer, size_t length, size_t& index)
{
    if (index >= length) return INVALID_CODEPOINT;

    uint16_t first = static_cast<uint16_t>(buffer[index]);

    // Single UTF-16 code unit (BMP character)
    if (first < 0xD800 || first > 0xDFFF)
    {
        return buffer[index++];
    }

    // High surrogate (first of pair)
    if (first >= 0xD800 && first <= 0xDBFF)
    {
        if (index + 1 >= length)
        {
            index = length;
            return INVALID_CODEPOINT;
        }

        uint16_t second = static_cast<uint16_t>(buffer[index + 1]);

        // Check if second is a low surrogate
        if (second >= 0xDC00 && second <= 0xDFFF)
        {
            // Decode surrogate pair
            uint32_t cp = 0x10000 + ((first - 0xD800) << 10) + (second - 0xDC00);
            index += 2;
            return cp;
        }
    }

    // Invalid: lone surrogate or wrong order
    index++;
    return INVALID_CODEPOINT;
}

uint32_t UTFDecoder::decodeUTF32CodePoint(const char32_t* buffer, size_t length, size_t& index)
{
    if (index >= length) return INVALID_CODEPOINT;

    uint32_t cp = static_cast<uint32_t>(buffer[index++]);

    // Optional validation (commented out for speed)
    // return (cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) ? INVALID_CODEPOINT : cp;

    return cp;
}