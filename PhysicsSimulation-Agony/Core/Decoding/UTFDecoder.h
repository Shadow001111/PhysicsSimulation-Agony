#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace UTFDecoder
{
    constexpr uint32_t INVALID_CODEPOINT = -1;

    std::vector<uint32_t> decodeUTF8(const std::u8string& str);
    std::vector<uint32_t> decodeUTF16(const std::u16string& str);
    std::vector<uint32_t> decodeUTF32(const std::u32string& str);

    uint32_t decodeUTF8CodePoint(const char8_t* buffer, size_t length, size_t& index);
    uint32_t decodeUTF16CodePoint(const char16_t* buffer, size_t length, size_t& index);
    uint32_t decodeUTF32CodePoint(const char32_t* buffer, size_t length, size_t& index);
}