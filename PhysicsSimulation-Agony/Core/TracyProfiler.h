#pragma once
#include <tracy/Tracy.hpp>
#include <cstdint>

constexpr static uint32_t getTracyColor(uint32_t category)
{
    constexpr uint32_t colors[] = {
    0xFF0000, // Red
    0x00FF00, // Green
    0x0000FF, // Blue
    0xFFFF00, // Yellow
    0xFF00FF, // Magenta
    0x00FFFF, // Cyan
    0xFFA500, // Orange
    0x800080, // Purple
    0x008000, // Dark Green
    0x000080, // Navy
    0x808000, // Olive
    0x800000, // Maroon
    0x008080, // Teal
    0xC0C0C0, // Silver
    0x808080, // Gray
    0xFFC0CB, // Pink
    0xFFD700, // Gold
    0xA52A2A, // Brown
    0xADD8E6, // Light Blue
    0x90EE90, // Light Green
    0xFF69B4, // Hot Pink
    0xCD5C5C, // Indian Red
    0x4B0082, // Indigo
    0x7FFF00, // Chartreuse
    0xDC143C, // Crimson
    0x00CED1, // Dark Turquoise
    0x9400D3, // Dark Violet
    0xFF4500, // Orange Red
    0x2E8B57, // Sea Green
    0x4682B4, // Steel Blue
    0xD2691E, // Chocolate
    0x9ACD32, // Yellow Green
    0x6495ED, // Cornflower Blue
    0xFFB6C1, // Light Pink
    0x20B2AA, // Light Sea Green
    0x87CEFA, // Light Sky Blue
    0x778899, // Light Slate Gray
    0xB0C4DE, // Light Steel Blue
    0xFFFFE0, // Light Yellow
    0x00FA9A, // Medium Spring Green
    0x48D1CC, // Medium Turquoise
    0xC71585, // Medium Violet Red
    0x191970, // Midnight Blue
    0xF5FFFA, // Mint Cream
    0xFFE4E1, // Misty Rose
    0xFFE4B5, // Moccasin
    0xFFDEAD, // Navajo White
    0x6B8E23, // Olive Drab
    0xFF6347, // Tomato
    0x40E0D0, // Turquoise
    0xEE82EE, // Violet
    0xF5DEB3  // Wheat
    };

    constexpr size_t colorCount = sizeof(colors) / sizeof(colors[0]);

    return colors[static_cast<size_t>(category) % colorCount];
}

#ifdef TRACY_ENABLE
#define TRACY_SCOPE_N(name) ZoneScopedN(name)
#define TRACY_SCOPE_NC(name, category) ZoneScopedNC(name, getTracyColor(static_cast<uint32_t>(category)))
#else
#define TRACY_SCOPE_N(name)
#define TRACY_SCOPE_NC(name, category)
#endif