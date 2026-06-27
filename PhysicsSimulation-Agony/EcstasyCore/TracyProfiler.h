#pragma once
#include <tracy/Tracy.hpp>
#include <cstdint>

namespace Ecstasy
{
    enum class Color : uint32_t
    {
        White             = 0xFFFFFF,
        Black             = 0x010000,
        Red               = 0xFF0000,
        Green             = 0x00FF00,
        Blue              = 0x0000FF,
        Yellow            = 0xFFFF00,
        Magenta           = 0xFF00FF,
        Cyan              = 0x00FFFF,
        Orange            = 0xFFA500,
        Purple            = 0x800080,
        DarkGreen         = 0x008000,
        Navy              = 0x000080,
        Olive             = 0x808000,
        Maroon            = 0x800000,
        Teal              = 0x008080,
        Silver            = 0xC0C0C0,
        Gray              = 0x808080,
        Pink              = 0xFFC0CB,
        Gold              = 0xFFD700,
        Brown             = 0xA52A2A,
        LightBlue         = 0xADD8E6,
        LightGreen        = 0x90EE90,
        HotPink           = 0xFF69B4,
        IndianRed         = 0xCD5C5C,
        Indigo            = 0x4B0082,
        Chartreuse        = 0x7FFF00,
        Crimson           = 0xDC143C,
        DarkTurquoise     = 0x00CED1,
        DarkViolet        = 0x9400D3,
        OrangeRed         = 0xFF4500,
        SeaGreen          = 0x2E8B57,
        SteelBlue         = 0x4682B4,
        Chocolate         = 0xD2691E,
        YellowGreen       = 0x9ACD32,
        CornflowerBlue    = 0x6495ED,
        LightPink         = 0xFFB6C1,
        LightSeaGreen     = 0x20B2AA,
        LightSkyBlue      = 0x87CEFA,
        LightSlateGray    = 0x778899,
        LightSteelBlue    = 0xB0C4DE,
        LightYellow       = 0xFFFFE0,
        MediumSpringGreen = 0x00FA9A,
        MediumTurquoise   = 0x48D1CC,
        MediumVioletRed   = 0xC71585,
        MidnightBlue      = 0x191970,
        MintCream         = 0xF5FFFA,
        MistyRose         = 0xFFE4E1,
        Moccasin          = 0xFFE4B5,
        NavajoWhite       = 0xFFDEAD,
        OliveDrab         = 0x6B8E23,
        Tomato            = 0xFF6347,
        Turquoise         = 0x40E0D0,
        Violet            = 0xEE82EE,
        Wheat             = 0xF5DEB3
    };
}

#ifdef TRACY_ENABLE
    #define TRACY_SCOPE_N(name) ZoneScopedN(name)
    #define TRACY_SCOPE_NC(name, color) ZoneScopedNC(name, static_cast<uint32_t>(color))
#else
    #define TRACY_SCOPE_N(name)
    #define TRACY_SCOPE_NC(name, color)
#endif