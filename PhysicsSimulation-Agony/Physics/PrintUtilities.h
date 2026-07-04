#pragma once
#include "GlmTypes.h"

#include <iostream>

namespace PS_AGONY
{
    std::ostream& operator<<(std::ostream& os, const Vec2& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const Vec3& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const Vec4& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";
        return os;
    }
}