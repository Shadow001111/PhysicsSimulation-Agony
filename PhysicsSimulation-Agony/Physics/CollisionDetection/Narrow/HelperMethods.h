#pragma once
#include "Constants.h"

#include "Physics/Types/Types.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <bit>

namespace PS_AGONY
{
    // Next index with wrap-around; replaces a runtime integer modulo.
    template <typename Index>
    [[nodiscard]] inline Index nextIndex(Index index, Index count)
    {
        return index + 1 == count ? Index(0) : Index(index + 1);
    }

    // Builds a 4-lane vector in registers. 'a' goes to lane 0 (Simd::set loads in reverse order).
    inline Ecstasy::Core::Simd<Real, sizeof(Real) * 32> makeReal4(Real a, Real b, Real c, Real d)
    {
        return Ecstasy::Core::Simd<Real, sizeof(Real) * 32>::set(d, c, b, a);
    }

    inline void flipSignIfNegative(Vec2& v, const Real& sign)
    {
        using Int = std::conditional_t<sizeof(Real) == 8, uint64_t, uint32_t>;
        constexpr Int signBit = Int(1) << (sizeof(Real) * 8 - 1);

        const Int signMask = std::bit_cast<Int>(sign) & signBit;
        v.x = std::bit_cast<Real>(std::bit_cast<Int>(v.x) ^ signMask);
        v.y = std::bit_cast<Real>(std::bit_cast<Int>(v.y) ^ signMask);
    }

    [[nodiscard]] inline bool clipSegment(Vec2& p1, Vec2& p2, Vec2 planePoint, Vec2 planeNormal)
    {
        const Real d1 = glm::dot(p1 - planePoint, planeNormal);
        const Real d2 = glm::dot(p2 - planePoint, planeNormal);

        if (d1 >= 0 && d2 >= 0) return false; // Both inside.
        if (d1 < 0 && d2 < 0) return true;  // Both outside.

        const Real t = d1 / (d1 - d2);
        const Vec2 dir = p2 - p1;
        const Vec2 intersect = p1 + dir * t;

        if (d1 >= 0) p2 = intersect;
        else         p1 = intersect;

        return false;
    }

    inline void projectVerticesOnAxis(
        const Vec2* ECSTASY_RESTRICT verticesPtr,
        size_t vertexCount,
        const Vec2& axis,
        Real& ECSTASY_RESTRICT minOut,
        Real& ECSTASY_RESTRICT maxOut
    )
    {
        minOut = std::numeric_limits<Real>::max();
        maxOut = -std::numeric_limits<Real>::max();

        for (size_t i = 0; i < vertexCount; i++)
        {
            const Real proj = glm::dot(verticesPtr[i], axis);
            minOut = std::fmin(minOut, proj);
            maxOut = std::fmax(maxOut, proj);
        }
    }

    [[nodiscard]] inline Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b)
    {
        const Vec2 ab = b - a;
        const Real denom = glm::dot(ab, ab);
        if (denom < ZERO_DIVISION_BOUNDARY_SQUARED) return a;
        const Real t = glm::dot(p - a, ab) / denom;
        return a + ab * std::fmin(std::fmax(t, Real(0)), Real(1));
    }

    // Finds the polygon edge with the greatest signed separation from 'point' (polygon-local space).
    // Returns false as soon as any edge separates the point by more than 'maxDistance': no contact possible.
    [[nodiscard]] inline bool findSupportEdge(
        const Vec2* vertsPtr,
        const Vec2* normalsPtr,
        size_t vertexCount,
        const Vec2& point,
        Real maxDistance,
        uint32_t& supportEdgeOut,
        Real& maxSeparationOut
    )
    {
        Real maxSeparation = -std::numeric_limits<Real>::max();
        uint32_t supportEdge = 0;
        for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
        {
            const Real separation = glm::dot(normalsPtr[i], point - vertsPtr[i]);
            if (separation > maxDistance) return false;
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
                supportEdge = i;
            }
        }
        supportEdgeOut = supportEdge;
        maxSeparationOut = maxSeparation;
        return true;
    }

    [[nodiscard]] inline __forceinline Real clampValue(Real value, Real lo, Real hi)
    {
        return std::fmin(std::fmax(value, lo), hi);
    }
}