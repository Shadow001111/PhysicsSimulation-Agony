#include "PhysicsGeometry.h"

namespace PS_AGONY::PhysicsGeometry
{
    Real calculateCircleInertia(Real mass, Real radius, Vec2 centerOfMass)
    {
        const Real radiusSquared = radius * radius;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return (Real(0.5) * radiusSquared + deltaSquared) * mass;
    }

    Real calculateBoxInertia(Real mass, Real width, Real height, Vec2 centerOfMass)
    {
        constexpr Real div = 1.0 / 12.0;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return mass * (div * (width * width + height * height) + deltaSquared);
    }

    std::pair<Real, Vec2> calculatePolygonInertia(
        Real mass,
        VerticesContainer& verticesContainer,
        std::optional<Vec2> centerOfMass
    )
    {
        const size_t verticesCount = verticesContainer.size();
        if (verticesCount < 3)
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        Real signedArea = Real(0);
        Real cx = Real(0);
        Real cy = Real(0);
        Real xx = Real(0);
        Real yy = Real(0);

        const bool computeCOM = !centerOfMass.has_value();
        const Vec2* verticesPtr = verticesContainer.data();

        for (size_t i = 0; i < verticesCount; i++)
        {
            const Vec2& p0 = verticesPtr[i];
            const Vec2& p1 = verticesPtr[(i + 1) % verticesCount];

            Real cross = p0.x * p1.y - p1.x * p0.y;
            signedArea += cross;

            if (computeCOM)
            {
                cx += (p0.x + p1.x) * cross;
                cy += (p0.y + p1.y) * cross;
            }

            // Area moments about origin.
            xx += (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) * cross;
            yy += (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) * cross;
        }

        // If winding order is clockwise, reverse the container to make it counter-clockwise.
        if (signedArea < Real(0))
        {
            std::reverse(verticesContainer.begin(), verticesContainer.end());
            signedArea = -signedArea;
            xx = -xx;
            yy = -yy;
            if (computeCOM)
            {
                cx = -cx;
                cy = -cy;
            }
        }

        signedArea *= Real(0.5);
        const Real absoluteArea = signedArea;
        if (absoluteArea < std::numeric_limits<Real>::epsilon())
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        // Determine final Center of Mass.
        Vec2 finalCOM;
        if (computeCOM)
        {
            finalCOM = Vec2(
                cx / (Real(6) * signedArea),
                cy / (Real(6) * signedArea)
            );
        }
        else
        {
            finalCOM = centerOfMass.value();
        }

        xx /= Real(12);
        yy /= Real(12);

        // Local/World polar moment of area scaled to mass moment
        Real inertia = (mass / absoluteArea) * (xx + yy);

        if (!computeCOM)
        {
            Real deltaSquared = glm::dot(finalCOM, finalCOM);
            inertia += mass * deltaSquared;
        }
        else
        {
            inertia -= mass * glm::dot(finalCOM, finalCOM);
        }

        return { inertia, finalCOM };
    }

    std::vector<Vec2> filterDuplicateVertices(std::span<const Vec2> vertices, Real epsilon)
    {
        if (vertices.empty())
        {
            return {};
        }

        const Real epsSq = epsilon * epsilon;
        auto isClose = [epsSq](const Vec2& a, const Vec2& b)
            {
                const auto dpos = a - b;
                return glm::dot(dpos, dpos) <= epsSq;
            };

        std::vector<Vec2> uniqueVertices;
        uniqueVertices.reserve(vertices.size());

        // Step 1: Forward pass collapsing contiguous duplicate neighbors in sequence.
        for (const Vec2& current : vertices)
        {
            if (uniqueVertices.empty() || !isClose(uniqueVertices.back(), current))
            {
                uniqueVertices.push_back(current);
            }
        }

        // Step 2: Cyclic wrap-around pass (checking boundary adjacency between last and first vertex).
        while (uniqueVertices.size() > 1 && isClose(uniqueVertices.back(), uniqueVertices.front()))
        {
            uniqueVertices.pop_back();
        }

        return uniqueVertices;
    }
}