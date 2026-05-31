#include "NarrowPhaseCollisionDetector.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

namespace PS_AGONY
{
    const NarrowPhaseCollisionDetector::CollisionFunc
        NarrowPhaseCollisionDetector::collisionFunctions[static_cast<size_t>(BodyType::COUNT)][static_cast<size_t>(BodyType::COUNT)] =
    {
        /* Row for Circle (0) */
        {
            &NarrowPhaseCollisionDetector::collisionCircleCircle,
            &NarrowPhaseCollisionDetector::collisionCircleBox,
            &NarrowPhaseCollisionDetector::collisionCirclePolygon
        },
        /* Row for Box (1) */
        {
            nullptr, // Box-Circle
            &NarrowPhaseCollisionDetector::collisionBoxBox,
            &NarrowPhaseCollisionDetector::collisionBoxPolygon
        },
        /* Row for Polygon (2) */
        {
            nullptr, // Polygon-Circle
            nullptr, // Polygon-Box
            &NarrowPhaseCollisionDetector::collisionPolygonPolygon
        }
    };


    void NarrowPhaseCollisionDetector::setDataViewers(const BodySoAViewer& bodies, const CircleSoAViewer& circles)
    {
        this->bodies = bodies;
        this->circles = circles;
    }

    const std::vector<BodyCollisionData>& NarrowPhaseCollisionDetector::findCollisions(const std::vector<BodyPair>& bodyPairs)
	{
        TRACY_SCOPE_N("Narrow phase");

        // Check viewers.
        if (this->bodies.positionX == nullptr) return narrowCollisionData;
        if (this->circles.radius == nullptr) return narrowCollisionData;

        // Prepare.
        narrowCollisionData.clear();

        if (bodyPairs.empty()) return narrowCollisionData; // No pairs to check.

        narrowCollisionData.reserve(bodyPairs.size());

        // Real search.
        const BodyType* CORE_RESTRICT bodyTypePtr = bodies.bodyType;

        for (auto [bodyIndexA, bodyIndexB] : bodyPairs)
        {
            BodyType bodyTypeA = bodyTypePtr[bodyIndexA];
            BodyType bodyTypeB = bodyTypePtr[bodyIndexB];

            // Ensure bodyTypeA <= bodyTypeB.
            if (bodyTypeA > bodyTypeB)
            {
                std::swap(bodyIndexA, bodyIndexB);
                std::swap(bodyTypeA, bodyTypeB);
            }

            // Retrieve the collision function from the matrix.
            auto func = collisionFunctions[static_cast<size_t>(bodyTypeA)][static_cast<size_t>(bodyTypeB)];
            
            // Call the func.
            (this->*func)(bodyIndexA, bodyIndexB);
        }

		return narrowCollisionData;
	}

    void NarrowPhaseCollisionDetector::collisionCircleCircle(BodyIndex indexA, BodyIndex indexB)
    {
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        // Gather data.
        const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
        const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

        const Real radiusA = radiusPtr[indexA];
        const Real radiusB = radiusPtr[indexB];

        // Delta position.
        const Vec2 deltaPosition = positionB - positionA;

        // Radius sum.
        const Real radiusSum = radiusA + radiusB;

        // Distance.
        const Real squaredDistance = deltaPosition.x * deltaPosition.x + deltaPosition.y * deltaPosition.y; // Fuck glm::dot, I don't like it. :P
        if (squaredDistance >= radiusSum * radiusSum)
        {
            return;
        }
        const Real distance = std::sqrt(squaredDistance);

        // Depth.
        const Real depth = radiusSum - distance;

        // Normal.
        Vec2 normal;
        if (distance == Real(0))
        {
            normal.x = 1.0;
            normal.y = 0.0;
        }
        else
        {
            const Real invDistance = Real(1.0) / distance;
            normal = deltaPosition * invDistance;
        }

        // Result.
        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            positionA + normal * radiusA,   // Contact 1.
            Vec2(),                         // Contact 2.
            1                               // Single contact.
        );
    }

    void NarrowPhaseCollisionDetector::collisionCircleBox(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }
}
