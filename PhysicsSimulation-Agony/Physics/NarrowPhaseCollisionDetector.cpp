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


    static inline Vec2 projectBox(
        const Vec2& center,
        const Vec2& right,
        const Vec2& up,
        const Real halfWidth,
        const Real halfHeight,
        const Vec2& axis)
    {
        const Real c = glm::dot(center, axis);
        const Real r = halfWidth * std::abs(glm::dot(right, axis)) +
            halfHeight * std::abs(glm::dot(up, axis));
        return { c - r, c + r };
    }

    static inline Vec2 supportPointOnBox(
        const Vec2& position,
        const Vec2& right,
        const Vec2& up,
        const Real halfWidth,
        const Real halfHeight,
        const Vec2& direction)
    {
        Vec2 point = position;
        point += (glm::dot(direction, right) >= Real(0) ? Real(1) : Real(-1)) * halfWidth * right;
        point += (glm::dot(direction, up) >= Real(0) ? Real(1) : Real(-1)) * halfHeight * up;
        return point;
    }


    void NarrowPhaseCollisionDetector::setDataViewers(
        const BodySoAViewer& bodies,
        const CircleSoAViewer& circles,
        const BoxSoAViewer& boxes
    )
    {
        this->bodies = bodies;
        this->circles = circles;
        this->boxes = boxes;
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

    size_t NarrowPhaseCollisionDetector::getMemoryUsage() const
    {
        size_t total = 0;

        total += PS_AGONY::getVectorMemoryUsage(narrowCollisionData);

        return total;
    }

    void NarrowPhaseCollisionDetector::collisionCircleCircle(BodyIndex indexA, BodyIndex indexB)
    {
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        // Gather data.
        const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
        const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

        const BodyIndex shapeA = shapeIndexPtr[indexA];
        const BodyIndex shapeB = shapeIndexPtr[indexB];

        const Real radiusA = radiusPtr[shapeA];
        const Real radiusB = radiusPtr[shapeB];

        // Delta position.
        const Vec2 deltaPosition = positionB - positionA;

        // Radius sum.
        const Real radiusSum = radiusA + radiusB;

        // Distance.
        const Real squaredDistance = glm::dot(deltaPosition, deltaPosition);
        if (squaredDistance >= radiusSum * radiusSum)
        {
            return;
        }
        const Real distance = std::sqrt(squaredDistance);

        // Depth.
        const Real depth = radiusSum - distance;

        // Normal.
        Vec2 normal;
        if (distance == Real(0)) [[unlikely]]
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
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;

        const Real* CORE_RESTRICT widthPtr = boxes.width;
        const Real* CORE_RESTRICT heightPtr = boxes.height;

        // Gather data.
        const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
        const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

        const Real cosB = rotationCosPtr[indexB];
        const Real sinB = rotationSinPtr[indexB];

        const BodyIndex shapeA = shapeIndexPtr[indexA];
        const BodyIndex shapeB = shapeIndexPtr[indexB];

        const Real radiusA = radiusPtr[shapeA];

        const Real halfWidthB = widthPtr[shapeB] * Real(0.5);
        const Real halfHeightB = heightPtr[shapeB] * Real(0.5);


        // Get axes.
        const Vec2 right = {  cosB, sinB };
        const Vec2 up    = { -sinB, cosB };

        // Circle center in box local space.
        const Vec2 d = positionA - positionB;
        const Vec2 circleLocalPosition = {
            glm::dot(d, right),
            glm::dot(d, up)
        };

        // Closest point on box to the circle, in local space.
        const Vec2 closestLocal = {
            glm::clamp(circleLocalPosition.x, -halfWidthB,  halfWidthB),
            glm::clamp(circleLocalPosition.y, -halfHeightB, halfHeightB)
        };

        // Distance.
        const Vec2 deltaLocal = circleLocalPosition - closestLocal;
        const Real squaredDistance = glm::dot(deltaLocal, deltaLocal);
        
        if (squaredDistance >= radiusA * radiusA)
        {
            return;
        }

        //
        auto toWorldRotation = [&](const Vec2& pLocal) -> Vec2 {
            return {
                cosB * pLocal.x - sinB * pLocal.y,
                sinB * pLocal.x + cosB * pLocal.y
            };
            };

        if (squaredDistance > Real(1e-8)) [[likely]]
        {
            const Real distance = std::sqrt(squaredDistance);

            const Vec2 normalLocal = deltaLocal / -distance;
            const Vec2 normal = toWorldRotation(normalLocal);
            
            const Real depth = radiusA - distance;

            const Vec2 contactOnCircle = positionA + normal * radiusA;

            narrowCollisionData.emplace_back(
                indexA, indexB,
                normal,
                depth,
                contactOnCircle,            // Contact 1.
                Vec2(),                     // Contact 2.
                1                           // Single contact.
            );
            return;
        }

        // Circle center is inside the box (or extremely close to an edge/corner)
        // Choose the nearest face in local space.
        const Real dx = halfWidthB - std::abs(circleLocalPosition.x);
        const Real dy = halfHeightB - std::abs(circleLocalPosition.y);
        const bool useX = dx < dy;
        const Real minPen = useX ? dx : dy;

        const Real sx = std::copysign(Real(1), circleLocalPosition.x);
        const Real sy = std::copysign(Real(1), circleLocalPosition.y);

        Vec2 normalLocal;
        Vec2 pointLocal;
        if (useX)
        {
            normalLocal = Vec2(sx, Real(0));
            pointLocal  = Vec2(sx * halfWidthB, circleLocalPosition.y);
        }
        else
        {
            normalLocal = Vec2(Real(0), sy);
            pointLocal  = Vec2(circleLocalPosition.x, sy * halfHeightB);
        }

        const Vec2 normal = toWorldRotation(normalLocal);
        const Real depth = radiusA + minPen;

        const Vec2 contactOnCircle = positionA + normal * radiusA;

        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            contactOnCircle,     // Contact 1.
            Vec2(),              // Contact 2.
            1                    // Single contact.
        );
    }

    void NarrowPhaseCollisionDetector::collisionCirclePolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionBoxBox(BodyIndex indexA, BodyIndex indexB)
    {
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos;
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin;
        const BodyIndex* CORE_RESTRICT shapeIndexPtr = bodies.shapeIndex;

        const Real* CORE_RESTRICT widthPtr = boxes.width;
        const Real* CORE_RESTRICT heightPtr = boxes.height;

        // Gather data.
        const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
        const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

        const Real cosA = rotationCosPtr[indexA];
        const Real sinA = rotationSinPtr[indexA];
        const Real cosB = rotationCosPtr[indexB];
        const Real sinB = rotationSinPtr[indexB];

        const BodyIndex shapeA = shapeIndexPtr[indexA];
        const BodyIndex shapeB = shapeIndexPtr[indexB];

        const Real halfWidthA = widthPtr[shapeA] * Real(0.5);
        const Real halfHeightA = heightPtr[shapeA] * Real(0.5);
        const Real halfWidthB = widthPtr[shapeB] * Real(0.5);
        const Real halfHeightB = heightPtr[shapeB] * Real(0.5);

        // Get axes.
        const Vec2 rightA = { cosA, sinA };
        const Vec2 upA = { -sinA, cosA };

        const Vec2 rightB = { cosB, sinB };
        const Vec2 upB = { -sinB, cosB };

        // Delta position.
        const Vec2 deltaPosition = positionB - positionA;

        // SAT.
        const Vec2 axes[4] = { rightA, upA, rightB, upB };

        Real depth = FLT_MAX;
        Vec2 normal = {};

        for (const Vec2& axis : axes)
        {
            const Vec2 rangeA = projectBox(positionA, rightA, upA, halfWidthA, halfHeightA, axis);
            const Vec2 rangeB = projectBox(positionB, rightB, upB, halfWidthB, halfHeightB, axis);

            if (rangeA.x >= rangeB.y || rangeB.x >= rangeA.y)
            {
                return;
            }

            const Real depthA = rangeB.y - rangeA.x;
            const Real depthB = rangeA.y - rangeB.x;
            const Real axisDepth = std::min(depthA, depthB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }
        }

        // Orient normal from A to B.
        if (glm::dot(deltaPosition, normal) < Real(0))
        {
            normal = -normal;
        }

        // Contact.
        const Vec2 contactOnA = supportPointOnBox(
            positionA,
            rightA,
            upA,
            halfWidthA,
            halfHeightA,
            normal
        );

        // Result.
        narrowCollisionData.emplace_back(
            indexA, indexB,
            normal,
            depth,
            contactOnA,    // Contact 1.
            Vec2(),        // Contact 2.
            1              // Single contact.
        );
    }

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(BodyIndex indexA, BodyIndex indexB)
    {
    }
}
