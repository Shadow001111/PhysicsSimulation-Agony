#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

namespace PS_AGONY2D
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    void NarrowPhaseCollisionDetector::collisionBoxBox(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-box collision");

        enum class SATAxis : uint32_t
        {
            A_RIGHT,
            A_UP,
            B_RIGHT,
            B_UP
        };

        // Fits exactly 4 Reals.
        using Real4Simd = Ecstasy::Core::Simd<Real, sizeof(Real) * 32>;

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        for (auto [indexA, indexB] : pairs)
        {
            const Vec2 positionA = { positionXPtr[indexA], positionYPtr[indexA] };
            const Vec2 positionB = { positionXPtr[indexB], positionYPtr[indexB] };

            const Real cosA = rotationCosPtr[indexA];
            const Real sinA = rotationSinPtr[indexA];
            const Real cosB = rotationCosPtr[indexB];
            const Real sinB = rotationSinPtr[indexB];

            const ObjectIndex shapeA = shapeIndexPtr[indexA];
            const ObjectIndex shapeB = shapeIndexPtr[indexB];

            const Real halfWidthA = halfWidthPtr[shapeA];
            const Real halfHeightA = halfHeightPtr[shapeA];
            const Real halfWidthB = halfWidthPtr[shapeB];
            const Real halfHeightB = halfHeightPtr[shapeB];

            const Vec2 rightA = { cosA,  sinA };
            const Vec2 upA = { -sinA, cosA };
            const Vec2 rightB = { cosB,  sinB };
            const Vec2 upB = { -sinB, cosB };

            // Signed relative rotation: relCos = rightA.rightB, relSin = rightA x rightB.
            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;
            const Real absRelativeCos = std::fabs(relCos);
            const Real absRelativeSin = std::fabs(relSin);

            const Vec2 centerDelta = positionB - positionA;

            // Lanes: [A_RIGHT, A_UP, B_RIGHT, B_UP]. Built in registers (no store->load forwarding stalls).
            const Real4Simd halfWidthV = makeReal4(halfWidthB, halfWidthB, halfWidthA, halfWidthA);
            const Real4Simd halfHeightV = makeReal4(halfHeightB, halfHeightB, halfHeightA, halfHeightA);
            const Real4Simd halfWidthHeightV = makeReal4(halfWidthA, halfHeightA, halfWidthB, halfHeightB);
            const Real4Simd cosSinV1 = makeReal4(absRelativeCos, absRelativeSin, absRelativeCos, absRelativeSin);
            const Real4Simd cosSinV2 = makeReal4(absRelativeSin, absRelativeCos, absRelativeSin, absRelativeCos);

            const Real4Simd radiusSumV = Real4Simd::mulAdd(halfWidthV, cosSinV1, Real4Simd::mulAdd(halfHeightV, cosSinV2, halfWidthHeightV));

            // Axis components per lane; dot(centerDelta, axis) = dx * axisX + dy * axisY.
            const Real4Simd axisXV = makeReal4(cosA, -sinA, cosB, -sinB);
            const Real4Simd axisYV = makeReal4(sinA, cosA, sinB, cosB);
            const Real4Simd centerDeltaOnAxisV = Real4Simd::mulAdd(Real4Simd(centerDelta.x), axisXV, Real4Simd(centerDelta.y) * axisYV);

            const Real4Simd overlap = radiusSumV - (Real4Simd::getAbsMask() & centerDeltaOnAxisV);
            if ((overlap < Real4Simd(0)).movemask() != 0) continue; // Abort!

            // Scalar branchless argmin (lowest index wins ties).
            alignas(Real4Simd::bytes) Real overlapArray[4];
            overlap.store(overlapArray);

            const Real depth = std::fmin(std::fmin(overlapArray[0], overlapArray[1]), std::fmin(overlapArray[2], overlapArray[3]));
            const uint32_t notAxis0 = overlapArray[0] != depth;
            const uint32_t notAxis01 = notAxis0 & (uint32_t)(overlapArray[1] != depth);
            const uint32_t notAxis012 = notAxis01 & (uint32_t)(overlapArray[2] != depth);
            const uint32_t bestAxis = notAxis0 + notAxis01 + notAxis012;

            alignas(Real4Simd::bytes) Real axisXArray[4];
            alignas(Real4Simd::bytes) Real axisYArray[4];
            alignas(Real4Simd::bytes) Real centerDeltaOnAxisArray[4];
            axisXV.store(axisXArray);
            axisYV.store(axisYArray);
            centerDeltaOnAxisV.store(centerDeltaOnAxisArray);

            Vec2 normal = { axisXArray[bestAxis], axisYArray[bestAxis] };
            flipSignIfNegative(normal, centerDeltaOnAxisArray[bestAxis]);

            //
            const bool refIsA = SATAxis(bestAxis) < SATAxis::B_RIGHT;

            Vec2 positionRef, positionInc;
            Real halfWidthRef, halfHeightRef, halfWidthInc, halfHeightInc;
            Vec2 rightRef, upRef, rightInc, upInc;
            Vec2 refNormal;

            if (refIsA)
            {
                positionRef = positionA;
                halfWidthRef = halfWidthA;
                halfHeightRef = halfHeightA;
                rightRef = rightA;
                upRef = upA;

                positionInc = positionB;
                halfWidthInc = halfWidthB;
                halfHeightInc = halfHeightB;
                rightInc = rightB;
                upInc = upB;

                refNormal = normal;
            }
            else
            {
                positionRef = positionB;
                halfWidthRef = halfWidthB;
                halfHeightRef = halfHeightB;
                rightRef = rightB;
                upRef = upB;

                positionInc = positionA;
                halfWidthInc = halfWidthA;
                halfHeightInc = halfHeightA;
                rightInc = rightA;
                upInc = upA;

                refNormal = -normal;
            }

            // refNormal == sign * (reference face axis), so no dot is needed.
            const Real sign = std::copysign(Real(1), refIsA ? centerDeltaOnAxisArray[bestAxis] : -centerDeltaOnAxisArray[bestAxis]);

            Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;
            {
                // The reference face axis is the SAT axis with the minimum overlap.
                const bool useX = SATAxis(bestAxis) == SATAxis::A_RIGHT || SATAxis(bestAxis) == SATAxis::B_RIGHT;

                if (useX)
                {
                    refFaceCenter = positionRef + rightRef * (sign * halfWidthRef);
                    sideDir = upRef;
                }
                else
                {
                    refFaceCenter = positionRef + upRef * (sign * halfHeightRef);
                    sideDir = rightRef;
                }
                const Vec2 edgeOffset = sideDir * (useX ? halfHeightRef : halfWidthRef);
                refEdgeStart = refFaceCenter + edgeOffset;
                refEdgeEnd = refFaceCenter - edgeOffset;
            }

            Vec2 incEdgeStart, incEdgeEnd;
            {
                // dot(refAxis, rightInc | upInc) per best axis: 0:(c,-s) 1:(s,c) 2:(c,s) 3:(-s,c), times 'sign'.
                const bool axisIsUp = (bestAxis & 1u) != 0;
                const Real sideTerm = (bestAxis == 0 || bestAxis == 3) ? -relSin : relSin;
                const Real dotX = sign * (axisIsUp ? sideTerm : relCos);
                const Real dotY = sign * (axisIsUp ? relCos : sideTerm);
                const bool useX = std::fabs(dotX) > std::fabs(dotY);
                const Real incSign = -std::copysign(Real(1), useX ? dotX : dotY);

                Vec2 faceCenter, edgeOffset;
                if (useX)
                {
                    faceCenter = positionInc + rightInc * (incSign * halfWidthInc);
                    edgeOffset = upInc * halfHeightInc;
                }
                else
                {
                    faceCenter = positionInc + upInc * (incSign * halfHeightInc);
                    edgeOffset = rightInc * halfWidthInc;
                }
                incEdgeStart = faceCenter + edgeOffset;
                incEdgeEnd = faceCenter - edgeOffset;
            }

            Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStart, incEdgeEnd };
            if (clipSegment(clipped[0], clipped[1], refEdgeStart, -sideDir)) continue;
            if (clipSegment(clipped[0], clipped[1], refEdgeEnd, sideDir)) continue;

            Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
            uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
            uint32_t contactCount = 0;
            const Real refPlaneDist = glm::dot(refFaceCenter, refNormal);
            const uint32_t contactIdBase = (uint32_t)bestAxis << 1;
            for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
            {
                if (glm::dot(clipped[i], refNormal) <= refPlaneDist + SAT_EPSILON)
                {
                    contactPoints[contactCount] = clipped[i];
                    contactIds[contactCount] = contactIdBase | i;
                    contactCount++;
                }
            }
            if (contactCount == 0) continue;

            outCollisionData.emplace_back(
                indexA, indexB,
                colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                normal,
                depth,
                contactCount,
                contactPoints[0],
                contactPoints[1],
                contactIds[0],
                contactIds[1]
            );
        }
    }
}