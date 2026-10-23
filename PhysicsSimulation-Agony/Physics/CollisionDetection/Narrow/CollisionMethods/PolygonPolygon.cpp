#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Physics/Limits.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

namespace PS_AGONY2D
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    void NarrowPhaseCollisionDetector::collisionPolygonPolygon(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Polygon-polygon collision");

        enum class SATAxis : uint32_t
        {
            A_EDGE,
            B_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

        alignas(CACHE_LINE_ALIGNMENT) Vec2 vertsB_inA[MAX_POLYGON_VERTEX_COUNT];
        alignas(CACHE_LINE_ALIGNMENT) Vec2 normalsB_inA[MAX_POLYGON_VERTEX_COUNT];  // B edge normals rotated into A-local.

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

            const PolygonGeometry& localVertsAContainer = polyGeometryPtr[shapeA];
            const PolygonGeometry& localVertsBContainer = polyGeometryPtr[shapeB];

            const size_t countA = localVertsAContainer.size();
            const size_t countB = localVertsBContainer.size();

            const Vec2 rightA = { cosA, sinA };
            const Vec2 upA = { -sinA, cosA };

            const Vec2 centerDelta = positionB - positionA;
            const Vec2 bodyDeltaLocal = {
                glm::dot(centerDelta, rightA),
                glm::dot(centerDelta, upA)
            };

            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2* localVertsA = localVertsAContainer.vertices();
            const Vec2* normalsA = localVertsAContainer.normals();
            const Vec2* localVertsB = localVertsBContainer.vertices();
            const Vec2* normalsB = localVertsBContainer.normals();

            for (size_t i = 0; i < countB; i++)
            {
                const Vec2 v = localVertsB[i];
                vertsB_inA[i] = {
                    bodyDeltaLocal.x + relCos * v.x - relSin * v.y,
                    bodyDeltaLocal.y + relSin * v.x + relCos * v.y
                };
            }

            // Facing direction for SAT axis tie-breaking: A origin to B origin, in A-local space.
            const Vec2 centerDeltaLocal = bodyDeltaLocal;

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::A_EDGE;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            // Face-separation SAT. For a face with outward unit normal n, the owning polygon's
            // max extent along n is just dot(n, faceVertex), so only the OTHER polygon is projected
            // (O(nA*nB) per direction instead of re-projecting both shapes on every axis).
            // overlap = faceDist - min_other(dot(n, v)); negative => separating axis.
            auto consider = [&](Real overlap, Vec2 axisLocal, SATAxis axisType, uint32_t axisIndex)
                {
                    if (overlap > depth) return;

                    const Real facingSign = Real(axisType == SATAxis::B_EDGE) * Real(2) - Real(1);
                    const Real axisFacing = facingSign * glm::dot(axisLocal, centerDeltaLocal);

                    if (overlap < depth || axisFacing < bestAxisFacing)
                    {
                        depth = overlap;
                        // axisLocal is that shape's own outward edge normal: A's points from A to B
                        // already; B's must be negated to match.
                        normalLocal = (axisType == SATAxis::A_EDGE) ? axisLocal : -axisLocal;

                        bestAxisType = axisType;
                        bestAxisIndex = axisIndex;
                        bestAxisFacing = axisFacing;
                    }
                };

            for (uint32_t i = 0; i < uint32_t(countA); i++)
            {
                const Vec2 axisLocal = normalsA[i];

                Real minB = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countB; j++)
                {
                    const Real p = glm::dot(vertsB_inA[j], axisLocal);
                    minB = std::fmin(minB, p);
                }

                const Real overlap = glm::dot(localVertsA[i], axisLocal) - minB;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::A_EDGE, i);
            }

            for (uint32_t i = 0; i < uint32_t(countB); i++)
            {
                const Vec2 nB = normalsB[i];
                const Vec2 axisLocal = { relCos * nB.x - relSin * nB.y, relSin * nB.x + relCos * nB.y };
                normalsB_inA[i] = axisLocal;

                Real minA = std::numeric_limits<Real>::max();
                for (size_t j = 0; j < countA; j++)
                {
                    const Real p = glm::dot(localVertsA[j], axisLocal);
                    minA = std::fmin(minA, p);
                }

                const Real overlap = glm::dot(vertsB_inA[i], axisLocal) - minA;
                if (overlap < Real(0)) goto nextPair;
                consider(overlap, axisLocal, SATAxis::B_EDGE, i);
            }

            {
                const bool refIsA = bestAxisType == SATAxis::A_EDGE;

                const Vec2* refVertsPtr = refIsA ? localVertsA : vertsB_inA;
                const size_t refCount = refIsA ? countA : countB;

                const Vec2* incVertsPtr = refIsA ? vertsB_inA : localVertsA;
                const size_t incCount = refIsA ? countB : countA;

                Vec2 refNormalLocal = refIsA ? normalLocal : -normalLocal;
                Vec2 refEdgeStart, refEdgeEnd, refFaceCenter, sideDir;

                {
                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = nextIndex(i0, uint32_t(refCount));

                    refEdgeStart = refVertsPtr[i0];
                    refEdgeEnd = refVertsPtr[i1];
                    refFaceCenter = (refEdgeStart + refEdgeEnd) * HALF;

                    const Vec2 edge = refEdgeEnd - refEdgeStart;
                    const Real edgeLenSquared = glm::dot(edge, edge);

                    sideDir = edge * (Real(1) / std::sqrt(edgeLenSquared));
                }

                uint32_t incidentEdgeIndex = 0;
                Real minDot = std::numeric_limits<Real>::max();

                for (uint32_t i = 0; i < uint32_t(incCount); i++)
                {
                    const Vec2 nLocal = refIsA ? normalsB_inA[i] : normalsA[i];

                    const Real d = glm::dot(refNormalLocal, nLocal);
                    if (d < minDot)
                    {
                        minDot = d;
                        incidentEdgeIndex = i;
                    }
                }

                Vec2 incEdgeStart = incVertsPtr[incidentEdgeIndex];
                Vec2 incEdgeEnd = incVertsPtr[nextIndex(incidentEdgeIndex, uint32_t(incCount))];

                Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStart, incEdgeEnd };
                if (clipSegment(clipped[0], clipped[1], refEdgeStart, sideDir)) goto nextPair;
                if (clipSegment(clipped[0], clipped[1], refEdgeEnd, -sideDir)) goto nextPair;

                const Real refPlaneDist = glm::dot(refFaceCenter, refNormalLocal);

                const uint32_t bodyTag = refIsA ? REFERENCE_BODY_TAG : 0u;
                const uint32_t id1 = bodyTag | incidentEdgeIndex;
                const uint32_t id2 = bodyTag | nextIndex(incidentEdgeIndex, uint32_t(incCount));

                Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactCount = 0;

                const Vec2 normalWorld = rightA * normalLocal.x + upA * normalLocal.y;

                for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionA + rightA * clipped[i].x + upA * clipped[i].y;
                        contactIds[contactCount] = (i == 0) ? id1 : id2;
                        contactCount++;
                    }
                }

                if (contactCount == 0) continue;

                outCollisionData.emplace_back(
                    indexA, indexB,
                    colliderOwnerPtr[indexA], colliderOwnerPtr[indexB],
                    normalWorld,
                    depth,
                    contactCount,
                    contactPoints[0],
                    contactPoints[1],
                    contactIds[0],
                    contactIds[1]
                );
            }
        nextPair:
            continue;
        }
    }
}