#include "../NarrowPhaseCollisionDetector.h"
#include "../Constants.h"
#include "../HelperMethods.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

#include <bit>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;

    void NarrowPhaseCollisionDetector::collisionBoxPolygon(
        std::span<const ObjectPair> pairs,
        CacheLineAlignedVector<BodyCollisionData>& outCollisionData)
    {
        TRACY_SCOPE_N("Box-polygon collision");

        // NOTE: This routine works in Polygon B's local space rather than Box A's.
        // The box is described by only a center point and two axes, so transforming
        // it into B's frame is O(1); transforming B's (potentially large) vertex
        // list into A's frame, as the box-box/polygon-polygon routines do, would be
        // O(N). Polygon vertices are therefore read directly from localVertsB with
        // no per-pair transform or scratch buffer.

        enum class SATAxis : uint32_t
        {
            BOX_RIGHT,
            BOX_UP,
            POLY_EDGE
        };

        const Real* ECSTASY_RESTRICT positionXPtr = colliders.worldPosX;
        const Real* ECSTASY_RESTRICT positionYPtr = colliders.worldPosY;
        const Real* ECSTASY_RESTRICT rotationCosPtr = colliders.worldRotationCos;
        const Real* ECSTASY_RESTRICT rotationSinPtr = colliders.worldRotationSin;
        const ObjectIndex* ECSTASY_RESTRICT shapeIndexPtr = colliders.shapeIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderOwnerPtr = colliders.bodyIndex;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        const PolygonGeometry* ECSTASY_RESTRICT polyGeometryPtr = polygons.geometry;

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

            const PolygonGeometry& localPolygonVertices = polyGeometryPtr[shapeB];
            const Vec2* localVertsB = localPolygonVertices.vertices();
            const Vec2* localNormalsB = localPolygonVertices.normals();
            const size_t vertexCount = localPolygonVertices.size();

            const Vec2 rightB = { cosB, sinB };
            const Vec2 upB = { -sinB, cosB };

            // Rotation taking a vector from Box A's local space into Polygon B's
            // local space (the inverse of the A<-B rotation used elsewhere in this
            // file), plus Box A's center expressed in B's local space.
            const Real relCos = cosA * cosB + sinA * sinB;
            const Real relSin = cosA * sinB - sinA * cosB;

            const Vec2 rightA_inB = { relCos, -relSin };
            const Vec2 upA_inB = { relSin, relCos };

            const Vec2 centerDeltaBA = positionA - positionB;
            const Vec2 boxCenterInB = {
                glm::dot(centerDeltaBA, rightB),
                glm::dot(centerDeltaBA, upB)
            };

            Vec2 normalLocal;
            Real depth = std::numeric_limits<Real>::max();
            SATAxis bestAxisType = SATAxis::BOX_RIGHT;
            uint32_t bestAxisIndex = 0;
            Real bestAxisFacing = std::numeric_limits<Real>::max();

            Real polyMinX, polyMaxX, polyMinY, polyMaxY;

            // Test Box X axis (rightA_inB), values kept relative to the box center
            // to match the box-local-space convention the rest of the routine uses.
            {
                Real polyMinXRaw, polyMaxXRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, rightA_inB, polyMinXRaw, polyMaxXRaw);

                const Real boxCenterProjX = glm::dot(boxCenterInB, rightA_inB);
                polyMinX = polyMinXRaw - boxCenterProjX;
                polyMaxX = polyMaxXRaw - boxCenterProjX;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfWidthA - polyMinX, polyMaxX + halfWidthA);
                if (overlap < Real(0)) continue;

                const Real polyCenterX = HALF * (polyMinX + polyMaxX);
                const Real axisFacing = -std::fabs(polyCenterX);
                depth = overlap;
                normalLocal = rightA_inB;
                flipSignIfNegative(normalLocal, polyCenterX);
                bestAxisType = SATAxis::BOX_RIGHT;
                bestAxisFacing = axisFacing;
            }

            // Test Box Y axis (upA_inB).
            {
                Real polyMinYRaw, polyMaxYRaw;
                projectVerticesOnAxis(localVertsB, vertexCount, upA_inB, polyMinYRaw, polyMaxYRaw);

                const Real boxCenterProjY = glm::dot(boxCenterInB, upA_inB);
                polyMinY = polyMinYRaw - boxCenterProjY;
                polyMaxY = polyMaxYRaw - boxCenterProjY;

                // SAT penetration depth calculation.
                const Real overlap = std::fmin(halfHeightA - polyMinY, polyMaxY + halfHeightA);
                if (overlap < Real(0)) continue;

                const Real polyCenterY = HALF * (polyMinY + polyMaxY);
                const Real axisFacing = -std::fabs(polyCenterY);
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;
                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    normalLocal = upA_inB;
                    flipSignIfNegative(normalLocal, polyCenterY);
                    bestAxisType = SATAxis::BOX_UP;
                    bestAxisFacing = axisFacing;
                }
            }

            // Test Polygon edge outward normals. These are already expressed in
            // Polygon B's own local space, so no per-edge rotation is needed here
            // (unlike the box-local-space version of this routine).
            const Real polyCenterX = HALF * (polyMinX + polyMaxX);
            const Real polyCenterY = HALF * (polyMinY + polyMaxY);
            const Vec2 polyCenterRelVec = polyCenterX * rightA_inB + polyCenterY * upA_inB;
            for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
            {
                const Vec2 nB = localNormalsB[i];
                Vec2 axisLocal = nB;

                const Real boxRadius = halfWidthA * std::fabs(glm::dot(axisLocal, rightA_inB)) + halfHeightA * std::fabs(glm::dot(axisLocal, upA_inB));

                // One-sided face separation: polygon max extent along its own outward
                // normal is dot(n, v_i); box min extent is (center - radius).
                const Real boxCenterProj = glm::dot(boxCenterInB, axisLocal);
                const Real overlap = glm::dot(axisLocal, localVertsB[i]) - (boxCenterProj - boxRadius);
                if (overlap < Real(0)) goto nextPair;

                const Real axisFacing = glm::dot(axisLocal, polyCenterRelVec);
                const bool strictlyBetter = overlap < depth;
                const bool tiedButBetterFacing = (overlap <= depth) && axisFacing < bestAxisFacing;

                if (strictlyBetter || tiedButBetterFacing)
                {
                    depth = overlap;
                    // axisLocal is B's own outward edge normal (unambiguous from local winding);
                    // negate to match the "A to B" normal convention, no position heuristic needed.
                    normalLocal = -axisLocal;
                    bestAxisType = SATAxis::POLY_EDGE;
                    bestAxisIndex = i;
                    bestAxisFacing = axisFacing;
                }
            }

            {
                const bool refIsBox = bestAxisType != SATAxis::POLY_EDGE;

                Vec2 refNormalLocal;
                Vec2 refFaceCenterLocal;
                Vec2 sideDirLocal;
                Vec2 refEdgeStartLocal;
                Vec2 refEdgeEndLocal;

                if (refIsBox)
                {
                    refNormalLocal = normalLocal;

                    // normalLocal is exactly +-rightA_inB (BOX_RIGHT) or +-upA_inB (BOX_UP),
                    // so its box-local "x"/"y" components collapse to a known best-axis
                    // check plus a dot product against the corresponding box axis.
                    const bool useX = bestAxisType == SATAxis::BOX_RIGHT;
                    const Real sign = useX ? glm::dot(refNormalLocal, rightA_inB) : glm::dot(refNormalLocal, upA_inB);

                    if (useX)
                    {
                        refFaceCenterLocal = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        sideDirLocal = upA_inB;
                    }
                    else
                    {
                        refFaceCenterLocal = boxCenterInB + upA_inB * (sign * halfHeightA);
                        sideDirLocal = rightA_inB;
                    }

                    const Vec2 edgeOffset = sideDirLocal * (useX ? halfHeightA : halfWidthA);
                    refEdgeStartLocal = refFaceCenterLocal + edgeOffset;
                    refEdgeEndLocal = refFaceCenterLocal - edgeOffset;
                }
                else
                {
                    refNormalLocal = -normalLocal;

                    const uint32_t i0 = bestAxisIndex;
                    const uint32_t i1 = nextIndex(i0, uint32_t(vertexCount));

                    refEdgeStartLocal = localVertsB[i0];
                    refEdgeEndLocal = localVertsB[i1];
                    refFaceCenterLocal = (refEdgeStartLocal + refEdgeEndLocal) * HALF;

                    const Vec2 edge = refEdgeEndLocal - refEdgeStartLocal;
                    const Real edgeLenSquared = glm::dot(edge, edge);

                    sideDirLocal = edge * (Real(-1) / std::sqrt(edgeLenSquared));
                }

                Vec2 incEdgeStartLocal, incEdgeEndLocal;
                uint32_t incidentEdge, incidentEdge2;

                if (refIsBox)
                {
                    incidentEdge = 0;
                    Real minDot = std::numeric_limits<Real>::max();

                    for (uint32_t i = 0; i < uint32_t(vertexCount); i++)
                    {
                        // Already in Polygon B's local space, same frame as refNormalLocal:
                        // no per-edge rotation needed here (unlike the box-local-space version).
                        const Vec2 nB = localNormalsB[i];
                        const Real d = glm::dot(refNormalLocal, nB);
                        if (d < minDot)
                        {
                            minDot = d;
                            incidentEdge = i;
                        }
                    }

                    incidentEdge2 = nextIndex(incidentEdge, uint32_t(vertexCount));
                    incEdgeStartLocal = localVertsB[incidentEdge];
                    incEdgeEndLocal = localVertsB[incidentEdge2];
                }
                else
                {
                    // refNormalLocal is B's polygon-edge normal expressed in B's local
                    // space already; project it onto the box's axes-in-B to recover the
                    // same box-local x/y components the box-local-space version reads
                    // directly, then pick the box's incident edge exactly as before.
                    const Real dotX = glm::dot(refNormalLocal, rightA_inB);
                    const Real dotY = glm::dot(refNormalLocal, upA_inB);
                    const bool useX = std::fabs(dotX) > std::fabs(dotY);
                    const Real sign = -std::copysign(Real(1), useX ? dotX : dotY);

                    const uint32_t* const incidentEdgeIds = BOX_INCIDENT_EDGE_IDS[useX][sign > 0];
                    incidentEdge = incidentEdgeIds[0];
                    incidentEdge2 = incidentEdgeIds[1];

                    Vec2 faceCenter, edgeOffset;
                    if (useX)
                    {
                        faceCenter = boxCenterInB + rightA_inB * (sign * halfWidthA);
                        edgeOffset = upA_inB * halfHeightA;
                    }
                    else
                    {
                        faceCenter = boxCenterInB + upA_inB * (sign * halfHeightA);
                        edgeOffset = rightA_inB * halfWidthA;
                    }

                    incEdgeStartLocal = faceCenter + edgeOffset;
                    incEdgeEndLocal = faceCenter - edgeOffset;
                }

                Vec2 clipped[MAX_CONTACTS_PER_MANIFOLD] = { incEdgeStartLocal, incEdgeEndLocal };
                if (clipSegment(clipped[0], clipped[1], refEdgeStartLocal, -sideDirLocal)) continue;
                if (clipSegment(clipped[0], clipped[1], refEdgeEndLocal, sideDirLocal)) continue;

                const uint32_t bodyTag = refIsBox ? REFERENCE_BODY_TAG : 0u;
                uint32_t id1 = bodyTag | incidentEdge;
                uint32_t id2 = bodyTag | incidentEdge2;

                Vec2 contactPoints[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactIds[MAX_CONTACTS_PER_MANIFOLD];
                uint32_t contactCount = 0;
                const Real refPlaneDist = glm::dot(refFaceCenterLocal, refNormalLocal);

                const Vec2 normalWorld = rightB * normalLocal.x + upB * normalLocal.y;

                for (uint32_t i = 0; i < MAX_CONTACTS_PER_MANIFOLD; i++)
                {
                    if (glm::dot(clipped[i], refNormalLocal) <= refPlaneDist + SAT_EPSILON)
                    {
                        contactPoints[contactCount] = positionB + rightB * clipped[i].x + upB * clipped[i].y;
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