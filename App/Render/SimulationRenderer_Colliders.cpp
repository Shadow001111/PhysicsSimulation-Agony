#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "PS_AGONY2D/Simulation.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <cmath>

namespace PS_AGONY_RENDER
{
    template<class Fn>
    void SimulationRenderer::forEachColliderByGroup(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, Fn&& fn) const
    {
        for (size_t g = 0; g < FoundColliders::groupCount; g++)
        {
            const uint32_t groupColor = groupColors[g];
            for (size_t i = range.groupBegin[g]; i < range.groupBegin[g + 1]; i++)
            {
                fn(i, range.colliders[i], groupColor);
            }
        }
    }

    void SimulationRenderer::renderColliders(const BodyTransformContext& transformContext, const GroupColors& groupColors)
    {
        TRACY_SCOPE_N("Render colliders");

        renderCircleColliders(foundColliders.getRange(ColliderShapeType::Circle), groupColors, transformContext);
        renderBoxColliders(foundColliders.getRange(ColliderShapeType::Box), groupColors, transformContext);
        renderCapsuleColliders(foundColliders.getRange(ColliderShapeType::Capsule), groupColors, transformContext);
        renderPolygonColliders(foundColliders.getRange(ColliderShapeType::Polygon), groupColors, transformContext);
    }

    void SimulationRenderer::renderCircleColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const BodyTransformContext& transformContext)
    {
        const size_t count = range.colliders.size();
        if (count == 0) return;

        circleInstanceData.resize(count);

        const ColliderPlacer placer = ColliderPlacer::make(bodies, colliders, transformContext);
        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT out = circleInstanceData.data();

        forEachColliderByGroup(range, groupColors, [&](size_t i, ObjectDenseIndex collider, uint32_t color)
            {
                const ColliderPlacement p = placer.place(collider);
                out[i] = {
                    p.x, p.y, p.comX, p.comY, p.rotation,
                    static_cast<float>(radiusPtr[p.shapeIndex]),
                    color, static_cast<uint32_t>(collider + 1)
                };
            });

        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBoxColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const BodyTransformContext& transformContext)
    {
        const size_t count = range.colliders.size();
        if (count == 0) return;

        boxInstanceData.resize(count);

        const ColliderPlacer placer = ColliderPlacer::make(bodies, colliders, transformContext);
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;
        ShapeRenderer::BoxInstanceData* ECSTASY_RESTRICT out = boxInstanceData.data();

        forEachColliderByGroup(range, groupColors, [&](size_t i, ObjectDenseIndex collider, uint32_t color)
            {
                const ColliderPlacement p = placer.place(collider);
                out[i] = {
                    p.x, p.y, p.comX, p.comY, p.rotation,
                    static_cast<float>(halfWidthPtr[p.shapeIndex]),
                    static_cast<float>(halfHeightPtr[p.shapeIndex]),
                    color, static_cast<uint32_t>(collider + 1)
                };
            });

        shapeRenderer.renderBoxShapes(boxInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderCapsuleColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const BodyTransformContext& transformContext)
    {
        const size_t count = range.colliders.size();
        if (count == 0) return;

        boxInstanceData.resize(count);

        const ColliderPlacer placer = ColliderPlacer::make(bodies, colliders, transformContext);
        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT radiusPtr = capsules.radius;
        ShapeRenderer::BoxInstanceData* ECSTASY_RESTRICT out = boxInstanceData.data();

        forEachColliderByGroup(range, groupColors, [&](size_t i, ObjectDenseIndex collider, uint32_t color)
            {
                const ColliderPlacement p = placer.place(collider);
                const float radius = static_cast<float>(radiusPtr[p.shapeIndex]);
                out[i] = {
                    p.x, p.y, p.comX, p.comY, p.rotation,
                    static_cast<float>(halfLengthPtr[p.shapeIndex]) + radius,
                    radius,
                    color, static_cast<uint32_t>(collider + 1)
                };
            });

        shapeRenderer.renderCapsuleShapes(boxInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderPolygonColliders(const FoundColliders::ShapeRange& range, const GroupColors& groupColors, const BodyTransformContext& transformContext)
    {
        const size_t count = range.colliders.size();
        if (count == 0) return;

        const ColliderPlacer placer = ColliderPlacer::make(bodies, colliders, transformContext);
        const PolygonGeometry* ECSTASY_RESTRICT geometryPtr = polygons.geometry;

        size_t totalVertices = 0;
        for (const ObjectDenseIndex collider : range.colliders)
        {
            totalVertices += geometryPtr[placer.shapeIndex[collider]].size();
        }

        polygonVertexData.resize(totalVertices);
        polygonInstanceData.resize(count);
        polygonDrawCommands.resize(count);

        glm::vec2* ECSTASY_RESTRICT verts = polygonVertexData.data();
        ShapeRenderer::PolygonInstanceData* ECSTASY_RESTRICT instances = polygonInstanceData.data();
        DrawArraysIndirectCommand* ECSTASY_RESTRICT commands = polygonDrawCommands.data();

        uint32_t vertexOffset = 0;

        forEachColliderByGroup(range, groupColors, [&](size_t i, ObjectDenseIndex collider, uint32_t color)
            {
                const ColliderPlacement p = placer.place(collider);

                const PolygonGeometry& geometry = geometryPtr[p.shapeIndex];
                const Vec2* ECSTASY_RESTRICT localVerts = geometry.vertices();
                const uint32_t vertCount = static_cast<uint32_t>(geometry.size());

                for (uint32_t v = 0; v < vertCount; v++)
                {
                    verts[vertexOffset + v].x = localVerts[v].x;
                    verts[vertexOffset + v].y = localVerts[v].y;
                }

                instances[i] = { p.x, p.y, p.comX, p.comY, p.rotation, color, static_cast<uint32_t>(collider + 1) };
                commands[i] = { vertCount, 1, vertexOffset, 0 };

                vertexOffset += vertCount;
            });

        shapeRenderer.renderPolygonShapes(polygonVertexData, polygonInstanceData, polygonDrawCommands, viewProjectionMatrix);
    }

    void SimulationRenderer::renderColliderAABBs(std::span<const ObjectDenseIndex> givenColliders)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        aabbInstanceData.resize(count);

        const Real* ECSTASY_RESTRICT minXPtr = colliders.aabbMinX;
        const Real* ECSTASY_RESTRICT minYPtr = colliders.aabbMinY;
        const Real* ECSTASY_RESTRICT maxXPtr = colliders.aabbMaxX;
        const Real* ECSTASY_RESTRICT maxYPtr = colliders.aabbMaxY;

        ShapeRenderer::FloatAABB* ECSTASY_RESTRICT out = aabbInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ObjectDenseIndex collider = givenColliders[i];
            out[i] = {
                static_cast<float>(minXPtr[collider]), static_cast<float>(minYPtr[collider]),
                static_cast<float>(maxXPtr[collider]), static_cast<float>(maxYPtr[collider])
            };
        }

        shapeRenderer.renderAABBShapes(aabbInstanceData, { 1.0f, 0.0f, 0.0f }, viewProjectionMatrix);
    }

    void SimulationRenderer::renderLeafAABBs(std::span<const AABB> aabbs, const glm::vec3& color)
    {
        if (aabbs.empty()) return;

        aabbInstanceData.resize(aabbs.size());

        for (size_t i = 0; i < aabbs.size(); i++)
        {
            aabbInstanceData[i] = {
                static_cast<float>(aabbs[i].minX), static_cast<float>(aabbs[i].minY),
                static_cast<float>(aabbs[i].maxX), static_cast<float>(aabbs[i].maxY)
            };
        }

        shapeRenderer.renderAABBShapes(aabbInstanceData, color, viewProjectionMatrix);
    }

    void SimulationRenderer::renderContactPoints(const Simulation& simulation, const AABB& cameraAABB)
    {
        const auto& collisionData = simulation.getBodyCollisionData();
        if (collisionData.size() == 0) return;

        const CullRect cull(cameraAABB);

        simplePointInstanceData.clear();

        for (const auto& collData : collisionData)
        {
            for (size_t j = 0; j < collData.contactCount; j++)
            {
                const auto& contact = collData.contactPoints[j];

                const float x = static_cast<float>(contact.x);
                const float y = static_cast<float>(contact.y);
                if (!cull.contains(x, y)) continue;

                simplePointInstanceData.push_back({ x, y, 0.05f, idToHexColor(collData.contactIds[j]), 0 });
            }
        }

        shapeRenderer.renderSimplePointCircles(simplePointInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderColliderExtents(
        std::span<const ObjectDenseIndex> givenColliders,
        const glm::vec3& color,
        const Real* extentPtr,
        const BodyTransformContext& transformContext
    )
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        hollowCircleData.resize(count);

        const ColliderPlacer placer = ColliderPlacer::make(bodies, colliders, transformContext);
        const uint32_t packedColor = packColor(color);
        ShapeRenderer::HollowCircleInstanceData* ECSTASY_RESTRICT out = hollowCircleData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ObjectDenseIndex collider = givenColliders[i];
            const ColliderPlacement p = placer.place(collider);
            const float extent = static_cast<float>(extentPtr[collider]);

            out[i] = { p.x, p.y, extent, std::fmax(0.0f, extent - 0.05f), packedColor, 0 };
        }

        shapeRenderer.renderHollowCircleShapes(hollowCircleData, viewProjectionMatrix);
    }
}