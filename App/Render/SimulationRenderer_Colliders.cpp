#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "PS_AGONY2D/Simulation.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY_RENDER
{
    void SimulationRenderer::renderColliders(const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render colliders");

        renderCircleColliders(foundColliderShapes[(size_t)BodyType::Circle], viewProjectionMatrix, transformContext);
        renderBoxColliders(foundColliderShapes[(size_t)BodyType::Box], viewProjectionMatrix, transformContext);
        renderCapsuleColliders(foundColliderShapes[(size_t)BodyType::Capsule], viewProjectionMatrix, transformContext);
        renderPolygonColliders(foundColliderShapes[(size_t)BodyType::Polygon], viewProjectionMatrix, transformContext);
    }

    void SimulationRenderer::renderBodyCentersOfMass(const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Reserve space.
        simplePointInstanceData.clear();
        simplePointInstanceData.reserve(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT centerXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT centerYPtr = bodies.localCenterOfMassY;

        for (size_t i = 0; i < count; i++)
        {
            const glm::vec2 localCenterOfMass = { centerXPtr[i], centerYPtr[i] };

            const glm::vec2 worldCenterOfMass = {
                positionXPtr[i] + localCenterOfMass.x,
                positionYPtr[i] + localCenterOfMass.y
            };

            if (worldCenterOfMass.x < cullMinX || worldCenterOfMass.x > cullMaxX ||
                worldCenterOfMass.y < cullMinY || worldCenterOfMass.y > cullMaxY)
            {
                continue;
            }

            auto& renderData = simplePointInstanceData.emplace_back();

            renderData.positionX = worldCenterOfMass.x;
            renderData.positionY = worldCenterOfMass.y;
            renderData.radius = 0.1f;
            renderData.color = 0xFF0000;
            renderData.id = 0;
        }

        // Render.
        shapeRenderer.renderSimplePointCircles(simplePointInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyPositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Reserve space.
        simplePointInstanceData.clear();
        simplePointInstanceData.reserve(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;

        for (size_t i = 0; i < count; i++)
        {
            const float posX = static_cast<float>(positionXPtr[i]);
            const float posY = static_cast<float>(positionYPtr[i]);

            if (posX < cullMinX || posX > cullMaxX || posY < cullMinY || posY > cullMaxY)
            {
                continue;
            }

            auto& renderData = simplePointInstanceData.emplace_back();

            renderData.positionX = posX;
            renderData.positionY = posY;
            renderData.radius = 0.03f;
            renderData.color = 0x0000FF;
            renderData.id = 0;
        }

        // Render.
        shapeRenderer.renderSimplePointCircles(simplePointInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyTruePositions(const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Reserve space.
        simplePointInstanceData.clear();
        simplePointInstanceData.reserve(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT truePositionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT truePositionYPtr = bodies.worldCenterY;

        for (size_t i = 0; i < count; i++)
        {
            const float posX = static_cast<float>(truePositionXPtr[i]);
            const float posY = static_cast<float>(truePositionYPtr[i]);

            if (posX < cullMinX || posX > cullMaxX || posY < cullMinY || posY > cullMaxY)
            {
                continue;
            }

            auto& renderData = simplePointInstanceData.emplace_back();

            renderData.positionX = posX;
            renderData.positionY = posY;
            renderData.radius = 0.03f;
            renderData.color = 0x00FF00;
            renderData.id = 0;
        }

        // Render.
        shapeRenderer.renderSimplePointCircles(simplePointInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderCircleColliders(const std::vector<ColliderDenseIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render circle colliders");

        // Reserve space.
        circleInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        const ObjectDenseIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderDenseIndex colliderIndex = givenColliders[i];

            const ObjectDenseIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectDenseIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const BodyTransformSample body = transformContext.sample(bodyIndex);

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const Real worldOffX = localOffX * body.cosR - localOffY * body.sinR;
            const Real worldOffY = localOffX * body.sinR + localOffY * body.cosR;

            renderDataPtr[i].positionX = body.x + worldOffX;
            renderDataPtr[i].positionY = body.y + worldOffY;
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = body.rotation + localRot;
            renderDataPtr[i].radius = radiusPtr[shapeIndex];
            renderDataPtr[i].color = 0xFFFFFF;
            renderDataPtr[i].id = colliderIndex + 1;
        }

        // Render.
        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBoxColliders(const std::vector<ColliderDenseIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render box colliders");

        // Reserve space.
        boxInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        const ObjectDenseIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        ShapeRenderer::BoxInstanceData* ECSTASY_RESTRICT renderDataPtr = boxInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderDenseIndex colliderIndex = givenColliders[i];

            const ObjectDenseIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectDenseIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const BodyTransformSample body = transformContext.sample(bodyIndex);

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const Real worldOffX = localOffX * body.cosR - localOffY * body.sinR;
            const Real worldOffY = localOffX * body.sinR + localOffY * body.cosR;

            renderDataPtr[i].positionX = body.x + worldOffX;
            renderDataPtr[i].positionY = body.y + worldOffY;
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = body.rotation + localRot;
            renderDataPtr[i].halfWidth = halfWidthPtr[shapeIndex];
            renderDataPtr[i].halfHeight = halfHeightPtr[shapeIndex];
            renderDataPtr[i].color = 0xFFFFFF;
            renderDataPtr[i].id = colliderIndex + 1;
        }

        // Render.
        shapeRenderer.renderBoxShapes(boxInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderCapsuleColliders(const std::vector<ColliderDenseIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render capsule colliders");

        // Reserve space.
        boxInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        const ObjectDenseIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT halfLengthPtr = capsules.halfLength;
        const Real* ECSTASY_RESTRICT radiusPtr = capsules.radius;

        ShapeRenderer::BoxInstanceData* ECSTASY_RESTRICT renderDataPtr = boxInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderDenseIndex colliderIndex = givenColliders[i];

            const ObjectDenseIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectDenseIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const BodyTransformSample body = transformContext.sample(bodyIndex);

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const Real worldOffX = localOffX * body.cosR - localOffY * body.sinR;
            const Real worldOffY = localOffX * body.sinR + localOffY * body.cosR;

            renderDataPtr[i].positionX = body.x + worldOffX;
            renderDataPtr[i].positionY = body.y + worldOffY;
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = body.rotation + localRot;
            const float r = static_cast<float>(radiusPtr[shapeIndex]);
            renderDataPtr[i].halfWidth = static_cast<float>(halfLengthPtr[shapeIndex]) + r;
            renderDataPtr[i].halfHeight = r;
            renderDataPtr[i].color = 0xFFFFFF;
            renderDataPtr[i].id = colliderIndex + 1;
        }

        // Render.
        shapeRenderer.renderCapsuleShapes(boxInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderPolygonColliders(const std::vector<ColliderDenseIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render polygon colliders");

        // Body SoA pointers.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        // Collider SoA pointers.
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectDenseIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        // Polygon SoA pointers.
        const PolygonGeometry* ECSTASY_RESTRICT geometryPtr = polygons.geometry;

        // Count total vertices needed this frame.
        size_t totalVertices = 0;
        for (size_t i = 0; i < count; i++)
        {
            const ColliderDenseIndex colliderIndex = givenColliders[i];
            const ObjectDenseIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];
            totalVertices += geometryPtr[shapeIndex].size();
        }

        shapeRenderer.ensurePolygonBufferCapacity(totalVertices, count);

        // Build CPU-side data.
        polygonVertexData.resize(totalVertices);
        polygonInstanceData.resize(count);
        polygonDrawCommands.resize(count);

        glm::vec2* ECSTASY_RESTRICT verts = polygonVertexData.data();
        ShapeRenderer::PolygonInstanceData* ECSTASY_RESTRICT instData = polygonInstanceData.data();
        DrawArraysIndirectCommand* ECSTASY_RESTRICT cmds = polygonDrawCommands.data();

        uint32_t vertexOffset = 0;

        for (size_t i = 0; i < count; i++)
        {
            const ColliderDenseIndex colliderIndex = givenColliders[i];

            const ObjectDenseIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectDenseIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const PolygonGeometry& geometry = geometryPtr[shapeIndex];
            const Vec2* ECSTASY_RESTRICT localVertsPtr = geometry.vertices();
            const uint32_t vertCount = geometry.size();

            for (uint32_t v = 0; v < vertCount; v++)
            {
                verts[vertexOffset + v].x = localVertsPtr[v].x;
                verts[vertexOffset + v].y = localVertsPtr[v].y;
            }

            const BodyTransformSample body = transformContext.sample(bodyIndex);

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const Real worldOffX = localOffX * body.cosR - localOffY * body.sinR;
            const Real worldOffY = localOffX * body.sinR + localOffY * body.cosR;

            instData[i].positionX = body.x + worldOffX;
            instData[i].positionY = body.y + worldOffY;
            instData[i].localCOMX = localCOMXPtr[bodyIndex];
            instData[i].localCOMY = localCOMYPtr[bodyIndex];
            instData[i].rotation = body.rotation + localRot;
            instData[i].color = 0xFFFFFF;
            instData[i].id = colliderIndex + 1;

            cmds[i].count = vertCount;
            cmds[i].instanceCount = 1;
            cmds[i].first = vertexOffset;
            cmds[i].baseInstance = 0;

            vertexOffset += vertCount;
        }

        shapeRenderer.renderPolygonShapes(polygonVertexData, polygonInstanceData, polygonDrawCommands, viewProjectionMatrix);
    }

    void SimulationRenderer::renderColliderAABBs(const std::vector<ColliderDenseIndex>& givenColliders, const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render collider AABBs");

        const size_t colliderCount = givenColliders.size();
        if (colliderCount == 0) return;

        // Reserve space.
        aabbInstanceData.resize(colliderCount);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT minXPtr = colliders.aabbMinX;
        const Real* ECSTASY_RESTRICT minYPtr = colliders.aabbMinY;
        const Real* ECSTASY_RESTRICT maxXPtr = colliders.aabbMaxX;
        const Real* ECSTASY_RESTRICT maxYPtr = colliders.aabbMaxY;

        ShapeRenderer::FloatAABB* ECSTASY_RESTRICT renderDataPtr = aabbInstanceData.data();

        for (size_t i = 0; i < colliderCount; i++)
        {
            ColliderDenseIndex colliderIndex = givenColliders[i];
            renderDataPtr[i].minX = static_cast<float>(minXPtr[colliderIndex]);
            renderDataPtr[i].minY = static_cast<float>(minYPtr[colliderIndex]);
            renderDataPtr[i].maxX = static_cast<float>(maxXPtr[colliderIndex]);
            renderDataPtr[i].maxY = static_cast<float>(maxYPtr[colliderIndex]);
        }

        shapeRenderer.renderAABBShapes(aabbInstanceData, { 1.0f, 0.0f, 0.0f }, viewProjectionMatrix);
    }

    void SimulationRenderer::renderLeafAABBs(std::span<const AABB> aabbs, const glm::vec3& color, const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render leaf AABBs");

        if (aabbs.empty()) return;

        aabbInstanceData.resize(aabbs.size());

        for (size_t i = 0; i < aabbs.size(); i++)
        {
            aabbInstanceData[i].minX = static_cast<float>(aabbs[i].minX);
            aabbInstanceData[i].minY = static_cast<float>(aabbs[i].minY);
            aabbInstanceData[i].maxX = static_cast<float>(aabbs[i].maxX);
            aabbInstanceData[i].maxY = static_cast<float>(aabbs[i].maxY);
        }

        shapeRenderer.renderAABBShapes(aabbInstanceData, color, viewProjectionMatrix);
    }

    void SimulationRenderer::renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render contact points");

        const auto& collisionData = simulation.getBodyCollisionData();

        const size_t collisionCount = collisionData.size();
        if (collisionCount == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Reserve space.
        simplePointInstanceData.clear();

        // Prepare instance data.
        for (size_t i = 0; i < collisionCount; i++)
        {
            const auto& collData = collisionData[i];

            for (size_t j = 0; j < collData.contactCount; j++)
            {
                const auto& contact = collData.contactPoints[j];

                if (contact.x < cullMinX || contact.x > cullMaxX ||
                    contact.y < cullMinY || contact.y > cullMaxY)
                {
                    continue;
                }

                auto& renderData = simplePointInstanceData.emplace_back();

                renderData.positionX = contact.x;
                renderData.positionY = contact.y;
                renderData.radius = 0.05f;
                renderData.color = idToHexColor(collData.contactIds[j]);
                renderData.id = 0;
            }
        }

        // Render.
        shapeRenderer.renderSimplePointCircles(simplePointInstanceData, viewProjectionMatrix);
    }
}