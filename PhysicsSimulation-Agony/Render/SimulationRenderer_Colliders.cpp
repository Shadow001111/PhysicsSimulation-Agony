#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "Physics/Simulation.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace Render
{
    void SimulationRenderer::renderColliders(const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render colliders");

        renderCircleColliders(foundColliderShapes[(size_t)BodyType::Circle], viewProjectionMatrix, transformContext);
        renderBoxColliders(foundColliderShapes[(size_t)BodyType::Box], viewProjectionMatrix, transformContext);
        renderPolygonColliders(foundColliderShapes[(size_t)BodyType::Polygon], viewProjectionMatrix, transformContext);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderBodyCentersOfMass(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT centerXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT centerYPtr = bodies.localCenterOfMassY;

        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const glm::vec2 localCenterOfMass = { centerXPtr[i], centerYPtr[i] };

            const glm::vec2 worldCenterOfMass = {
                positionXPtr[i] + localCenterOfMass.x,
                positionYPtr[i] + localCenterOfMass.y
            };

            renderDataPtr[i].positionX = worldCenterOfMass.x;
            renderDataPtr[i].positionY = worldCenterOfMass.y;
            renderDataPtr[i].localCOMX = 0.0f;
            renderDataPtr[i].localCOMY = 0.0f;
            renderDataPtr[i].rotation = 0.785f;
            renderDataPtr[i].radius = 0.03f;
            renderDataPtr[i].color = 0xFF0000;
        }

        // Render.
        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderBodyPositions(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;

        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            renderDataPtr[i].positionX = positionXPtr[i];
            renderDataPtr[i].positionY = positionYPtr[i];
            renderDataPtr[i].localCOMX = 0.0f;
            renderDataPtr[i].localCOMY = 0.0f;
            renderDataPtr[i].rotation = 0.0f;
            renderDataPtr[i].radius = 0.03f;
            renderDataPtr[i].color = 0x0000FF;
        }

        // Render.
        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderBodyTruePositions(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT truePositionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT truePositionYPtr = bodies.worldCenterY;

        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            renderDataPtr[i].positionX = truePositionXPtr[i];
            renderDataPtr[i].positionY = truePositionYPtr[i];
            renderDataPtr[i].localCOMX = 0.0f;
            renderDataPtr[i].localCOMY = 0.0f;
            renderDataPtr[i].rotation = 0.0f;
            renderDataPtr[i].radius = 0.03f;
            renderDataPtr[i].color = 0x00FF00;
        }

        // Render.
        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render circle colliders");

        // Reserve space.
        circleInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        const ObjectIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        ShapeRenderer::CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = givenColliders[i];

            const ObjectIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

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

    void SimulationRenderer::renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render box colliders");

        // Reserve space.
        boxInstanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        const ObjectIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        ShapeRenderer::BoxInstanceData* ECSTASY_RESTRICT renderDataPtr = boxInstanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = givenColliders[i];

            const ObjectIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

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

    void SimulationRenderer::renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render polygon colliders");

        // Body SoA pointers.
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;

        // Collider SoA pointers.
        const ObjectIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        // Polygon SoA pointers.
        const VerticesContainer* ECSTASY_RESTRICT localVertsPtr = polygons.localVertices;

        // Count total vertices needed this frame.
        size_t totalVertices = 0;
        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = givenColliders[i];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];
            totalVertices += localVertsPtr[shapeIndex].size();
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
            const ColliderIndex colliderIndex = givenColliders[i];

            const ObjectIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const VerticesContainer& vc = localVertsPtr[shapeIndex];
            const uint32_t vertCount = static_cast<uint32_t>(vc.size());
            const Vec2* src = vc.data();

            for (uint32_t v = 0; v < vertCount; v++)
            {
                verts[vertexOffset + v].x = src[v].x;
                verts[vertexOffset + v].y = src[v].y;
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

    void SimulationRenderer::renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix)
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
            ColliderIndex colliderIndex = givenColliders[i];
            renderDataPtr[i].minX = minXPtr[colliderIndex];
            renderDataPtr[i].minY = minYPtr[colliderIndex];
            renderDataPtr[i].maxX = maxXPtr[colliderIndex];
            renderDataPtr[i].maxY = maxYPtr[colliderIndex];
        }

        shapeRenderer.renderAABBShapes(aabbInstanceData, { 1.0f, 0.0f, 0.0f }, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render broad phase AABBs");

        // Fetch AABBs.
        queriedAABBs.clear();
        aabbInstanceData.clear();

        const auto& bfcd = simulation.getBroadPhaseCollisionDetector();
        bfcd.queryLeafNodeAABBsInShape(BroadPhaseInternal::AABBQuery(cameraAABB), queriedAABBs);

        if (queriedAABBs.empty()) return;

        // Allocate space for instance data
        aabbInstanceData.resize(queriedAABBs.size());

        // Copy and cast values. 
        for (size_t i = 0; i < queriedAABBs.size(); i++)
        {
            aabbInstanceData[i].minX = static_cast<float>(queriedAABBs[i].minX);
            aabbInstanceData[i].minY = static_cast<float>(queriedAABBs[i].minY);
            aabbInstanceData[i].maxX = static_cast<float>(queriedAABBs[i].maxX);
            aabbInstanceData[i].maxY = static_cast<float>(queriedAABBs[i].maxY);
        }

        // Render.
        shapeRenderer.renderAABBShapes(aabbInstanceData, { 0.0f, 1.0f, 0.0f }, viewProjectionMatrix);
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
        circleInstanceData.clear();

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

                auto& renderData = circleInstanceData.emplace_back();

                renderData.positionX = contact.x;
                renderData.positionY = contact.y;
                renderData.localCOMX = 0.0f;
                renderData.localCOMY = 0.0f;
                renderData.rotation = 0.0f;
                renderData.radius = 0.05f;
                renderData.color = idToHexColor(collData.contactIds[j]);
            }
        }

        // Render.
        shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
    }
}