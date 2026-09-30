#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "Physics/Simulation.h"
#include "Physics/Constants.h"
#include "Physics/FastCosSin.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <cmath>

namespace PS_AGONY_RENDER
{
    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        shapeRenderer.init();
    }

    void SimulationRenderer::renderSimulation(
        const Simulation& simulation,
        Real simRenderAlpha,
        const AABB& cameraAABB,
        const RenderOptions& renderOptions,
        GLuint idTextureID
    )
    {
        TRACY_SCOPE_N("Render simulation");

        // Set references.
        bodies = simulation.getBodies();
        colliders = simulation.getColliders();
        circles = simulation.getCircles();
        boxes = simulation.getBoxes();
        polygons = simulation.getPolygons();

        // Camera.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Built once per frame and threaded through every render function below,
        // instead of each one re-fetching the same body pointers and redoing the
        // same lerp/wrap math. When renderOptions.interpolation is false, sampling
        // a body transform skips that math entirely (see BodyTransformContext::sample).
        const BodyTransformContext transformContext = BodyTransformContext::make(bodies, simRenderAlpha, renderOptions.interpolation);

        // Fetch colliders which overlap camera AABB.
        queryCollidersForRender(simulation, cameraAABB);

        // Render.
        renderColliders(viewProjectionMatrix, transformContext);

        // Color-only from here: avoids a feedback loop when sampling the id texture.
        glDrawBuffers(1, &RenderAttachments::colorDrawBuffer);

        if (renderOptions.outlineColliders)
        {
            renderOutlines(idTextureID, renderOptions.outlineColor);
        }

        renderSprings(simulation, viewProjectionMatrix, transformContext, cameraAABB);
        renderJoints(simulation, viewProjectionMatrix, transformContext, cameraAABB);
        renderPrismaticJoints(simulation, viewProjectionMatrix, transformContext, cameraAABB);
        renderRods(simulation, viewProjectionMatrix, transformContext, cameraAABB);

        if (renderOptions.colliderAABBs)
        {
            renderColliderAABBs(foundColliders, viewProjectionMatrix);
        }
        if (renderOptions.broadPhaseAABBs)
        {
            renderBroadPhaseAABBs(simulation, viewProjectionMatrix, cameraAABB);
        }
        if (renderOptions.contactPoints)
        {
            renderContactPoints(simulation, viewProjectionMatrix, cameraAABB);
        }

        //renderBodyCentersOfMass(viewProjectionMatrix, cameraAABB); // Red.
        //renderBodyTruePositions(viewProjectionMatrix, cameraAABB); // Green.
        //renderBodyPositions(viewProjectionMatrix, cameraAABB); // Blue.
    }

    void SimulationRenderer::renderObjectPreview(const void* params, BodyType type)
    {
        if (params == nullptr) return;

        // Get matrices.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Draw body depending on the type.
        if (type == BodyType::Circle)
        {
            const auto* circleParams = static_cast<const CircleCreateParams*>(params);

            circleInstanceData.resize(1);
            ShapeRenderer::CircleInstanceData& data = circleInstanceData[0];

            data.positionX = static_cast<float>(circleParams->base.position.x);
            data.positionY = static_cast<float>(circleParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(circleParams->base.rotation);
            data.radius = static_cast<float>(circleParams->radius);

            data.color = 0x00FF00;

            shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
        }
        else if (type == BodyType::Box)
        {
            const auto* boxParams = static_cast<const BoxCreateParams*>(params);

            boxInstanceData.resize(1);
            ShapeRenderer::BoxInstanceData& data = boxInstanceData[0];

            data.positionX = static_cast<float>(boxParams->base.position.x);
            data.positionY = static_cast<float>(boxParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(boxParams->base.rotation);
            data.halfWidth = static_cast<float>(boxParams->size.x * 0.5);
            data.halfHeight = static_cast<float>(boxParams->size.y * 0.5);

            data.color = 0x00FF00;

            shapeRenderer.renderBoxShapes(boxInstanceData, viewProjectionMatrix);
        }
        else if (type == BodyType::Polygon)
        {
            const auto* polyParams = static_cast<const PolygonCreateParams*>(params);

            const size_t vertCount = polyParams->verticesCount;
            if (vertCount < 3) return;

            shapeRenderer.ensurePolygonBufferCapacity(vertCount, 1);

            polygonVertexData.resize(vertCount);
            polygonInstanceData.resize(1);
            polygonDrawCommands.resize(1);

            glm::vec2* verts = polygonVertexData.data();
            for (size_t i = 0; i < vertCount; i++)
            {
                verts[i].x = static_cast<float>(polyParams->localVertices[i].x);
                verts[i].y = static_cast<float>(polyParams->localVertices[i].y);
            }

            ShapeRenderer::PolygonInstanceData& inst = polygonInstanceData[0];
            inst.positionX = static_cast<float>(polyParams->base.position.x);
            inst.positionY = static_cast<float>(polyParams->base.position.y);
            inst.localCOMX = 0.0f;
            inst.localCOMY = 0.0f;
            inst.rotation = static_cast<float>(polyParams->base.rotation);
            inst.color = 0x00FF00;

            DrawArraysIndirectCommand& cmd = polygonDrawCommands[0];
            cmd.count = static_cast<uint32_t>(vertCount);
            cmd.instanceCount = 1;
            cmd.first = 0;
            cmd.baseInstance = 0;

            shapeRenderer.renderPolygonShapes(polygonVertexData, polygonInstanceData, polygonDrawCommands, viewProjectionMatrix);
        }
    }

    void SimulationRenderer::queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB)
    {
        {
            TRACY_SCOPE_N("Find colliders for render");

            const auto& collisionDetector = simulation.getBroadPhaseCollisionDetector();

            foundColliders.clear();
            collisionDetector.queryCollidersInShape(BroadPhaseInternal::AABBQuery(cameraAABB), foundColliders);
        }
        {
            TRACY_SCOPE_N("Partition found colliders");

            for (auto& vec : foundColliderShapes)
            {
                vec.clear();
            }

            for (ColliderIndex collider : foundColliders)
            {
                BodyType shape = colliders.shapeType[collider];
                foundColliderShapes[(size_t)shape].push_back(collider);
            }
        }
    }

    size_t SimulationRenderer::getMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += getVectorMemoryUsage(foundColliders);
        for (const auto& vec : foundColliderShapes)
            total += getVectorMemoryUsage(vec);

        total += getVectorMemoryUsage(circleInstanceData);
        total += getVectorMemoryUsage(simplePointInstanceData);
        total += getVectorMemoryUsage(boxInstanceData);

        total += getVectorMemoryUsage(polygonVertexData);
        total += getVectorMemoryUsage(polygonInstanceData);
        total += getVectorMemoryUsage(polygonDrawCommands);

        total += getVectorMemoryUsage(lineVertexData);

        total += getVectorMemoryUsage(aabbInstanceData);

        return total;
    }

    size_t SimulationRenderer::getVideoMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += shapeRenderer.circleResources.vbo.getCapacity();
        total += shapeRenderer.circleResources.instanceVbo.getCapacity();

        total += shapeRenderer.simplePointCircleResources.vbo.getCapacity();
        total += shapeRenderer.simplePointCircleResources.instanceVbo.getCapacity();

        total += shapeRenderer.boxResources.vbo.getCapacity();
        total += shapeRenderer.boxResources.instanceVbo.getCapacity();

        total += shapeRenderer.polygonResources.vertexVbo.getCapacity();
        total += shapeRenderer.polygonResources.instanceVbo.getCapacity();
        total += shapeRenderer.polygonResources.indirectBuf.getCapacity();

        total += shapeRenderer.lineResources.vbo.getCapacity();

        total += shapeRenderer.aabbResources.vbo.getCapacity();
        total += shapeRenderer.aabbResources.instanceVbo.getCapacity();

        return total;
    }

    void SimulationRenderer::renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor)
    {
        shapeRenderer.renderOutlines(idTextureID, outlineColor);
    }
}