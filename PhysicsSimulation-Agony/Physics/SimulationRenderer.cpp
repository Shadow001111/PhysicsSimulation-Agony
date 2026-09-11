#include "SimulationRenderer.h"
#include "Simulation.h"
#include "Constants.h"
#include "FastCosSin.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <cmath>

namespace PS_AGONY
{
    [[nodiscard]] static uint32_t idToHexColor(uint32_t x) noexcept
    {
        x ^= x >> 16;
        x *= 0x7feb352dU;
        x ^= x >> 15;
        x *= 0x846ca68bU;
        x ^= x >> 16;
        return x & 0xFFFFFF;
    }

    [[nodiscard]] __forceinline static Real interpolateRotation(
        const Real& rot,
        const Real& oldRot,
        const Real& wrapCount,
        const Real& alpha)
    {
        const Real fullRotation = rot + wrapCount * PS_AGONY::Constants::TWO_PI;

        Real interpolatedRotation = oldRot + (fullRotation - oldRot) * alpha;

        Real isRotNegativeMask = interpolatedRotation < 0;
        interpolatedRotation += isRotNegativeMask * Constants::TWO_PI;

        return interpolatedRotation;
    }



    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        initShaders();
        initBuffers();
    }

    void SimulationRenderer::renderSimulation(const Simulation& simulation, Real simRenderAlpha, const AABB& cameraAABB)
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

        // Fetch colliders which overlap camera AABB.
        queryCollidersForRender(simulation, cameraAABB);

        // Render.
        renderColliders(viewProjectionMatrix, simRenderAlpha);

        //renderBodyCentersOfMass(viewProjectionMatrix); // Red.
        //renderBodyTruePositions(viewProjectionMatrix); // Green.
        //renderBodyPositions(viewProjectionMatrix); // Blue.
        renderColliderAABBs(foundColliders, viewProjectionMatrix);

        renderBroadPhaseAABBs(simulation, viewProjectionMatrix, cameraAABB);
        renderContactPoints(simulation, viewProjectionMatrix, cameraAABB);
        renderSprings(simulation, viewProjectionMatrix, simRenderAlpha);
        renderJoints(simulation, viewProjectionMatrix, simRenderAlpha);
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

            circleResources.instanceData.resize(1);
            CircleInstanceData& data = circleResources.instanceData[0];

            data.positionX = static_cast<float>(circleParams->base.position.x);
            data.positionY = static_cast<float>(circleParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(circleParams->base.rotation);
            data.radius = static_cast<float>(circleParams->radius);

            data.color = 0x00FF00;

            renderCircleShapes(viewProjectionMatrix);
        }
        else if (type == BodyType::Box)
        {
            const auto* boxParams = static_cast<const BoxCreateParams*>(params);

            boxResources.instanceData.resize(1);
            BoxInstanceData& data = boxResources.instanceData[0];

            data.positionX = static_cast<float>(boxParams->base.position.x);
            data.positionY = static_cast<float>(boxParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(boxParams->base.rotation);
            data.halfWidth = static_cast<float>(boxParams->size.x * 0.5);
            data.halfHeight = static_cast<float>(boxParams->size.y * 0.5);

            data.color = 0x00FF00;

            renderBoxShapes(viewProjectionMatrix);
        }
        else if (type == BodyType::Polygon)
        {
            const auto* polyParams = static_cast<const PolygonCreateParams*>(params);

            const size_t vertCount = polyParams->verticesCount;
            if (vertCount < 3) return;

            ensurePolygonBufferCapacity(vertCount, 1);

            polygonResources.vertexData.resize(vertCount);
            polygonResources.instanceData.resize(1);
            polygonResources.drawCommands.resize(1);

            glm::vec2* verts = polygonResources.vertexData.data();
            for (size_t i = 0; i < vertCount; i++)
            {
                verts[i].x = static_cast<float>(polyParams->localVertices[i].x);
                verts[i].y = static_cast<float>(polyParams->localVertices[i].y);
            }

            PolygonInstanceData& inst = polygonResources.instanceData[0];
            inst.positionX = static_cast<float>(polyParams->base.position.x);
            inst.positionY = static_cast<float>(polyParams->base.position.y);
            inst.localCOMX = 0.0f;
            inst.localCOMY = 0.0f;
            inst.rotation = static_cast<float>(polyParams->base.rotation);
            inst.color = 0x00FF00;

            DrawArraysIndirectCommand& cmd = polygonResources.drawCommands[0];
            cmd.count = static_cast<uint32_t>(vertCount);
            cmd.instanceCount = 1;
            cmd.first = 0;
            cmd.baseInstance = 0;

            renderPolygonShapes(viewProjectionMatrix);
        }
    }

    void SimulationRenderer::initShaders()
    {
        // Circle.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/Bodies/circle.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/Bodies/circle.frag" }
            };

            circleResources.shader.create(sources);
        }

        // Box.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/Bodies/box.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/Bodies/box.frag" }
            };

            boxResources.shader.create(sources);
        }

        // Polygon.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER,   "res/Shaders/Bodies/polygon.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/Bodies/polygon.frag" }
            };
            polygonResources.shader.create(sources);
        }

        // AABB.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/aabb.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/aabb.frag" }
            };
            aabbResources.shader.create(sources);
        }

        // Spring.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/spring.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/spring.frag" }
            };
            springResources.shader.create(sources);
        }
    }

    void SimulationRenderer::initBuffers()
    {
        // Circle.
        {
            const float circleVertices[] =
            {
                0.0f, 2.0f,
                1.7321f, -1.0f,
                -1.7321f, -1.0f
            };

            circleResources.vbo.create();
            circleResources.vbo.allocateStorage(sizeof(circleVertices), 0, circleVertices);

            circleResources.vao.create();
            circleResources.vao.bindVertexBuffer(0, circleResources.vbo.getID(), 0, sizeof(float) * 2);

            circleResources.vao.enableAttribute(0);
            circleResources.vao.setFloatAttribute(0, 2, 0, 0);

            ensureCircleInstanceVboCapacity(64);
        }

        // Box.
        {
            const float vertices[] =
            {
                0.0f, 0.0f,
                1.0f, 0.0f,
                1.0f, 1.0f,
                0.0f, 1.0f
            };

            boxResources.vbo.create();
            boxResources.vbo.allocateStorage(sizeof(vertices), 0, vertices);

            boxResources.vao.create();
            boxResources.vao.bindVertexBuffer(0, boxResources.vbo.getID(), 0, sizeof(float) * 2);

            boxResources.vao.enableAttribute(0);
            boxResources.vao.setFloatAttribute(0, 2, 0, 0);

            // Initial instance VBO capacity
            ensureBoxInstanceVboCapacity(64);
        }

        // Polygon - VAO only; buffers are grown on first use.
        {
            polygonResources.vao.create();
            ensurePolygonBufferCapacity(256, 64);
        }

        // AABB.
        {
            const float vertices[] =
            {
                0.0f, 0.0f,
                1.0f, 0.0f,
                1.0f, 1.0f,
                0.0f, 1.0f
            };

            aabbResources.vbo.create();
            aabbResources.vbo.allocateStorage(sizeof(vertices), 0, vertices);

            aabbResources.vao.create();
            aabbResources.vao.bindVertexBuffer(0, aabbResources.vbo.getID(), 0, sizeof(float) * 2);

            aabbResources.vao.enableAttribute(0);
            aabbResources.vao.setFloatAttribute(0, 2, 0, 0);

            // Initial instance VBO capacity
            ensureAABBInstanceVboCapacity(64);
        }

        // Spring.
        {
            springResources.vao.create();
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

    void SimulationRenderer::renderColliders(const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        TRACY_SCOPE_N("Render colliders");

        renderCircleColliders(foundColliderShapes[(size_t)BodyType::Circle], viewProjectionMatrix, simRenderAlpha);
        renderBoxColliders(foundColliderShapes[(size_t)BodyType::Box], viewProjectionMatrix, simRenderAlpha);
        renderPolygonColliders(foundColliderShapes[(size_t)BodyType::Polygon], viewProjectionMatrix, simRenderAlpha);
    }

    void SimulationRenderer::renderBodyCentersOfMass(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT centerXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT centerYPtr = bodies.localCenterOfMassY;

        CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyPositions(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;

        CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyTruePositions(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT truePositionXPtr = bodies.worldCenterX;
        const Real* ECSTASY_RESTRICT truePositionYPtr = bodies.worldCenterY;

        CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render circle colliders");

        // Reserve space.
        circleResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT oldPositionXPtr = bodies.renderOldOffsetX;
        const Real* ECSTASY_RESTRICT oldPositionYPtr = bodies.renderOldOffsetY;
        const Real* ECSTASY_RESTRICT oldRotationPtr = bodies.renderOldRotation;
        const Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        const ObjectIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;

        CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = givenColliders[i];

            const ObjectIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const Real posX = positionXPtr[bodyIndex];
            const Real posY = positionYPtr[bodyIndex];
            const Real oldPosX = oldPositionXPtr[bodyIndex];
            const Real oldPosY = oldPositionYPtr[bodyIndex];

            const Real interpolatedBodyPosX = oldPosX + (posX - oldPosX) * simRenderAlpha;
            const Real interpolatedBodyPosY = oldPosY + (posY - oldPosY) * simRenderAlpha;

            const Real interpolatedBodyRotation = interpolateRotation(
                rotationPtr[bodyIndex],
                oldRotationPtr[bodyIndex],
                rotationWrapCountPtr[bodyIndex],
                simRenderAlpha
            );

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const auto [bodyCos, bodySin] = FastCosSin::order4Scalar(interpolatedBodyRotation);

            const Real worldOffX = localOffX * bodyCos - localOffY * bodySin;
            const Real worldOffY = localOffX * bodySin + localOffY * bodyCos;

            renderDataPtr[i].positionX = interpolatedBodyPosX + worldOffX;
            renderDataPtr[i].positionY = interpolatedBodyPosY + worldOffY;
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = interpolatedBodyRotation + localRot;
            renderDataPtr[i].radius = radiusPtr[shapeIndex];
            renderDataPtr[i].color = 0xFFFFFF;
        }

        // Render.
        renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render box colliders");

        // Reserve space.
        boxResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT oldPositionXPtr = bodies.renderOldOffsetX;
        const Real* ECSTASY_RESTRICT oldPositionYPtr = bodies.renderOldOffsetY;
        const Real* ECSTASY_RESTRICT oldRotationPtr = bodies.renderOldRotation;
        const Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        const ObjectIndex* ECSTASY_RESTRICT colliderBodyIndexPtr = colliders.bodyIndex;
        const ObjectIndex* ECSTASY_RESTRICT colliderShapeIndexPtr = colliders.shapeIndex;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetXPtr = colliders.localOffsetX;
        const Real* ECSTASY_RESTRICT colliderLocalOffsetYPtr = colliders.localOffsetY;
        const Real* ECSTASY_RESTRICT colliderLocalRotationPtr = colliders.localRotation;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;

        BoxInstanceData* ECSTASY_RESTRICT renderDataPtr = boxResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = givenColliders[i];

            const ObjectIndex bodyIndex = colliderBodyIndexPtr[colliderIndex];
            const ObjectIndex shapeIndex = colliderShapeIndexPtr[colliderIndex];

            const Real posX = positionXPtr[bodyIndex];
            const Real posY = positionYPtr[bodyIndex];
            const Real oldPosX = oldPositionXPtr[bodyIndex];
            const Real oldPosY = oldPositionYPtr[bodyIndex];

            const Real interpolatedBodyPosX = oldPosX + (posX - oldPosX) * simRenderAlpha;
            const Real interpolatedBodyPosY = oldPosY + (posY - oldPosY) * simRenderAlpha;

            const Real interpolatedBodyRotation = interpolateRotation(
                rotationPtr[bodyIndex],
                oldRotationPtr[bodyIndex],
                rotationWrapCountPtr[bodyIndex],
                simRenderAlpha
            );

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const auto [bodyCos, bodySin] = FastCosSin::order4Scalar(interpolatedBodyRotation);

            const Real worldOffX = localOffX * bodyCos - localOffY * bodySin;
            const Real worldOffY = localOffX * bodySin + localOffY * bodyCos;

            renderDataPtr[i].positionX = interpolatedBodyPosX + worldOffX;
            renderDataPtr[i].positionY = interpolatedBodyPosY + worldOffY;
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = interpolatedBodyRotation + localRot;
            renderDataPtr[i].halfWidth = halfWidthPtr[shapeIndex];
            renderDataPtr[i].halfHeight = halfHeightPtr[shapeIndex];
            renderDataPtr[i].color = 0xFFFFFF;
        }

        // Render.
        renderBoxShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        const size_t count = givenColliders.size();
        if (count == 0) return;

        TRACY_SCOPE_N("Render polygon colliders");

        // Body SoA pointers.
        const Real* ECSTASY_RESTRICT oldPositionXPtr = bodies.renderOldOffsetX;
        const Real* ECSTASY_RESTRICT oldPositionYPtr = bodies.renderOldOffsetY;
        const Real* ECSTASY_RESTRICT oldRotationPtr = bodies.renderOldRotation;
        const Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

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

        ensurePolygonBufferCapacity(totalVertices, count);

        // Build CPU-side data.
        polygonResources.vertexData.resize(totalVertices);
        polygonResources.instanceData.resize(count);
        polygonResources.drawCommands.resize(count);

        glm::vec2* ECSTASY_RESTRICT verts = polygonResources.vertexData.data();
        PolygonInstanceData* ECSTASY_RESTRICT instData = polygonResources.instanceData.data();
        DrawArraysIndirectCommand* ECSTASY_RESTRICT cmds = polygonResources.drawCommands.data();

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

            const Real posX = positionXPtr[bodyIndex];
            const Real posY = positionYPtr[bodyIndex];
            const Real oldPosX = oldPositionXPtr[bodyIndex];
            const Real oldPosY = oldPositionYPtr[bodyIndex];

            const Real interpolatedBodyPosX = oldPosX + (posX - oldPosX) * simRenderAlpha;
            const Real interpolatedBodyPosY = oldPosY + (posY - oldPosY) * simRenderAlpha;

            const Real interpolatedBodyRotation = interpolateRotation(
                rotationPtr[bodyIndex],
                oldRotationPtr[bodyIndex],
                rotationWrapCountPtr[bodyIndex],
                simRenderAlpha
            );

            const Real localOffX = colliderLocalOffsetXPtr[colliderIndex];
            const Real localOffY = colliderLocalOffsetYPtr[colliderIndex];
            const Real localRot = colliderLocalRotationPtr[colliderIndex];

            const auto [bodyCos, bodySin] = FastCosSin::order4Scalar(interpolatedBodyRotation);

            const Real worldOffX = localOffX * bodyCos - localOffY * bodySin;
            const Real worldOffY = localOffX * bodySin + localOffY * bodyCos;

            instData[i].positionX = interpolatedBodyPosX + worldOffX;
            instData[i].positionY = interpolatedBodyPosY + worldOffY;
            instData[i].localCOMX = localCOMXPtr[bodyIndex];
            instData[i].localCOMY = localCOMYPtr[bodyIndex];
            instData[i].rotation = interpolatedBodyRotation + localRot;
            instData[i].color = 0xFFFFFF;

            cmds[i].count = vertCount;
            cmds[i].instanceCount = 1;
            cmds[i].first = vertexOffset;
            cmds[i].baseInstance = 0;

            vertexOffset += vertCount;
        }

        renderPolygonShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render collider AABBs");

        const size_t colliderCount = givenColliders.size();
        if (colliderCount == 0) return;

        // Reserve space.
        aabbResources.instanceData.resize(colliderCount);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT minXPtr = colliders.aabbMinX;
        const Real* ECSTASY_RESTRICT minYPtr = colliders.aabbMinY;
        const Real* ECSTASY_RESTRICT maxXPtr = colliders.aabbMaxX;
        const Real* ECSTASY_RESTRICT maxYPtr = colliders.aabbMaxY;

        FloatAABB* ECSTASY_RESTRICT renderDataPtr = aabbResources.instanceData.data();

        for (size_t i = 0; i < colliderCount; i++)
        {
            ColliderIndex colliderIndex = givenColliders[i];
            renderDataPtr[i].minX = minXPtr[colliderIndex];
            renderDataPtr[i].minY = minYPtr[colliderIndex];
            renderDataPtr[i].maxX = maxXPtr[colliderIndex];
            renderDataPtr[i].maxY = maxYPtr[colliderIndex];
        }

        renderAABBs({ 1.0f, 0.0f, 0.0f }, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render broad phase AABBs");

        // Fetch AABBs.
        aabbResources.aabbs.clear();
        aabbResources.instanceData.clear();

        const auto& bfcd = simulation.getBroadPhaseCollisionDetector();
        bfcd.queryLeafNodeAABBsInShape(BroadPhaseInternal::AABBQuery(cameraAABB), aabbResources.aabbs);

        if (aabbResources.aabbs.empty()) return;

        // Allocate space for instance data
        aabbResources.instanceData.resize(aabbResources.aabbs.size());

        // Copy and cast values. 
        for (size_t i = 0; i < aabbResources.aabbs.size(); i++)
        {
            aabbResources.instanceData[i].minX = static_cast<float>(aabbResources.aabbs[i].minX);
            aabbResources.instanceData[i].minY = static_cast<float>(aabbResources.aabbs[i].minY);
            aabbResources.instanceData[i].maxX = static_cast<float>(aabbResources.aabbs[i].maxX);
            aabbResources.instanceData[i].maxY = static_cast<float>(aabbResources.aabbs[i].maxY);
        }

        // Render.
        renderAABBs({ 0.0f, 1.0f, 0.0f }, viewProjectionMatrix);
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
        circleResources.instanceData.clear();

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

                auto& renderData = circleResources.instanceData.emplace_back();

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
        renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        TRACY_SCOPE_N("Render springs");

        const auto& springs = simulation.getSprings();
        const size_t springCount = springs.getCount();
        if (springCount == 0) return;

        const size_t vertexCount = springCount * 2;
        ensureSpringBufferCapacity(vertexCount);

        springResources.vertexData.resize(vertexCount);
        LineVertex* ECSTASY_RESTRICT verts = springResources.vertexData.data();

        const Real* ECSTASY_RESTRICT oldPositionXPtr = bodies.renderOldOffsetX;
        const Real* ECSTASY_RESTRICT oldPositionYPtr = bodies.renderOldOffsetY;
        const Real* ECSTASY_RESTRICT oldRotationPtr = bodies.renderOldRotation;
        const Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        for (size_t i = 0; i < springCount; i++)
        {
            const ObjectIndex bodyIndexA = springs.bodyIndexA[i];
            const ObjectIndex bodyIndexB = springs.bodyIndexB[i];

            // Interpolate body A transform.
            const Real posAx = positionXPtr[bodyIndexA];
            const Real posAy = positionYPtr[bodyIndexA];
            const Real oldPosAx = oldPositionXPtr[bodyIndexA];
            const Real oldPosAy = oldPositionYPtr[bodyIndexA];

            const Real interpolatedPosAx = oldPosAx + (posAx - oldPosAx) * simRenderAlpha;
            const Real interpolatedPosAy = oldPosAy + (posAy - oldPosAy) * simRenderAlpha;

            const Real fullOldRotationA = oldRotationPtr[bodyIndexA];
            const Real fullNewRotationA = rotationPtr[bodyIndexA] + (rotationWrapCountPtr[bodyIndexA] * PS_AGONY::Constants::TWO_PI);
            const Real interpolatedRotationA = fullOldRotationA + (fullNewRotationA - fullOldRotationA) * simRenderAlpha;

            const Real cosA = std::cos(interpolatedRotationA);
            const Real sinA = std::sin(interpolatedRotationA);

            // Interpolate body B transform.
            const Real posBx = positionXPtr[bodyIndexB];
            const Real posBy = positionYPtr[bodyIndexB];
            const Real oldPosBx = oldPositionXPtr[bodyIndexB];
            const Real oldPosBy = oldPositionYPtr[bodyIndexB];

            const Real interpolatedPosBx = oldPosBx + (posBx - oldPosBx) * simRenderAlpha;
            const Real interpolatedPosBy = oldPosBy + (posBy - oldPosBy) * simRenderAlpha;

            const Real fullOldRotationB = oldRotationPtr[bodyIndexB];
            const Real fullNewRotationB = rotationPtr[bodyIndexB] + (rotationWrapCountPtr[bodyIndexB] * PS_AGONY::Constants::TWO_PI);
            const Real interpolatedRotationB = fullOldRotationB + (fullNewRotationB - fullOldRotationB) * simRenderAlpha;

            const Real cosB = std::cos(interpolatedRotationB);
            const Real sinB = std::sin(interpolatedRotationB);

            // Compute world positions of spring endpoints.
            const float worldAx = interpolatedPosAx + (springs.localAnchorA[i].x * cosA - springs.localAnchorA[i].y * sinA);
            const float worldAy = interpolatedPosAy + (springs.localAnchorA[i].x * sinA + springs.localAnchorA[i].y * cosA);

            const float worldBx = interpolatedPosBx + (springs.localAnchorB[i].x * cosB - springs.localAnchorB[i].y * sinB);
            const float worldBy = interpolatedPosBy + (springs.localAnchorB[i].x * sinB + springs.localAnchorB[i].y * cosB);

            // Compute displacement.
            const float dx = worldBx - worldAx;
            const float dy = worldBy - worldAy;
            const float currentLength = std::sqrt(dx * dx + dy * dy);
            const float restLength = static_cast<float>(springs.restLength[i]);
            const float displacement = currentLength - restLength;

            // Compute color.
            uint32_t color;
            {
                // Normalize strain into [-1.0, 1.0].
                float t = std::clamp(displacement / restLength, -1.0f, 1.0f);

                // Calculate weights for each endpoint color.
                float wBlue = std::fmax(0.0f, -t);    // Active (< 0).
                float wRed = std::fmax(0.0f, t);     // Active (> 0).
                float wGrey = 1.0f - std::fabs(t);    // Active (= 0).

                // Blend RGB channels.
                uint32_t r = uint32_t(255.0f * wRed + 187.0f * wGrey);
                uint32_t g = uint32_t(187.0f * wGrey);
                uint32_t b = uint32_t(255.0f * wBlue + 187.0f * wGrey);

                color = (r << 16) | (g << 8) | b;
            }

            size_t vIdx = i * 2;
            verts[vIdx].x = worldAx;
            verts[vIdx].y = worldAy;
            verts[vIdx].color = color;

            verts[vIdx + 1].x = worldBx;
            verts[vIdx + 1].y = worldBy;
            verts[vIdx + 1].color = color;
        }

        springResources.vbo.write(springResources.vertexData.data(), vertexCount * sizeof(LineVertex));

        springResources.shader.use();
        springResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        springResources.vao.bind();
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertexCount));
    }

    void SimulationRenderer::renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha)
    {
        TRACY_SCOPE_N("Render joints");

        const auto& joints = simulation.getJoints();
        const size_t jointCount = joints.getCount();
        if (jointCount == 0) return;

        // 2 lines per joint = 4 vertices.
        const size_t vertexCount = jointCount * 4;
        ensureSpringBufferCapacity(vertexCount);

        springResources.vertexData.resize(vertexCount);
        LineVertex* ECSTASY_RESTRICT verts = springResources.vertexData.data();

        const Real* ECSTASY_RESTRICT oldPositionXPtr = bodies.renderOldOffsetX;
        const Real* ECSTASY_RESTRICT oldPositionYPtr = bodies.renderOldOffsetY;
        const Real* ECSTASY_RESTRICT oldRotationPtr = bodies.renderOldRotation;
        const Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        for (size_t i = 0; i < jointCount; i++)
        {
            const ObjectIndex bodyIndexA = joints.bodyIndexA[i];
            const ObjectIndex bodyIndexB = joints.bodyIndexB[i];

            // Interpolate body A transform.
            const Real posAx = positionXPtr[bodyIndexA];
            const Real posAy = positionYPtr[bodyIndexA];
            const Real oldPosAx = oldPositionXPtr[bodyIndexA];
            const Real oldPosAy = oldPositionYPtr[bodyIndexA];

            const Real interpolatedPosAx = oldPosAx + (posAx - oldPosAx) * simRenderAlpha;
            const Real interpolatedPosAy = oldPosAy + (posAy - oldPosAy) * simRenderAlpha;

            const Real fullOldRotationA = oldRotationPtr[bodyIndexA];
            const Real fullNewRotationA = rotationPtr[bodyIndexA] + (rotationWrapCountPtr[bodyIndexA] * PS_AGONY::Constants::TWO_PI);
            const Real interpolatedRotationA = fullOldRotationA + (fullNewRotationA - fullOldRotationA) * simRenderAlpha;

            const Real cosA = std::cos(interpolatedRotationA);
            const Real sinA = std::sin(interpolatedRotationA);

            // Interpolate body B transform.
            const Real posBx = positionXPtr[bodyIndexB];
            const Real posBy = positionYPtr[bodyIndexB];
            const Real oldPosBx = oldPositionXPtr[bodyIndexB];
            const Real oldPosBy = oldPositionYPtr[bodyIndexB];

            const Real interpolatedPosBx = oldPosBx + (posBx - oldPosBx) * simRenderAlpha;
            const Real interpolatedPosBy = oldPosBy + (posBy - oldPosBy) * simRenderAlpha;

            const Real fullOldRotationB = oldRotationPtr[bodyIndexB];
            const Real fullNewRotationB = rotationPtr[bodyIndexB] + (rotationWrapCountPtr[bodyIndexB] * PS_AGONY::Constants::TWO_PI);
            const Real interpolatedRotationB = fullOldRotationB + (fullNewRotationB - fullOldRotationB) * simRenderAlpha;

            const Real cosB = std::cos(interpolatedRotationB);
            const Real sinB = std::sin(interpolatedRotationB);

            // Compute world positions of joint anchors.
            const float worldAx = interpolatedPosAx + (joints.localAnchorA[i].x * cosA - joints.localAnchorA[i].y * sinA);
            const float worldAy = interpolatedPosAy + (joints.localAnchorA[i].x * sinA + joints.localAnchorA[i].y * cosA);

            const float worldBx = interpolatedPosBx + (joints.localAnchorB[i].x * cosB - joints.localAnchorB[i].y * sinB);
            const float worldBy = interpolatedPosBy + (joints.localAnchorB[i].x * sinB + joints.localAnchorB[i].y * cosB);

            constexpr uint32_t color = 0x00FFFF;
            const size_t vIdx = i * 4;

            // Line 1: Body A position to Anchor A
            verts[vIdx + 0].x = static_cast<float>(interpolatedPosAx);
            verts[vIdx + 0].y = static_cast<float>(interpolatedPosAy);
            verts[vIdx + 0].color = color;

            verts[vIdx + 1].x = worldAx;
            verts[vIdx + 1].y = worldAy;
            verts[vIdx + 1].color = color;

            // Line 2: Body B position to Anchor B
            verts[vIdx + 2].x = static_cast<float>(interpolatedPosBx);
            verts[vIdx + 2].y = static_cast<float>(interpolatedPosBy);
            verts[vIdx + 2].color = color;

            verts[vIdx + 3].x = worldBx;
            verts[vIdx + 3].y = worldBy;
            verts[vIdx + 3].color = color;
        }

        springResources.vbo.write(springResources.vertexData.data(), vertexCount * sizeof(LineVertex));

        springResources.shader.use();
        springResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        springResources.vao.bind();
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertexCount));
    }

    void SimulationRenderer::renderCircleShapes(const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render circle shapes");

        const size_t count = circleResources.instanceData.size();
        if (count == 0) return;

        // Reserve space.
        ensureCircleInstanceVboCapacity(count);

        // Move data to gpu.
        circleResources.instanceVbo.write(circleResources.instanceData.data(), count * sizeof(CircleInstanceData));

        // Bind things, set uniforms.
        circleResources.shader.use();
        circleResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        circleResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, count);
    }

    void SimulationRenderer::renderBoxShapes(const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render box shapes");

        const size_t count = boxResources.instanceData.size();
        if (count == 0) return;

        // Reserve space.
        ensureBoxInstanceVboCapacity(count);

        // Move data to gpu.
        boxResources.instanceVbo.write(boxResources.instanceData.data(), count * sizeof(BoxInstanceData));

        // Bind things, set uniforms.
        boxResources.shader.use();
        boxResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        boxResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, count);
    }

    void SimulationRenderer::renderPolygonShapes(const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render polygon shapes");

        const size_t polygonCount = polygonResources.drawCommands.size();
        if (polygonCount == 0) return;

        const size_t vertexBytes = polygonResources.vertexData.size() * sizeof(glm::vec2);
        const size_t instanceBytes = polygonCount * sizeof(PolygonInstanceData);
        const size_t cmdBytes = polygonCount * sizeof(DrawArraysIndirectCommand);

        // Upload vertex positions.
        polygonResources.vertexVbo.write(polygonResources.vertexData.data(), vertexBytes);

        // Upload per-polygon transforms.
        polygonResources.instanceVbo.write(polygonResources.instanceData.data(), instanceBytes);

        // Upload indirect draw commands.
        polygonResources.indirectBuf.write(polygonResources.drawCommands.data(), cmdBytes);

        // Bind and draw.
        polygonResources.shader.use();
        polygonResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        polygonResources.vao.bind();
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, polygonResources.indirectBuf.getID());

        glMultiDrawArraysIndirect(GL_TRIANGLE_FAN, nullptr, static_cast<GLsizei>(polygonCount), 0);
    }

    void SimulationRenderer::renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render AABBs");

        const size_t count = aabbResources.instanceData.size();
        if (count == 0) return;

        // Reserve space.
        ensureAABBInstanceVboCapacity(count);

        // Move data to gpu.
        aabbResources.instanceVbo.write(aabbResources.instanceData.data(), count * sizeof(FloatAABB));

        // Bind things, set uniforms.
        aabbResources.shader.use();
        aabbResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);
        aabbResources.shader.setVec3("color", color.x, color.y, color.z);

        aabbResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_LINE_LOOP, 0, 4, count);
    }

    size_t SimulationRenderer::getMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += getVectorMemoryUsage(foundColliders);
        for (const auto& vec : foundColliderShapes)
            total += getVectorMemoryUsage(vec);

        total += getVectorMemoryUsage(circleResources.instanceData);
        total += getVectorMemoryUsage(boxResources.instanceData);

        total += getVectorMemoryUsage(polygonResources.vertexData);
        total += getVectorMemoryUsage(polygonResources.instanceData);
        total += getVectorMemoryUsage(polygonResources.drawCommands);

        total += getVectorMemoryUsage(springResources.vertexData);

        total += getVectorMemoryUsage(aabbResources.aabbs);
        total += getVectorMemoryUsage(aabbResources.instanceData);

        return total;
    }

    size_t SimulationRenderer::getVideoMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += circleResources.vbo.getCapacity();
        total += circleResources.instanceVbo.getCapacity();

        total += boxResources.vbo.getCapacity();
        total += boxResources.instanceVbo.getCapacity();

        total += polygonResources.vertexVbo.getCapacity();
        total += polygonResources.instanceVbo.getCapacity();
        total += polygonResources.indirectBuf.getCapacity();

        total += springResources.vbo.getCapacity();

        total += aabbResources.vbo.getCapacity();
        total += aabbResources.instanceVbo.getCapacity();

        return total;
    }

    void SimulationRenderer::ensureCircleInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_INSTANCE = sizeof(CircleInstanceData);

        auto& vao = circleResources.vao;
        auto& instanceVbo = circleResources.instanceVbo;

        const size_t neededCapacity = count * SIZEOF_INSTANCE;
        const size_t currentCapacity = instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        instanceVbo.create();
        instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        vao.bindVertexBuffer(1, instanceVbo.getID(), 0, SIZEOF_INSTANCE);

        vao.enableAttribute(1);
        vao.setFloatAttribute(1, 2, 0, 1);
        vao.setAttributeDivisor(1, 1);

        vao.enableAttribute(2);
        vao.setFloatAttribute(2, 2, sizeof(float) * 2, 1);
        vao.setAttributeDivisor(2, 1);

        vao.enableAttribute(3);
        vao.setFloatAttribute(3, 1, sizeof(float) * 4, 1);
        vao.setAttributeDivisor(3, 1);

        vao.enableAttribute(4);
        vao.setFloatAttribute(4, 1, sizeof(float) * 5, 1);
        vao.setAttributeDivisor(4, 1);

        vao.enableAttribute(5);
        vao.setIntAttribute(5, 1, sizeof(float) * 6, 1);
        vao.setAttributeDivisor(5, 1);
    }

    void SimulationRenderer::ensureBoxInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_INSTANCE = sizeof(BoxInstanceData);

        auto& vao = boxResources.vao;
        auto& instanceVbo = boxResources.instanceVbo;

        const size_t neededCapacity = count * SIZEOF_INSTANCE;
        const size_t currentCapacity = instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        instanceVbo.create();
        instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        vao.bindVertexBuffer(1, instanceVbo.getID(), 0, SIZEOF_INSTANCE);

        vao.enableAttribute(1);
        vao.setFloatAttribute(1, 2, 0, 1);
        vao.setAttributeDivisor(1, 1);

        vao.enableAttribute(2);
        vao.setFloatAttribute(2, 2, sizeof(float) * 2, 1);
        vao.setAttributeDivisor(2, 1);

        vao.enableAttribute(3);
        vao.setFloatAttribute(3, 1, sizeof(float) * 4, 1);
        vao.setAttributeDivisor(3, 1);

        vao.enableAttribute(4);
        vao.setFloatAttribute(4, 2, sizeof(float) * 5, 1);
        vao.setAttributeDivisor(4, 1);

        vao.enableAttribute(5);
        vao.setIntAttribute(5, 1, sizeof(float) * 7, 1);
        vao.setAttributeDivisor(5, 1);
    }

    void SimulationRenderer::ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount)
    {
        constexpr size_t SIZEOF_VERTEX = sizeof(glm::vec2);
        constexpr size_t SIZEOF_INSTANCE = sizeof(PolygonInstanceData);
        constexpr size_t SIZEOF_CMD = sizeof(DrawArraysIndirectCommand);

        auto& vao = polygonResources.vao;
        auto& vertexVbo = polygonResources.vertexVbo;
        auto& instanceVbo = polygonResources.instanceVbo;
        auto& indirectBuf = polygonResources.indirectBuf;

        const size_t neededVertexBytes = vertexCount * SIZEOF_VERTEX;
        if (neededVertexBytes > vertexVbo.getCapacity())
        {
            const size_t newCapacity = neededVertexBytes + (neededVertexBytes >> 1);

            vertexVbo.create();
            vertexVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

            vao.bindVertexBuffer(0, vertexVbo.getID(), 0, SIZEOF_VERTEX);
            vao.enableAttribute(0);
            vao.setFloatAttribute(0, 2, 0, 0);
        }

        const size_t neededInstanceBytes = polygonCount * SIZEOF_INSTANCE;
        if (neededInstanceBytes > instanceVbo.getCapacity())
        {
            const size_t newCapacity = neededInstanceBytes + (neededInstanceBytes >> 1);

            instanceVbo.create();
            instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, instanceVbo.getID());
        }

        const size_t neededCmdBytes = polygonCount * SIZEOF_CMD;
        if (neededCmdBytes > indirectBuf.getCapacity())
        {
            const size_t newCapacity = neededCmdBytes + (neededCmdBytes >> 1);

            indirectBuf.create();
            indirectBuf.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);
        }
    }

    void SimulationRenderer::ensureAABBInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_INSTANCE = sizeof(FloatAABB);

        auto& vao = aabbResources.vao;
        auto& instanceVbo = aabbResources.instanceVbo;

        const size_t neededCapacity = count * SIZEOF_INSTANCE;
        const size_t currentCapacity = instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        instanceVbo.create();
        instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        vao.bindVertexBuffer(1, instanceVbo.getID(), 0, SIZEOF_INSTANCE);

        vao.enableAttribute(1);
        vao.setFloatAttribute(1, 4, 0, 1);
        vao.setAttributeDivisor(1, 1);
    }

    void SimulationRenderer::ensureSpringBufferCapacity(size_t vertexCount)
    {
        constexpr size_t SIZEOF_VERTEX = sizeof(LineVertex);

        auto& vao = springResources.vao;
        auto& vbo = springResources.vbo;

        const size_t neededBytes = vertexCount * SIZEOF_VERTEX;
        if (neededBytes <= vbo.getCapacity()) return;

        const size_t newCapacity = neededBytes + (neededBytes >> 1);

        vbo.create();
        vbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        vao.bindVertexBuffer(0, vbo.getID(), 0, SIZEOF_VERTEX);

        // Setup positions attribute (location = 0)
        vao.enableAttribute(0);
        vao.setFloatAttribute(0, 2, 0, 0);

        // Setup packed Hex Color attribute (location = 1)
        vao.enableAttribute(1);
        vao.setIntAttribute(1, 1, sizeof(float) * 2, 0);
    }
}