#include "SimulationRenderer.h"
#include "Simulation.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include "EcstasyGraphics/TextureLoader.h"

#include <iostream>

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


    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        initShaders();
        initBuffers();
    }

    void SimulationRenderer::renderSimulation(const Simulation& simulation)
    {
        TRACY_SCOPE_N("SimulationRenderer render");

        // Set references.
        bodies = simulation.getBodies();
        circles = simulation.getCircles();
        boxes = simulation.getBoxes();
        polygons = simulation.getPolygons();

        // Camera.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Render.
        renderBodies(viewProjectionMatrix);
		//renderBroadPhaseAABBs(simulation, viewProjectionMatrix);
        //renderContactPoints(simulation, viewProjectionMatrix);
        renderSprings(simulation, viewProjectionMatrix);
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
            const auto* circleParams = static_cast<const Simulation::CircleCreateParams*>(params);

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
            const auto* boxParams = static_cast<const Simulation::BoxCreateParams*>(params);

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
            const auto* polyParams = static_cast<const Simulation::PolygonCreateParams*>(params);

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

        // Circle.
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

    void SimulationRenderer::renderBodies(const Mat4& viewProjectionMatrix)
    {
        renderCircleBodies(viewProjectionMatrix);
        renderBoxBodies(viewProjectionMatrix);
        renderPolygonBodies(viewProjectionMatrix);

		//renderBodyCentersOfMass(viewProjectionMatrix); // Red.
        //renderBodyTruePositions(viewProjectionMatrix); // Green.
        //renderBodyPositions(viewProjectionMatrix); // Blue.
		//renderBodyAABBs(viewProjectionMatrix);
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

    void SimulationRenderer::renderCircleBodies(const Mat4& viewProjectionMatrix)
    {
        const size_t count = circles.getCount();
        if (count == 0) return;

		// Reserve space.
		circleResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius;
        const ObjectIndex* ECSTASY_RESTRICT bodyIndexPtr = circles.bodyIndices;

        CircleInstanceData* ECSTASY_RESTRICT renderDataPtr = circleResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ObjectIndex bodyIndex = bodyIndexPtr[i];

            renderDataPtr[i].positionX = positionXPtr[bodyIndex];
            renderDataPtr[i].positionY = positionYPtr[bodyIndex];
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = rotationPtr[bodyIndex];
            renderDataPtr[i].radius = radiusPtr[i];
			renderDataPtr[i].color = 0xFFFFFF;
        }

        // Render.
		renderCircleShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBoxBodies(const Mat4& viewProjectionMatrix)
    {
        const size_t count = boxes.getCount();
        if (count == 0) return;

        // Reserve space.
        boxResources.instanceData.resize(count);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight;
        const ObjectIndex* ECSTASY_RESTRICT bodyIndexPtr = boxes.bodyIndices;

        BoxInstanceData* ECSTASY_RESTRICT renderDataPtr = boxResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const ObjectIndex bodyIndex = bodyIndexPtr[i];

            renderDataPtr[i].positionX = positionXPtr[bodyIndex];
            renderDataPtr[i].positionY = positionYPtr[bodyIndex];
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = rotationPtr[bodyIndex];
            renderDataPtr[i].halfWidth = halfWidthPtr[i];
            renderDataPtr[i].halfHeight = halfHeightPtr[i];
            renderDataPtr[i].color = 0xFFFFFF;
        }

        // Render.
        renderBoxShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderPolygonBodies(const Mat4& viewProjectionMatrix)
    {
        const size_t count = polygons.getCount();
        if (count == 0) return;

        // Body SoA pointers.
        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* ECSTASY_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        // Polygon SoA pointers.
        const ObjectIndex* ECSTASY_RESTRICT bodyIndexPtr = polygons.bodyIndices;
        const VerticesContainer* ECSTASY_RESTRICT localVertsPtr = polygons.localVertices;

        // Count total vertices needed this frame.
        size_t totalVertices = 0;
        for (size_t i = 0; i < count; i++)
            totalVertices += localVertsPtr[i].size();

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
            const ObjectIndex         bodyIndex = bodyIndexPtr[i];
            const VerticesContainer& vc = localVertsPtr[i];
            const uint32_t           vertCount = static_cast<uint32_t>(vc.size());
            const Vec2* src = vc.data();

            for (uint32_t v = 0; v < vertCount; v++)
            {
                verts[vertexOffset + v].x = src[v].x;
                verts[vertexOffset + v].y = src[v].y;
            }

            instData[i].positionX = positionXPtr[bodyIndex];
            instData[i].positionY = positionYPtr[bodyIndex];
            instData[i].localCOMX = localCOMXPtr[bodyIndex];
            instData[i].localCOMY = localCOMYPtr[bodyIndex];
            instData[i].rotation = rotationPtr[bodyIndex];
            instData[i].color = 0xFFFFFF;

            cmds[i].count = vertCount;
            cmds[i].instanceCount = 1;
            cmds[i].first = vertexOffset;
            cmds[i].baseInstance = 0;

            vertexOffset += vertCount;
        }

        renderPolygonShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyAABBs(const Mat4& viewProjectionMatrix)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;
        
        // Reserve space.
        aabbResources.instanceData.resize(bodyCount);

        // Prepare instance data.
        const Real* ECSTASY_RESTRICT minXPtr = bodies.aabb.minX;
        const Real* ECSTASY_RESTRICT minYPtr = bodies.aabb.minY;
        const Real* ECSTASY_RESTRICT maxXPtr = bodies.aabb.maxX;
        const Real* ECSTASY_RESTRICT maxYPtr = bodies.aabb.maxY;

        FloatAABB* ECSTASY_RESTRICT renderDataPtr = aabbResources.instanceData.data();

        for (size_t i = 0; i < bodyCount; i++)
        {
			renderDataPtr[i].minX = minXPtr[i];
			renderDataPtr[i].minY = minYPtr[i];
			renderDataPtr[i].maxX = maxXPtr[i];
			renderDataPtr[i].maxY = maxYPtr[i];
        }

		renderAABBs({ 1.0f, 0.0f, 0.0f }, viewProjectionMatrix);
    }

    void SimulationRenderer::renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix)
    {
        // Fetch AABBs.
        aabbResources.aabbs.clear();
        aabbResources.instanceData.clear();

        simulation.getBroadPhaseAABBs(aabbResources.aabbs);

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

    void SimulationRenderer::renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix)
    {
        const auto& collisionData = simulation.getBodyCollisionData();

        const size_t collisionCount = collisionData.size();
        if (collisionCount == 0) return;

        // Reserve space.
        circleResources.instanceData.clear();

        // Prepare instance data.
        for (size_t i = 0; i < collisionCount; i++)
        {
            const auto& collData = collisionData[i];

            for (size_t j = 0; j < collData.contactCount; j++)
            {
                const auto& contact = collData.contactPoints[j];

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

    void SimulationRenderer::renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix)
    {
        const auto& springs = simulation.getSprings();
        const size_t springCount = springs.getCount();
        if (springCount == 0) return;

        const size_t vertexCount = springCount * 2;
        ensureSpringBufferCapacity(vertexCount);

        springResources.vertexData.resize(vertexCount);
        LineVertex* ECSTASY_RESTRICT verts = springResources.vertexData.data();

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX;
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY;
        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation;

        for (size_t i = 0; i < springCount; ++i)
        {
            const ObjectIndex idxA = springs.bodyIndexA[i];
            const ObjectIndex idxB = springs.bodyIndexB[i];

            const float rA = static_cast<float>(rotationPtr[idxA]);
            const float cA = std::cos(rA);
            const float sA = std::sin(rA);

            const float rB = static_cast<float>(rotationPtr[idxB]);
            const float cB = std::cos(rB);
            const float sB = std::sin(rB);

            // 1. Transform local coordinates into world space positions
            const float wAx = static_cast<float>(positionXPtr[idxA]) + (springs.localAnchorA[i].x * cA - springs.localAnchorA[i].y * sA);
            const float wAy = static_cast<float>(positionYPtr[idxA]) + (springs.localAnchorA[i].x * sA + springs.localAnchorA[i].y * cA);

            const float wBx = static_cast<float>(positionXPtr[idxB]) + (springs.localAnchorB[i].x * cB - springs.localAnchorB[i].y * sB);
            const float wBy = static_cast<float>(positionYPtr[idxB]) + (springs.localAnchorB[i].x * sB + springs.localAnchorB[i].y * cB);

            // 2. Dynamic Strain Color-Coding Calculation
            const float dx = wBx - wAx;
            const float dy = wBy - wAy;
            const float currentLength = std::sqrt(dx * dx + dy * dy);
            const float restLength = static_cast<float>(springs.restLength[i]);
            const float displacement = currentLength - restLength;

            uint32_t color = 0xBBBBBB; // Default neutral state gray
            if (displacement > 0.02f)
            {
                // Tension (Stretched) -> Interpolate to Red
                float factor = std::min(displacement / (restLength + 0.001f), 1.0f);
                uint32_t greenBlue = static_cast<uint32_t>(187.0f * (1.0f - factor));
                color = (0xFF << 16) | (greenBlue << 8) | greenBlue;
            }
            else if (displacement < -0.02f)
            {
                // Compression (Squeezed) -> Interpolate to Blue
                float factor = std::min(std::abs(displacement) / (restLength + 0.001f), 1.0f);
                uint32_t redGreen = static_cast<uint32_t>(187.0f * (1.0f - factor));
                color = (redGreen << 16) | (redGreen << 8) | 0xFF;
            }

            // 3. Stage Structural Vertex Data Pairs
            size_t vIdx = i * 2;
            verts[vIdx].x = wAx;
            verts[vIdx].y = wAy;
            verts[vIdx].color = color;

            verts[vIdx + 1].x = wBx;
            verts[vIdx + 1].y = wBy;
            verts[vIdx + 1].color = color;
        }

        // 4. Stream and Bind to GPU Pipeline
        springResources.vbo.write(springResources.vertexData.data(), vertexCount * sizeof(LineVertex));

        springResources.shader.use();
        springResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        springResources.vao.bind();
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertexCount));
    }

    void SimulationRenderer::renderCircleShapes(const Mat4& viewProjectionMatrix)
    {
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