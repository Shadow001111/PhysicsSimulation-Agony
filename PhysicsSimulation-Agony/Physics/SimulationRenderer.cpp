#include "SimulationRenderer.h"
#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include "Graphics/TextureLoader.h"

namespace PS_AGONY
{
    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        initShaders();
        initBuffers();

        {
            TextureLoader::TextureLoadParams params
            {
                .desiredChannels = 4,
                .createMipmaps = true,
                .compression = TextureCompression::Format::NONE,
                .isHDR = false
            };
            TextureLoader::createTexture2DFromImage(hardcodedTexture, "res/Textures/Francis.png", params);
        }
    }

    void SimulationRenderer::render(const Simulation& simulation)
    {
        TRACY_SCOPE_N("SimulationRenderer render");

        // Set references.
        bodies = simulation.getBodies();
        circles = simulation.getCircles();
        boxes = simulation.getBoxes();

        // Camera.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Render.
        renderBodies(viewProjectionMatrix);
		//renderBodyAABBs(viewProjectionMatrix);
		//renderBroadPhaseAABBs(simulation, viewProjectionMatrix);
    }

    void SimulationRenderer::initShaders()
    {
        // Circle.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/Bodies/Circle/circle_textured.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/Bodies/Circle/circle_textured.frag" }
            };
            
            circleResources.shader.create(sources);
        }

        // Box.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/Bodies/Box/box.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/Bodies/Box/box.frag" }
            };

            boxResources.shader.create(sources);
        }

        // AABB.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/aabb.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/aabb.frag" }
            };
            aabbResources.shader.create(sources);
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
    }

    void SimulationRenderer::renderBodies(const Mat4& viewProjectionMatrix)
    {
        renderCircleBodies(viewProjectionMatrix);
        renderBoxBodies(viewProjectionMatrix);

		//renderBodyCentersOfMass(viewProjectionMatrix);
        //renderBodyPositions(viewProjectionMatrix);
        //renderBodyTruePositions(viewProjectionMatrix);
		//renderBodyAABBs(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyCentersOfMass(const Mat4& viewProjectionMatrix)
    {
        const size_t count = bodies.getCount();
        if (count == 0) return;

        // Reserve space.
        circleResources.instanceData.resize(count);

        // Prepare instance data.
		const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
		const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT centerXPtr = bodies.localCenterOfMassX;
        const Real* CORE_RESTRICT centerYPtr = bodies.localCenterOfMassY;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        const Real* CORE_RESTRICT truePositionXPtr = bodies.truePositionX;
        const Real* CORE_RESTRICT truePositionYPtr = bodies.truePositionY;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

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
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* CORE_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* CORE_RESTRICT rotationPtr = bodies.rotation;
        const BodyTextureId* CORE_RESTRICT bodyTextureIdPtr = bodies.textureId;

        const Real* CORE_RESTRICT radiusPtr = circles.radius;
        const BodyIndex* CORE_RESTRICT bodyIndexPtr = circles.bodyIndices;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];

            renderDataPtr[i].positionX = positionXPtr[bodyIndex];
            renderDataPtr[i].positionY = positionYPtr[bodyIndex];
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = rotationPtr[bodyIndex];
            renderDataPtr[i].radius = radiusPtr[i];
			renderDataPtr[i].color = 0xFFFFFF;
            renderDataPtr[i].textureId = bodyTextureIdPtr[bodyIndex];
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
        const Real* CORE_RESTRICT positionXPtr = bodies.positionX;
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY;
        const Real* CORE_RESTRICT localCOMXPtr = bodies.localCenterOfMassX;
        const Real* CORE_RESTRICT localCOMYPtr = bodies.localCenterOfMassY;
        const Real* CORE_RESTRICT rotationPtr = bodies.rotation;
        const BodyTextureId* CORE_RESTRICT bodyTextureIdPtr = bodies.textureId;

        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth;
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight;
        const BodyIndex* CORE_RESTRICT bodyIndexPtr = boxes.bodyIndices;

        BoxInstanceData* CORE_RESTRICT renderDataPtr = boxResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];

            renderDataPtr[i].positionX = positionXPtr[bodyIndex];
            renderDataPtr[i].positionY = positionYPtr[bodyIndex];
            renderDataPtr[i].localCOMX = localCOMXPtr[bodyIndex];
            renderDataPtr[i].localCOMY = localCOMYPtr[bodyIndex];
            renderDataPtr[i].rotation = rotationPtr[bodyIndex];
            renderDataPtr[i].halfWidth = halfWidthPtr[i];
            renderDataPtr[i].halfHeight = halfHeightPtr[i];
            renderDataPtr[i].color = 0xFFFFFF;
            renderDataPtr[i].textureId = bodyTextureIdPtr[bodyIndex];
        }

        // Render.
        renderBoxShapes(viewProjectionMatrix);
    }

    void SimulationRenderer::renderBodyAABBs(const Mat4& viewProjectionMatrix)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;
        
        // Reserve space.
        aabbResources.instanceData.resize(bodyCount);

        // Prepare instance data.
        const Real* CORE_RESTRICT minXPtr = bodies.aabb.minX;
        const Real* CORE_RESTRICT minYPtr = bodies.aabb.minY;
        const Real* CORE_RESTRICT maxXPtr = bodies.aabb.maxX;
        const Real* CORE_RESTRICT maxYPtr = bodies.aabb.maxY;

        AABB* CORE_RESTRICT renderDataPtr = aabbResources.instanceData.data();

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
		aabbResources.instanceData.clear();
        simulation.fetchBroadPhaseAABBs(aabbResources.instanceData);
        if (aabbResources.instanceData.empty()) return;

        // Render.
		renderAABBs({ 0.0f, 1.0f, 0.0f }, viewProjectionMatrix);
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

        hardcodedTexture.bindUnit(0);
        circleResources.shader.setInt("uHardCodedTexture", 0);

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

    void SimulationRenderer::renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix)
    {
        const size_t count = aabbResources.instanceData.size();
		if (count == 0) return;

		// Reserve space.
		ensureAABBInstanceVboCapacity(count);

        // Move data to gpu.
        aabbResources.instanceVbo.write(aabbResources.instanceData.data(), count * sizeof(AABB));

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

        vao.enableAttribute(6);
        vao.setIntAttribute(6, 1, sizeof(float) * 7, 1);
        vao.setAttributeDivisor(6, 1);
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

        vao.enableAttribute(6);
        vao.setIntAttribute(6, 1, sizeof(float) * 8, 1);
        vao.setAttributeDivisor(6, 1);
    }

    void SimulationRenderer::ensureAABBInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_INSTANCE = sizeof(AABB);

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
}