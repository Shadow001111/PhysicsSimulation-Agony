#include "SimulationRenderer.h"
#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

namespace PS_AGONY
{
    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        initShaders();
        initBuffers();
    }

    void SimulationRenderer::render(const Simulation& simulation)
    {
        TRACY_SCOPE_N("SimulationRenderer render");

        // Set references.
        bodies = simulation.getBodies();
        circles = simulation.getCircles();

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
        // Circles.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/circle.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/circle.frag" }
            };
            
            circleResources.shader.create(sources);
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
        // Circles.
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
		renderBodyCentersOfMass(viewProjectionMatrix);
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
		const Real* CORE_RESTRICT rotationPtr = bodies.rotation;
        const Real* CORE_RESTRICT centerXPtr = bodies.localCenterOfMassX;
        const Real* CORE_RESTRICT centerYPtr = bodies.localCenterOfMassY;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
			const Vec2 localCenterOfMass = { centerXPtr[i], centerYPtr[i] };

            const Vec2 rotatedLocalCenterOfMass = {
                localCenterOfMass.x * std::cos(rotationPtr[i]) - localCenterOfMass.y * std::sin(rotationPtr[i]),
                localCenterOfMass.x * std::sin(rotationPtr[i]) + localCenterOfMass.y * std::cos(rotationPtr[i])
			};

            const Vec2 worldCenterOfMass = {
                positionXPtr[i] + rotatedLocalCenterOfMass.x,
                positionYPtr[i] + rotatedLocalCenterOfMass.y
			};

            renderDataPtr[i].x = worldCenterOfMass.x;
            renderDataPtr[i].y = worldCenterOfMass.y;
            renderDataPtr[i].rotation = 0.785f;
            renderDataPtr[i].radius = 0.03f;
			renderDataPtr[i].color = 0xFF0000;
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
        const Real* CORE_RESTRICT rotationPtr = bodies.rotation;
        const Real* CORE_RESTRICT radiusPtr = circles.radius;
        const BodyIndex* CORE_RESTRICT bodyIndexPtr = circles.bodyIndices;

        CircleInstanceData* CORE_RESTRICT renderDataPtr = circleResources.instanceData.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];

            renderDataPtr[i].x = positionXPtr[bodyIndex];
            renderDataPtr[i].y = positionYPtr[bodyIndex];
            renderDataPtr[i].rotation = rotationPtr[bodyIndex];
            renderDataPtr[i].radius = radiusPtr[i];
			renderDataPtr[i].color = 0xFFFFFF;
        }

        // Render.
		renderCircleShapes(viewProjectionMatrix);
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

        // Draw.
        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, count);
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

        const size_t neededCapacity = count * SIZEOF_INSTANCE;
        const size_t currentCapacity = circleResources.instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        circleResources.instanceVbo.create();
        circleResources.instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        circleResources.vao.bindVertexBuffer(1, circleResources.instanceVbo.getID(), 0, SIZEOF_INSTANCE);

        circleResources.vao.enableAttribute(1);
        circleResources.vao.setFloatAttribute(1, 2, 0, 1);
        circleResources.vao.setAttributeDivisor(1, 1);

        circleResources.vao.enableAttribute(2);
        circleResources.vao.setFloatAttribute(2, 1, sizeof(float) * 2, 1);
        circleResources.vao.setAttributeDivisor(2, 1);

        circleResources.vao.enableAttribute(3);
        circleResources.vao.setFloatAttribute(3, 1, sizeof(float) * 3, 1);
        circleResources.vao.setAttributeDivisor(3, 1);

		circleResources.vao.enableAttribute(4);
        circleResources.vao.setIntAttribute(4, 1, sizeof(float) * 4, 1);
		circleResources.vao.setAttributeDivisor(4, 1);
    }

    void SimulationRenderer::ensureAABBInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_INSTANCE = sizeof(AABB);

        const size_t neededCapacity = count * SIZEOF_INSTANCE;
        const size_t currentCapacity = aabbResources.instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        aabbResources.instanceVbo.create();
        aabbResources.instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        aabbResources.vao.bindVertexBuffer(1, aabbResources.instanceVbo.getID(), 0, SIZEOF_INSTANCE);

        aabbResources.vao.enableAttribute(1);
        aabbResources.vao.setFloatAttribute(1, 4, 0, 1);
        aabbResources.vao.setAttributeDivisor(1, 1);
    }
}