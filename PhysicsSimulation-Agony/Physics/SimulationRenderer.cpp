#include "SimulationRenderer.h"
#include "Simulation.h"

#include "Core/TracyProfiler.h"

namespace PS_AGONY
{
    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        camera.position = { 0.0, 0.0 };
        camera.viewRange = { 10.0, 10.0 };

        initShaders();
        initBuffers();
    }

    void SimulationRenderer::render(const Simulation& simulation)
    {
        TRACY_SCOPE_N("SimulationRenderer render");

        // Camera.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        renderCircles(simulation, viewProjectionMatrix);
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
    }

    void SimulationRenderer::initBuffers()
    {
        // Circles.
        const float circleVertices[3 * 2] =
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

    void SimulationRenderer::renderCircles(const Simulation& simulation, const Mat4& viewProjectionMatrix)
    {
        // Bodies reference.
        const auto& bodies = simulation.getBodies();

        // Circles reference.
        const auto& circles = simulation.getCircles();
        const size_t circleCount = circles.getCount();
        if (circleCount == 0)
        {
            return;
        }

        // Reserve space.
        ensureCircleInstanceVboCapacity(circleCount);

        // Prepare instance data.
        for (size_t i = 0; i < circleCount; i++)
        {
            const Real radius = circles.radius[i];
            const BodyIndex bodyIndex = circles.bodyIndices[i];

            const Real positionX = bodies.positionX[bodyIndex];
            const Real positionY = bodies.positionY[bodyIndex];

            circleResources.renderData[i].x = positionX;
            circleResources.renderData[i].y = positionY;
            circleResources.renderData[i].radius = radius;
        }

        // Move data to gpu.
        circleResources.instanceVbo.write(circleResources.renderData.data(), circleCount * sizeof(CircleRenderData));

        // Bind things, set uniforms, draw.
        circleResources.shader.use();
        circleResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        circleResources.vao.bind();

        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, circleCount);
    }

    void SimulationRenderer::ensureCircleInstanceVboCapacity(size_t count)
    {
        constexpr size_t SIZEOF_RENDER_DATA = sizeof(CircleRenderData);

        const size_t neededCapacity = count * SIZEOF_RENDER_DATA;
        const size_t currentCapacity = circleResources.instanceVbo.getCapacity();

        if (neededCapacity <= currentCapacity) return;

        const size_t newCapacity = neededCapacity + (neededCapacity >> 1);

        circleResources.instanceVbo.create();
        circleResources.instanceVbo.allocateStorage(newCapacity, GL_DYNAMIC_STORAGE_BIT);

        circleResources.vao.bindVertexBuffer(1, circleResources.instanceVbo.getID(), 0, SIZEOF_RENDER_DATA);

        circleResources.vao.enableAttribute(1);
        circleResources.vao.setFloatAttribute(1, 3, 0, 1);
        circleResources.vao.setAttributeDivisor(1, 1);

        circleResources.renderData.resize(newCapacity / SIZEOF_RENDER_DATA);
    }
}