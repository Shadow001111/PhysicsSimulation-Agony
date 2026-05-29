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

        // Bodies reference.
        const auto& bodies = simulation.getBodies();

        // Circles.
        shaderCircle.use();
        shaderCircle.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        vaoCircle.bind();

        const auto& circles = simulation.getCircles();
        const size_t circleCount = circles.radius.size();
        for (size_t i = 0; i < circleCount; i++)
        {
            const Real radius = circles.radius[i];
            const BodyIndex bodyIndex = circles.bodyIndices[i];

            const Real positionX = bodies.positionX[bodyIndex];
            const Real positionY = bodies.positionY[bodyIndex];

            shaderCircle.setVec2("position", positionX, positionY);
            shaderCircle.setFloat("radius", radius);
            shaderCircle.setVec3("color", 1.0f, 1.0f, 1.0f);

            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
    }

    void SimulationRenderer::initShaders()
    {
        // Circles.
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/circle.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/circle.frag" }
            };
            
            shaderCircle.create(sources);
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

        vboCircle.create();
        vboCircle.allocateStorage(sizeof(circleVertices), 0, circleVertices);

        vaoCircle.create();
        vaoCircle.bindVertexBuffer(0, vboCircle.getID(), 0, sizeof(float) * 2);

        vaoCircle.enableAttribute(0);
        vaoCircle.setFloatAttribute(0, 2, 0, 0);
    }
}