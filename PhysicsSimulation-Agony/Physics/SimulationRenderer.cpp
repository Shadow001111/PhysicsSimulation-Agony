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

        // Circles.
        shaderCircle.use();

        shaderCircle.setVec2("position", 0.0f, 0.0f);
        shaderCircle.setFloat("radius", 1.0f);
        shaderCircle.setVec3("color", 1.0f, 1.0f, 1.0f);
        shaderCircle.setMat4("viewMatrix", viewMatrix);
        shaderCircle.setMat4("projectionMatrix", projectionMatrix);

        vaoCircle.bind();

        glDrawArrays(GL_TRIANGLES, 0, 3);
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