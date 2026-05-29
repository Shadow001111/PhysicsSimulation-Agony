#include "SimulationRenderer.h"
#include "Simulation.h"

namespace PS_AGONY
{
    void SimulationRenderer::init()
    {
        initShaders();
        initBuffers();
    }

    void SimulationRenderer::render(const Simulation& simulation)
    {
    }

    void SimulationRenderer::initShaders()
    {
        // Circles
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
        // Circles
        const float circleVertices[3 * 2] =
        {
            0.0f, 2.0f,
            1.7321f, -1.0f,
            -1.7321f, -1.0f
        };

        vboCircle.create()
    }
}