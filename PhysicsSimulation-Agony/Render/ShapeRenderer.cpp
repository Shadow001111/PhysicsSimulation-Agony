#include "ShapeRenderer.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace Render
{
    void ShapeRenderer::init()
    {
        TRACY_SCOPE_N("ShapeRenderer init");

        initShaders();
        initBuffers();
    }

    void ShapeRenderer::initShaders()
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

        // Lines (springs, joints, rods, ...).
        {
            std::vector<Shader::ShaderSource> sources = {
                { GL_VERTEX_SHADER, "res/Shaders/spring.vert" },
                { GL_FRAGMENT_SHADER, "res/Shaders/spring.frag" }
            };
            lineResources.shader.create(sources);
        }
    }

    void ShapeRenderer::initBuffers()
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

        // Lines (springs, joints, rods, ...).
        {
            lineResources.vao.create();
        }
    }

    void ShapeRenderer::renderCircleShapes(std::span<const CircleInstanceData> instances, const glm::mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render circle shapes");

        const size_t count = instances.size();
        if (count == 0) return;

        // Reserve space.
        ensureCircleInstanceVboCapacity(count);

        // Move data to gpu.
        circleResources.instanceVbo.write(instances.data(), count * sizeof(CircleInstanceData));

        // Bind things, set uniforms.
        circleResources.shader.use();
        circleResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        circleResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, count);
    }

    void ShapeRenderer::renderBoxShapes(std::span<const BoxInstanceData> instances, const glm::mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render box shapes");

        const size_t count = instances.size();
        if (count == 0) return;

        // Reserve space.
        ensureBoxInstanceVboCapacity(count);

        // Move data to gpu.
        boxResources.instanceVbo.write(instances.data(), count * sizeof(BoxInstanceData));

        // Bind things, set uniforms.
        boxResources.shader.use();
        boxResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        boxResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, count);
    }

    void ShapeRenderer::renderPolygonShapes(
        std::span<const glm::vec2> vertices,
        std::span<const PolygonInstanceData> instances,
        std::span<const DrawArraysIndirectCommand> drawCommands,
        const glm::mat4& viewProjectionMatrix
    )
    {
        TRACY_SCOPE_N("Render polygon shapes");

        const size_t polygonCount = drawCommands.size();
        if (polygonCount == 0) return;

        const size_t vertexBytes = vertices.size() * sizeof(glm::vec2);
        const size_t instanceBytes = instances.size() * sizeof(PolygonInstanceData);
        const size_t cmdBytes = drawCommands.size() * sizeof(Ecstasy::OpenGL::DrawArraysIndirectCommand);

        // Upload vertex positions.
        polygonResources.vertexVbo.write(vertices.data(), vertexBytes);

        // Upload per-polygon transforms.
        polygonResources.instanceVbo.write(instances.data(), instanceBytes);

        // Upload indirect draw commands.
        polygonResources.indirectBuf.write(drawCommands.data(), cmdBytes);

        // Bind and draw.
        polygonResources.shader.use();
        polygonResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        polygonResources.vao.bind();
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, polygonResources.indirectBuf.getID());

        glMultiDrawArraysIndirect(GL_TRIANGLE_FAN, nullptr, static_cast<GLsizei>(polygonCount), 0);
    }

    void ShapeRenderer::renderAABBShapes(std::span<const FloatAABB> instances, const glm::vec3& color, const glm::mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render AABB shapes");

        const size_t count = instances.size();
        if (count == 0) return;

        // Reserve space.
        ensureAABBInstanceVboCapacity(count);

        // Move data to gpu.
        aabbResources.instanceVbo.write(instances.data(), count * sizeof(FloatAABB));

        // Bind things, set uniforms.
        aabbResources.shader.use();
        aabbResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);
        aabbResources.shader.setVec3("color", color.x, color.y, color.z);

        aabbResources.vao.bind();

        // Draw.
        glDrawArraysInstanced(GL_LINE_LOOP, 0, 4, count);
    }

    void ShapeRenderer::ensureCircleInstanceVboCapacity(size_t count)
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

    void ShapeRenderer::ensureBoxInstanceVboCapacity(size_t count)
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

    void ShapeRenderer::ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount)
    {
        constexpr size_t SIZEOF_VERTEX = sizeof(glm::vec2);
        constexpr size_t SIZEOF_INSTANCE = sizeof(PolygonInstanceData);
        constexpr size_t SIZEOF_CMD = sizeof(Ecstasy::OpenGL::DrawArraysIndirectCommand);

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

    void ShapeRenderer::ensureAABBInstanceVboCapacity(size_t count)
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

    void ShapeRenderer::ensureLineBufferCapacity(size_t vertexCount)
    {
        constexpr size_t SIZEOF_VERTEX = sizeof(LineVertex);

        auto& vao = lineResources.vao;
        auto& vbo = lineResources.vbo;

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

    void ShapeRenderer::renderLines(std::span<const LineVertex> vertices, const glm::mat4& viewProjectionMatrix)
    {
        TRACY_SCOPE_N("Render lines");

        const size_t count = vertices.size();
        if (count == 0) return;

        // Reserve space.
        ensureLineBufferCapacity(count);

        // Move data to gpu.
        lineResources.vbo.write(vertices.data(), count * sizeof(LineVertex));

        // Bind things, set uniforms.
        lineResources.shader.use();
        lineResources.shader.setMat4("viewProjectionMatrix", viewProjectionMatrix);

        lineResources.vao.bind();

        // Draw.
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(count));
    }
}