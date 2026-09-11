#pragma once
#include "Ecstasy/OpenGL/Shader.h"
#include "Ecstasy/OpenGL/ImmutableBuffer.h"
#include "Ecstasy/OpenGL/VertexArray.h"
#include "Ecstasy/OpenGL/DrawCommands.h"

#include <glm/glm.hpp>

#include <vector>
#include <span>
#include <cstdint>

namespace Render
{
	using namespace Ecstasy::OpenGL;

	class ShapeRenderer
	{
	public:
		struct CircleInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float radius;
			uint32_t color;
		};

		struct BoxInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float halfWidth, halfHeight;
			uint32_t color;
		};

		struct PolygonInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			uint32_t color;
			uint32_t textureId;
		};

		struct LineVertex
		{
			float x, y;
			uint32_t color;
		};

		struct FloatAABB
		{
			float minX, minY;
			float maxX, maxY;
		};

		struct CircleRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
		};

		struct BoxRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
		};

		struct PolygonRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vertexVbo;
			ImmutableBuffer instanceVbo;
			ImmutableBuffer indirectBuf;
			Shader shader;
		};

		// Shared by any 2-point-per-segment line draw: springs, joints, rods, etc.
		struct LineRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			Shader shader;
		};

		struct AABBResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
		};
	public:
		// Render resources (GPU-side only; CPU-side instance data is owned by the caller).
		CircleRenderResources circleResources;
		BoxRenderResources boxResources;
		PolygonRenderResources polygonResources;
		LineRenderResources lineResources;
		AABBResources aabbResources;
	public:
		void init();
		void initShaders();
		void initBuffers();

		void renderCircleShapes(std::span<const CircleInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderBoxShapes(std::span<const BoxInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderPolygonShapes(
			std::span<const glm::vec2> vertices,
			std::span<const PolygonInstanceData> instances,
			std::span<const DrawArraysIndirectCommand> drawCommands,
			const glm::mat4& viewProjectionMatrix
		);
		void renderAABBShapes(std::span<const FloatAABB> instances, const glm::vec3& color, const glm::mat4& viewProjectionMatrix);
		void renderLines(std::span<const LineVertex> vertices, const glm::mat4& viewProjectionMatrix);

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount);
		void ensureAABBInstanceVboCapacity(size_t count);
		void ensureLineBufferCapacity(size_t vertexCount);
	};
}