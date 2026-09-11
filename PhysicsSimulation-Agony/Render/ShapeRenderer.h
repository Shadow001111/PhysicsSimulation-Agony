#pragma once
#include "Ecstasy/OpenGL/Shader.h"
#include "Ecstasy/OpenGL/ImmutableBuffer.h"
#include "Ecstasy/OpenGL/VertexArray.h"
#include "Ecstasy/OpenGL/DrawCommands.h"

#include <glm/glm.hpp>

#include <vector>
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
			std::vector<CircleInstanceData> instanceData;
		};

		struct BoxRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<BoxInstanceData> instanceData;
		};

		struct PolygonRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vertexVbo;
			ImmutableBuffer instanceVbo;
			ImmutableBuffer indirectBuf;
			Shader shader;
			std::vector<glm::vec2> vertexData;
			std::vector<PolygonInstanceData> instanceData;
			std::vector<DrawArraysIndirectCommand> drawCommands;
		};

		struct SpringRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			Shader shader;
			std::vector<LineVertex> vertexData;
		};

		struct AABBResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<FloatAABB> instanceData;
		};
	public:
		// Render resources.
		CircleRenderResources circleResources;
		BoxRenderResources boxResources;
		PolygonRenderResources polygonResources;
		SpringRenderResources springResources;
		AABBResources aabbResources;
	public:
		void init();
		void initShaders();
		void initBuffers();

		void renderCircleShapes(const glm::mat4& viewProjectionMatrix);
		void renderBoxShapes(const glm::mat4& viewProjectionMatrix);
		void renderPolygonShapes(const glm::mat4& viewProjectionMatrix);
		void renderAABBShapes(const glm::vec3& color, const glm::mat4& viewProjectionMatrix);
	
		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount);
		void ensureAABBInstanceVboCapacity(size_t count);
		void ensureSpringBufferCapacity(size_t vertexCount);
	};
}

