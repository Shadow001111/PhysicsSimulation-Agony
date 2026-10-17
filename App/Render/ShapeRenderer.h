#pragma once
#include "Ecstasy/OpenGL/Shader.h"
#include "Ecstasy/OpenGL/ImmutableBuffer.h"
#include "Ecstasy/OpenGL/VertexArray.h"
#include "Ecstasy/OpenGL/DrawCommands.h"

#include <glm/glm.hpp>

#include <vector>
#include <span>
#include <cstdint>

namespace PS_AGONY_RENDER
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
			uint32_t id;
		};

		// Position-only circle marker: no rotation, no local COM offset.
		// Used for debug/marker points (body positions, COM dots, contact
		// points) where the extra 12 bytes and per-vertex rotate2D of
		// CircleInstanceData buy nothing since the shape is a circle.
		struct SimplePointInstanceData
		{
			float positionX, positionY;
			float radius;
			uint32_t color;
			uint32_t id;
		};

		struct BoxInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float halfWidth, halfHeight;
			uint32_t color;
			uint32_t id;
		};

		struct PolygonInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			uint32_t color;
			uint32_t id;
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

		struct SimplePointCircleRenderResources
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

		// Capsule reuses BoxInstanceData (halfWidth = halfLength + radius, halfHeight = radius)
		// and box.vert; capsule.frag carves the rounded ends via discard.
		struct CapsuleRenderResources
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

		struct OutlineRenderResources
		{
			VertexArray vao; // Empty.
			Shader shader;
		};
	public:
		// Render resources (GPU-side only; CPU-side instance data is owned by the caller).
		CircleRenderResources circleResources;
		SimplePointCircleRenderResources simplePointCircleResources;
		BoxRenderResources boxResources;
		CapsuleRenderResources capsuleResources;
		PolygonRenderResources polygonResources;
		LineRenderResources lineResources;
		AABBResources aabbResources;
		OutlineRenderResources outlineResources;
	public:
		void init();
		void initShaders();
		void initBuffers();

		void renderCircleShapes(std::span<const CircleInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderSimplePointCircles(std::span<const SimplePointInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderBoxShapes(std::span<const BoxInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderCapsuleShapes(std::span<const BoxInstanceData> instances, const glm::mat4& viewProjectionMatrix);
		void renderPolygonShapes(
			std::span<const glm::vec2> vertices,
			std::span<const PolygonInstanceData> instances,
			std::span<const DrawArraysIndirectCommand> drawCommands,
			const glm::mat4& viewProjectionMatrix
		);
		void renderAABBShapes(std::span<const FloatAABB> instances, const glm::vec3& color, const glm::mat4& viewProjectionMatrix);
		void renderLines(std::span<const LineVertex> vertices, const glm::mat4& viewProjectionMatrix);

		void renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor);

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureSimplePointInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensureCapsuleInstanceVboCapacity(size_t count);
		void ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount);
		void ensureAABBInstanceVboCapacity(size_t count);
		void ensureLineBufferCapacity(size_t vertexCount);
	private:
		template<class Resources>
		static void ensureBoxLikeInstanceVboCapacity(Resources& resources, size_t count);
	};
}