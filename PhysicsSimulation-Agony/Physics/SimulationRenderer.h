#pragma once
#include "Camera2D.h"

#include "OpenGLWrappers/Shader.h"
#include "OpenGLWrappers/ImmutableBuffer.h"
#include "OpenGLWrappers/VertexArray.h"
#include "OpenGLWrappers/Texture.h"

#include "BodySoAViewer.h"

namespace PS_AGONY
{
	class Simulation;

	class SimulationRenderer
	{
		struct CircleInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float radius;
			uint32_t color; // 3 bytes used.
		};

		struct BoxInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float halfWidth, halfHeight;
			uint32_t color; // 3 bytes used.
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

		struct FloatAABB
		{
			float minX, minY;
			float maxX, maxY;
		};

		struct AABBResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<AABB> aabbs;
			std::vector<FloatAABB> instanceData;
		};

		struct DrawArraysIndirectCommand
		{
			uint32_t count;        
			uint32_t instanceCount;
			uint32_t first;        
			uint32_t baseInstance; 
		};

		struct PolygonInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			uint32_t color;
			uint32_t textureId;
		};

		struct PolygonRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vertexVbo;   // Packed local vertex positions for all polygons.
			ImmutableBuffer instanceVbo; // Per-polygon transform data.
			ImmutableBuffer indirectBuf; // DrawArraysIndirectCommand array.
			Shader shader;
			std::vector<glm::vec2>               vertexData;
			std::vector<PolygonInstanceData>     instanceData;
			std::vector<DrawArraysIndirectCommand> drawCommands;
		};

		// Resources.
		CircleRenderResources circleResources;
		BoxRenderResources boxResources;
		PolygonRenderResources polygonResources;

		AABBResources aabbResources;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;

		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		PolygonSoAViewer polygons;
	public:
		SimulationRenderer() = default;
		~SimulationRenderer() = default;
		SimulationRenderer(const SimulationRenderer&) = delete;
		SimulationRenderer& operator=(const SimulationRenderer&) = delete;
		SimulationRenderer(SimulationRenderer&&) = delete;
		SimulationRenderer& operator=(SimulationRenderer&&) = delete;

		void init();
		void render(const Simulation& simulation);

		Camera2D& getCamera() noexcept { return camera; }
	private:
		void initShaders();
		void initBuffers();

		// Collect data and render.

		void renderBodies(const Mat4& viewProjectionMatrix);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix);
		void renderBodyPositions(const Mat4& viewProjectionMatrix);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix);
		
		void renderCircleBodies(const Mat4& viewProjectionMatrix);
		void renderBoxBodies(const Mat4& viewProjectionMatrix);
		void renderPolygonBodies(const Mat4& viewProjectionMatrix);

		void renderBodyAABBs(const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix);

		// Render shapes.

		void renderCircleShapes(const Mat4& viewProjectionMatrix);
		void renderBoxShapes(const Mat4& viewProjectionMatrix);
		void renderPolygonShapes(const Mat4& viewProjectionMatrix);
		
		void renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix);

		// Buffer helpers.

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount);

		void ensureAABBInstanceVboCapacity(size_t count);
	};
}

