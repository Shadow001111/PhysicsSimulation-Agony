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
			uint32_t textureId;
		};

		struct BoxInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float halfWidth, halfHeight;
			uint32_t color; // 3 bytes used.
			uint32_t textureId;
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

		struct AABBResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<AABB> instanceData;
		};

		// Resources.
		CircleRenderResources circleResources;
		BoxRenderResources boxResources;
		AABBResources aabbResources;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;
		CircleSoAViewer circles;
		BoxSoAViewer boxes;

		// Delete.
		Texture hardcodedTexture;
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
		void renderCircleBodies(const Mat4& viewProjectionMatrix);
		void renderBoxBodies(const Mat4& viewProjectionMatrix);

		void renderBodyAABBs(const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix);

		// Render shapes.

		void renderCircleShapes(const Mat4& viewProjectionMatrix);
		void renderBoxShapes(const Mat4& viewProjectionMatrix);
		void renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix);

		// Buffer helpers.

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensureAABBInstanceVboCapacity(size_t count);
	};
}

