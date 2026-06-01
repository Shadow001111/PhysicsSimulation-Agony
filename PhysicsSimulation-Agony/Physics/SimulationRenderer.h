#pragma once
#include "Camera2D.h"

#include "OpenGLWrappers/Shader.h"
#include "OpenGLWrappers/ImmutableBuffer.h"
#include "OpenGLWrappers/VertexArray.h"

#include "BodySoAViewer.h"

namespace PS_AGONY
{
	class Simulation;

	class SimulationRenderer
	{
		struct CircleInstanceData
		{
			float x, y, rotation, radius;
			int color; // 3 bytes used.
		};

		struct CircleRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<CircleInstanceData> instanceData;
		};

		struct AABBLineResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<AABB> instanceData;
		};

		// Resources.
		CircleRenderResources circleResources;
		AABBLineResources aabbResources;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;
		CircleSoAViewer circles;
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

		void renderBodyAABBs(const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix);

		// Render shapes.

		void renderCircleShapes(const Mat4& viewProjectionMatrix);
		void renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix);

		// Buffer helpers.

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureAABBInstanceVboCapacity(size_t count);
	};
}

