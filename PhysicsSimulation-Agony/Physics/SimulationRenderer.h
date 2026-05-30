#pragma once
#include "Camera2D.h"

#include "OpenGLWrappers/Shader.h"
#include "OpenGLWrappers/ImmutableBuffer.h"
#include "OpenGLWrappers/VertexArray.h"

namespace PS_AGONY
{
	class Simulation;

	struct CircleRenderData
	{
		float x, y, radius;
		uint32_t collisionDebug;
	};

	struct CircleRenderResources
	{
		VertexArray vao;
		ImmutableBuffer vbo;
		ImmutableBuffer instanceVbo;
		Shader shader;
		std::vector<CircleRenderData> renderData;
	};

	class SimulationRenderer
	{
		// Resources.
		CircleRenderResources circleResources;

		// Camera.
		Camera2D camera;
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

		void renderCircles(const Simulation& simulation, const Mat4& viewProjectionMatrix);

		void ensureCircleInstanceVboCapacity(size_t count);
	};
}

