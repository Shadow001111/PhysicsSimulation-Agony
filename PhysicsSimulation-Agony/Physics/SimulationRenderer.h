#pragma once
#include "OpenGLWrappers/Shader.h"
#include "OpenGLWrappers/ImmutableBuffer.h"
#include "OpenGLWrappers/VertexArray.h"

namespace PS_AGONY
{
	class Simulation;

	class SimulationRenderer
	{
		VertexArray vaoCircle;
		ImmutableBuffer vboCircle;
		Shader shaderCircle;
	public:
		SimulationRenderer() = default;
		~SimulationRenderer() = default;
		SimulationRenderer(const SimulationRenderer&) = delete;
		SimulationRenderer& operator=(const SimulationRenderer&) = delete;
		SimulationRenderer(SimulationRenderer&&) = delete;
		SimulationRenderer& operator=(SimulationRenderer&&) = delete;

		void init();
		void render(const Simulation& simulation);
	private:
		void initShaders();
		void initBuffers();
	};
}

