#pragma once
#include "GlmTypes.h"

#include <vector>

namespace PS_AGONY
{
	enum class BodyType : uint8_t
	{
		Circle,
		Box,
		Polygon
	};

	struct BodiesSoA
	{
		std::vector<Real> positionX;
		std::vector<Real> positionY;
		std::vector<Real> velocityX;
		std::vector<Real> velocityY;
		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;

		size_t getCount() const noexcept { return positionX.size(); }
	};

	struct CirclesSoA
	{
		std::vector<Real> radius;

		std::vector<BodyIndex> bodyIndices;

		size_t getCount() const noexcept { return radius.size(); }
	};

	struct SimulationSettings
	{
		Real updateInterval = 1 / 60.0;
		
		Vec2 gravity{ 0.0, -9.81 };
	};

	class Simulation
	{
		// Bodies SoA.
		BodiesSoA bodies;
		CirclesSoA circles;

		// Settings.
		SimulationSettings simulationSettings;

		// Other.
		Real updateTimeAccumulator = 0.0;
	public:
		Simulation() = default;
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);

		BodyIndex createCircle(Vec2 position, Vec2 velocity, Real radius);

		const auto& getBodies() const noexcept { return bodies; }
		const auto& getCircles() const noexcept { return circles; }
	private:
		void physicsStep(Real deltaTime);
	};
}
