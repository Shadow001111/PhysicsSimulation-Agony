#pragma once
#include "Types.h"

#include <vector>

namespace PS_AGONY
{
	using BodyType = uint8_t;

	struct BodiesSoA
	{
		std::vector<Real> positionX;
		std::vector<Real> positionY;

		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;
	};

	struct CirclesSoA
	{
		std::vector<Real> radius;
	};

	struct SimulationSettings
	{
		Real updateInterval = 1.0;
	};

	class Simulation
	{
		// Bodies SoA
		BodiesSoA bodies;
		CirclesSoA circles;

		// Settings
		SimulationSettings simulationSettings;
	public:
		Simulation() = default;
		~Simulation() = default;
		Simulation(const Simulation&) = delete;
		Simulation& operator=(const Simulation&) = delete;
		Simulation(Simulation&&) = delete;
		Simulation& operator=(Simulation&&) = delete;

		void update(Real deltaTime);
	private:
	};
}
