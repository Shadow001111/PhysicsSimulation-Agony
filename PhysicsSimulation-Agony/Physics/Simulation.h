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

	struct Material
	{
		Real elasticity = 1.0;
		Real staticFriction = 0.0;
		Real dynamicFriction = 0.0;
	};

	struct AABBSoA
	{
		std::vector<Real> minX;
		std::vector<Real> minY;
		std::vector<Real> maxX;
		std::vector<Real> maxY;
	};

	struct BodiesSoA
	{
		std::vector<Real> positionX;
		std::vector<Real> positionY;

		std::vector<Real> velocityX;
		std::vector<Real> velocityY;

		std::vector<Real> rotation;

		std::vector<Real> angularVelocity;

		std::vector<Real> mass;
		std::vector<Real> invMass;

		std::vector<Real> inertia;
		std::vector<Real> invInertia;

		// std::vector<Real> localCenterOfMassX;
		// std::vector<Real> localCenterOfMassY;

		std::vector<MaterialIndex> materialIndex;

		AABBSoA aabb;

		std::vector<BodyType> bodyType;
		std::vector<BodyIndex> shapeIndex;

		std::vector<uint8_t> collisionDebug;

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

	struct BodyPair
	{
		BodyIndex a, b;
	};

	class Simulation
	{
		// Bodies SoA.
		BodiesSoA bodies;
		CirclesSoA circles;

		// Materials.
		std::vector<Material> materials;

		// Settings.
		SimulationSettings simulationSettings;

		// Collisions.
		std::vector<BodyPair> broadPhaseCollisions;

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

		BodyIndex createCircle(Vec2 position, Vec2 velocity, Real radius, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex);

		MaterialIndex createMaterial(const Material& material);

		const auto& getBodies() const noexcept { return bodies; }
		const auto& getCircles() const noexcept { return circles; }
	private:
		void physicsStep(Real deltaTime);

		void applyExternalForces(size_t bodyCount, Real deltaTime);

		void integrate(size_t bodyCount, Real deltaTime);

		void boundaryCollisionResolution(size_t bodyCount);

		void buildCircleAABBs();

		void broadPhaseCollisionDetection(const size_t bodyCount);

		void narrowPhaseCollisionDetection();

		void resolveCollisions();
	private:
		// Broad phase methods

		void justAABB(const size_t bodyCount);
		void sweepAndPrune(size_t bodyCount);
	};
}
