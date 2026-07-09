#pragma once
#include "NarrowPhaseCollisionDetector.h"
#include "Material.h"

#include "SolvingPlanners/LLSolvingPlanner.h"

#include <atomic>
#include <vector>
#include <span>
#include <iostream>

namespace PS_AGONY
{
	class Solver
	{
		struct PositionAnchor
		{
			Vec2 localAnchorA;
			Vec2 localAnchorB;
		};

		struct ContactPointMass
		{
			Vec2 rAPerp, rBPerp;
			Real normalMass;
			Real tangentMass;
		};

		struct ContactMassData
		{
			std::array<ContactPointMass, 2> points;
		};

		struct SimulationSettings
		{
			// Baumgarte stabilization.
			const Real positionCorrectionPercent = 0.5;
			const Real positionCorrectionSlop = 0.001;
		};

		struct WorkerResources
		{
			struct alignas(64) WorkerData
			{
				std::atomic<bool> isDestroyed{ false };

				WorkerData() = default;
				~WorkerData() = default;

				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					isDestroyed.store(true, std::memory_order_release);
				}

				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						isDestroyed.store(true, std::memory_order_release);
					}
					return *this;
				}
			};

			static constexpr uint32_t STOP_WAVE = -1;

			std::vector<WorkerData> workerData;
			alignas(64) std::atomic<uint32_t> workNotDone{ 0 };
			alignas(64) std::atomic<uint32_t> currentWaveTicket{ 0 };
		};

		BodySoA* bodies;
		const std::vector<Material>* materials;

		WorkerResources workerResources;
		LLSolvingPlanner solvingPlanner;

		std::vector<BodyPair> collidingBodyPairs;
		std::vector<BodyCollisionData> indirectCollisionData;

		std::vector<PositionAnchor> positionAnchors;
		std::vector<ContactMassData> contactMasses;


		SimulationSettings simulationSettings;
	public:
		Solver() = default;
		~Solver() = default;
		Solver(const Solver&) = delete;
		Solver& operator=(const Solver&) = delete;
		Solver(Solver&&) = delete;
		Solver& operator=(Solver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const std::vector<Material>& materials
		);

		size_t getMemoryUsage() const;

		void solve(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);

		void solveThreaded(
			const std::vector<BodyCollisionData>& narrowPhaseCollisions,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	private:
		size_t planWorkerCount(size_t collisionCount);

		void computeConstantData(const std::vector<BodyCollisionData>& collisionDataContainer);

		void solveVelocityConstraints(
			std::span<const BodyCollisionData> narrowPhaseCollisions
		);

		void solvePositionConstraints(
			std::span<const BodyCollisionData> narrowPhaseCollisions,
			std::span<const PositionAnchor> positionAnchors
		);

		void solveConstraintsThreaded(
			const std::vector<BodyCollisionData>& collisionDataContainer,
			uint32_t velocityIterations,
			uint32_t positionIterations
		);
	};
}
