#pragma once
#include "SolverBase.h"
#include "../NarrowPhaseCollisionDetector.h"

#include <span>
#include <vector>

namespace PS_AGONY
{
	class SpringSolver : public SolverBase
	{
		// Structures / classes.

		struct SpringConstraintData
		{
			ObjectIndex bodyIndexA, bodyIndexB;
			Vec2 dir;
			Vec2 rotatedAnchorA, rotatedAnchorB;
			Real invEffectiveMass; // 1 / (effectiveMass + gamma).
			Real bias; // beta * C.
		};

		// Member fields.

		SpringSoAViewer springs;

		std::vector<ObjectPair> springBodyPairs;

		std::vector<size_t> orderedSpringIndices;

		std::vector<SpringConstraintData> springConstraintContainer;
	public:
		SpringSolver() = default;
		~SpringSolver() override = default;
		SpringSolver(const SpringSolver&) = delete;
		SpringSolver& operator=(const SpringSolver&) = delete;
		SpringSolver(SpringSolver&&) = delete;
		SpringSolver& operator=(SpringSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const SpringSoAViewer& springs,
			SolvingPlanner& solvingPlanner
		);

		size_t getMemoryUsage() const override;

		void solveSprings(
			Real deltaTime,
			uint32_t springIterations,
			bool springsWereChanged
		);
	private:
		// Spring constraint solving.

		size_t planWorkerCount(size_t springCount) const;

		void computeConstraintData(Real deltaTime, std::span<const size_t> springIndices);

		void solveVelocityConstraints(
			std::span<const SpringConstraintData> constraintDataContainer
		);

		void solveConstraintsThreaded(uint32_t springIterations);
	};
}