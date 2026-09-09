#pragma once
#include "SolverBase.h"
#include "../NarrowPhaseCollisionDetector.h"
#include "../SoA/Constraints/RodSoA.h"

#include <span>
#include <vector>

namespace PS_AGONY
{
	class RodSolver : public SolverBase
	{
		// Structures / classes.

		struct RodConstraintData
		{
			ObjectIndex bodyIndexA, bodyIndexB;
			Vec2 dir;
			Vec2 rotatedAnchorA, rotatedAnchorB;
			Real invEffectiveMass; // 1 / effectiveMass (no compliance - rod is rigid).
			Real bias; // Baumgarte term: (baumgarteFactor / dt) * C.
		};

		// Member fields.

		RodSoAViewer rods;

		std::vector<ObjectPair> rodBodyPairs;

		std::vector<size_t> orderedRodIndices;

		std::vector<RodConstraintData> rodConstraintContainer;
	public:
		RodSolver() = default;
		~RodSolver() override = default;
		RodSolver(const RodSolver&) = delete;
		RodSolver& operator=(const RodSolver&) = delete;
		RodSolver(RodSolver&&) = delete;
		RodSolver& operator=(RodSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const RodSoAViewer& rods,
			SolvingPlanner& solvingPlanner
		);

		size_t getMemoryUsage() const override;

		void solveRods(
			Real deltaTime,
			uint32_t rodIterations,
			bool rodsWereChanged
		);
	private:
		// Rod constraint solving.

		size_t planWorkerCount(size_t rodCount) const;

		void computeConstraintData(Real deltaTime, std::span<const size_t> rodIndices);

		void solveVelocityConstraints(
			std::span<const RodConstraintData> constraintDataContainer
		);

		void solveConstraintsThreaded(uint32_t rodIterations);
	};
}