#pragma once
#include "ConstraintSolverBase.h"

#include "Physics/SoA/Constraints/RodSoA.h"

namespace PS_AGONY
{
	struct RodConstraintData
	{
		ObjectIndex bodyIndexA, bodyIndexB;
		Vec2 dir;
		Vec2 rotatedAnchorA, rotatedAnchorB;
		Real invEffectiveMass; // 1 / effectiveMass (no compliance - rod is rigid).
		Real bias; // Baumgarte term: (baumgarteFactor / dt) * C.
	};

	class RodSolver final : public PairConstraintSolverBase<RodSolver, RodSoAViewer, RodConstraintData>
	{
		using Base = PairConstraintSolverBase<RodSolver, RodSoAViewer, RodConstraintData>;
	public:
		// Placeholder value; rod's per-constraint cost/count profile hasn't been measured yet.
		RodSolver() : Base(ConstraintSolverTuning{ 7'000, 3 }) {}

		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}