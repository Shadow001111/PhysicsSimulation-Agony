#pragma once
#include "ConstraintSolverBase.h"

#include "Physics/SoA/Constraints/SpringSoA.h"

namespace PS_AGONY
{
	struct SpringConstraintData
	{
		ObjectIndex bodyIndexA, bodyIndexB;
		Vec2 dir;
		Vec2 rotatedAnchorA, rotatedAnchorB;
		Real invEffectiveMass; // 1 / (effectiveMass + gamma).
		Real bias; // beta * C.
	};

	class SpringSolver final : public ConstraintSolverBase<SpringSolver, SpringSoAViewer, SpringConstraintData>
	{
		using Base = ConstraintSolverBase<SpringSolver, SpringSoAViewer, SpringConstraintData>;
		friend Base;
	public:
		// Tuned for collisions; re-profile once springs are a bottleneck on their own.
		SpringSolver() : Base(ConstraintSolverTuning{ 7'000, 3 }) {}
	private:
		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}
