#pragma once
#include "ConstraintSolverBase.h"

#include "PS_AGONY2D/SoA/Constraints/RevoluteJointSoA.h"

namespace PS_AGONY2D
{
	struct RevoluteJointConstraintData
	{
		ObjectDenseIndex bodyIndexA, bodyIndexB;
		Vec2 rotatedAnchorA, rotatedAnchorB;
		// Symmetric inverse effective mass matrix: [invK11 invK12; invK12 invK22].
		// All three left at 0 means "nothing to solve" (mirrors SpringSolver's invEffectiveMass == 0 sentinel).
		Real invK11, invK12, invK22;
		Vec2 bias; // beta * positionError.
	};

	// Soft revolute joint: pins two body-local anchor points together (both x and y),
	// leaving rotation free. Same soft-constraint (gamma/beta) derivation as SpringSolver,
	// just with a 2x2 effective mass matrix instead of a scalar since the constraint has
	// two degrees of freedom instead of one.
	class RevoluteJointSolver final : public PairConstraintSolverBase<RevoluteJointSolver, RevoluteJointSoAViewer, RevoluteJointConstraintData>
	{
		using Base = PairConstraintSolverBase<RevoluteJointSolver, RevoluteJointSoAViewer, RevoluteJointConstraintData>;
	public:
		// Mirrors SpringSolver's threshold as-is; re-profile once joints are a bottleneck on their own (a joint's constraint data is heavier than a spring's).
		RevoluteJointSolver() : Base(ConstraintSolverTuning{ 7'000, 3 }) {}

		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}