#pragma once
#include "ConstraintSolverBase.h"

#include "PS_AGONY2D/SoA/Constraints/PrismaticJointSoA.h"

namespace PS_AGONY2D
{
	struct PrismaticJointConstraintData
	{
		ObjectDenseIndex bodyIndexA, bodyIndexB;

		Vec2 axis; // World-space axis (rotates with body A); translation is locked perpendicular to this.
		Vec2 perp; // World-space perpendicular to axis.

		Real s1, s2; // Moment arms for the perpendicular / anti-rotation block.
		Real a1, a2; // Moment arms for the axis (limit) constraint.

		// Soft 2x2 effective mass for the (perpendicular translation, relative angle) block.
		Real invK11 = 0, invK12 = 0, invK22 = 0;
		bool blockValid = false;

		Real bias1 = 0; // Soft bias for the perpendicular constraint.
		Real bias2 = 0; // Soft bias for the relative-angle (anti-spin) constraint.

		// Axis limit (hard, one-sided).
		Real axisInvEffectiveMass = 0; // 0 if the limit is inactive this step.
		Real axisBias = 0;
		Real limitDirection = 0;       // +1 lower limit active, -1 upper limit active, 0 inactive.
		Real accumulatedAxisImpulse = 0;
	};

	// Solves prismatic (slider / piston) joints: translation locked to an axis that
	// rotates with body A, relative rotation locked, plus an optional hard travel limit
	// along the axis. The (perpendicular translation, relative angle) pair is solved as
	// a soft 2x2 block (stiffness/damping), same soft-constraint technique as SpringSolver.
	// The axis limit is a separate hard, one-sided, impulse-clamped stop.
	class PrismaticJointSolver final : public PairConstraintSolverBase<PrismaticJointSolver, PrismaticJointSoAViewer, PrismaticJointConstraintData>
	{
		using Base = PairConstraintSolverBase<PrismaticJointSolver, PrismaticJointSoAViewer, PrismaticJointConstraintData>;
	public:
		// Tuned for collisions; re-profile once prismatic joints are a bottleneck on their own.
		PrismaticJointSolver() : Base(ConstraintSolverTuning{ 7'000, 3 }) {}

		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}