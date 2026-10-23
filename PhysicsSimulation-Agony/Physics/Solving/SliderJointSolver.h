#pragma once
#include "ConstraintSolverBase.h"

#include "Physics/SoA/Constraints/SliderJointSoA.h"

namespace PS_AGONY2D
{
	struct SliderJointConstraintData
	{
		ObjectIndex bodyIndexA, bodyIndexB;

		Vec2 axis; // World-space axis (rotates with body A); translation is locked perpendicular to this.
		Vec2 perp; // World-space perpendicular to axis.

		Real s1, s2; // Moment arms for the perpendicular constraint.
		Real a1, a2; // Moment arms for the axis (limit) constraint.

		// Soft perpendicular constraint.
		Real invK = 0; // Soft inverse effective mass; 0 if the row is inactive this step.
		Real bias = 0;

		// Axis limit (hard, one-sided).
		Real axisInvEffectiveMass = 0; // 0 if the limit is inactive this step.
		Real axisBias = 0;
		Real limitDirection = 0;       // +1 lower limit active, -1 upper limit active, 0 inactive.
		Real accumulatedAxisImpulse = 0;
	};

	// Solves slider joints: B's anchor is kept on a line through A's anchor. The line's
	// axis rotates with body A. Relative rotation is NOT constrained, plus an optional hard
	// travel limit along the axis. The perpendicular translation is a soft scalar row
	// (stiffness/damping), same soft-constraint technique as SpringSolver.
	// The axis limit is a separate hard, one-sided, impulse-clamped stop.
	class SliderJointSolver final : public PairConstraintSolverBase<SliderJointSolver, SliderJointSoAViewer, SliderJointConstraintData>
	{
		using Base = PairConstraintSolverBase<SliderJointSolver, SliderJointSoAViewer, SliderJointConstraintData>;
	public:
		// Tuned for collisions; re-profile once slider joints are a bottleneck on their own.
		SliderJointSolver() : Base(ConstraintSolverTuning{ 7'000, 3 }) {}

		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}