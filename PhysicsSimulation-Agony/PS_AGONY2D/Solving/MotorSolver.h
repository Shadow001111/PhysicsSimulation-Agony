#pragma once
#include "ConstraintSolverBase.h"

#include "PS_AGONY2D/SoA/Constraints/MotorSoA.h"

namespace PS_AGONY2D
{
	struct MotorConstraintData
	{
		ObjectDenseIndex bodyIndex;
		size_t soaIndex;       // Original SoA index, for impulse write-back.
		Real invInertia;       // 0 = skip.
		Real inertia;          // Effective mass = 1 / invInertia, cached to avoid a divide.
		Real targetSpeed;
		Real maxImpulse;       // maxTorque * dt.
		Real impulse;          // Accumulated impulse (local copy).
	};

	class MotorSolver final : public SingleConstraintSolverBase<MotorSolver, MotorSoAViewer, MotorConstraintData>
	{
		using Base = SingleConstraintSolverBase<MotorSolver, MotorSoAViewer, MotorConstraintData>;
	public:
		// Very cheap constraint; re-profile.
		MotorSolver() : Base(ConstraintSolverTuning{ 20'000, 3 }) {}

		void computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset);

		// Applies the accumulated impulse to the body. Runs through the solving plan before the first iteration.
		void warmStart(SolvingPlan::Pass pass);

		void solveVelocityConstraints(SolvingPlan::Pass pass);
	};
}