#pragma once
#include "SolverBase.h"

#include "Physics/SoA/BodySoA.h"
#include "Physics/SoA/Constraints/RevoluteJointSoA.h"

#include <span>
#include <vector>

namespace PS_AGONY
{
	// Soft revolute joint: pins two body-local anchor points together (both x and y),
	// leaving rotation free. Same soft-constraint (gamma/beta) derivation as SpringSolver,
	// just with a 2x2 effective mass matrix instead of a scalar since the constraint has
	// two degrees of freedom instead of one.
	class RevoluteJointSolver : public SolverBase
	{
		// Structures / classes.

		struct JointConstraintData
		{
			ObjectIndex bodyIndexA, bodyIndexB;
			Vec2 rotatedAnchorA, rotatedAnchorB;
			// Symmetric inverse effective mass matrix: [invK11 invK12; invK12 invK22].
			// All three left at 0 means "nothing to solve" (mirrors SpringSolver's invEffectiveMass == 0 sentinel).
			Real invK11, invK12, invK22;
			Vec2 bias; // beta * positionError.
		};

		// Member fields.

		JointSoAViewer joints;

		std::vector<ObjectPair> jointBodyPairs;

		std::vector<size_t> orderedJointIndices;

		std::vector<JointConstraintData> jointConstraintContainer;
	public:
		RevoluteJointSolver() = default;
		~RevoluteJointSolver() override = default;
		RevoluteJointSolver(const RevoluteJointSolver&) = delete;
		RevoluteJointSolver& operator=(const RevoluteJointSolver&) = delete;
		RevoluteJointSolver(RevoluteJointSolver&&) = delete;
		RevoluteJointSolver& operator=(RevoluteJointSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const JointSoAViewer& joints
		);

		size_t getMemoryUsage() const override;

		void solveJoints(
			Real deltaTime,
			uint32_t jointIterations,
			bool jointsWereChanged
		);
	private:
		// Joint constraint solving.

		size_t planWorkerCount(size_t jointCount) const;

		void computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices);

		void solveVelocityConstraints(SolvingPlan::Pass pass);

		void solveConstraintsThreaded(uint32_t jointIterations);
	};
}