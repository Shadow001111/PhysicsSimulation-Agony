#pragma once
#include "SolverBase.h"
#include "../NarrowPhaseCollisionDetector.h"
#include "../SoA/Constraints/JointSoA.h"

#include <span>
#include <vector>

namespace PS_AGONY
{
	// Soft revolute joint: pins two body-local anchor points together (both x and y),
	// leaving rotation free. Same soft-constraint (gamma/beta) derivation as SpringSolver,
	// just with a 2x2 effective mass matrix instead of a scalar since the constraint has
	// two degrees of freedom instead of one.
	class JointSolver : public SolverBase
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

		// Memeber fields.

		JointSoAViewer joints;

		std::vector<ObjectPair> jointBodyPairs;

		std::vector<size_t> orderedJointIndices;

		std::vector<JointConstraintData> jointConstraintContainer;
	public:
		JointSolver() = default;
		~JointSolver() override = default;
		JointSolver(const JointSolver&) = delete;
		JointSolver& operator=(const JointSolver&) = delete;
		JointSolver(JointSolver&&) = delete;
		JointSolver& operator=(JointSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const JointSoAViewer& joints,
			SolvingPlanner& solvingPlanner
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

		void solveVelocityConstraints(
			std::span<const JointConstraintData> constraintDataContainer
		);

		void solveConstraintsThreaded(uint32_t jointIterations);
	};
}