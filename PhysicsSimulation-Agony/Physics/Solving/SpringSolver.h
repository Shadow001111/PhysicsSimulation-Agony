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

		// Everything about a spring that stays constant across all springIterations of one
		// solve() call, since only velocities (not positions/rotations) change between them.
		// Precomputing this once - instead of re-deriving anchors/effective mass/soft-constraint
		// terms every iteration like the original code did - mirrors how BodyCollisionsSolver
		// splits constant data from the per-iteration velocity solve.
		struct SpringConstraintData
		{
			Vec2 dir;
			Vec2 rotatedAnchorA, rotatedAnchorB;
			Real invEffectiveMass; // 1 / (effectiveMass + gamma). Zero means "skip this spring".
			Real bias;             // beta * C, folded in so the per-iteration step is one FMA.
		};

		// Memeber fields.

		SpringSoAViewer springs;

		std::vector<BodyPair> springBodyPairs;

		// Position in this array -> original spring index. Identity for the single-threaded
		// path (springs solved in their natural order); reordered via SolvingPlanner's graph
		// coloring for the multi-threaded path. Kept as plain indices - rather than a full
		// copy of the spring payload like BodyCollisionsSolver's orderedCollisionData - since
		// the spring data already lives contiguously in the SoA and doesn't need duplicating.
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
			uint32_t springIterations
		);
	private:
		// Spring constraint solving.

		size_t planWorkerCount(size_t springCount) const;

		void computeConstantData(Real deltaTime, std::span<const size_t> springIndices);

		void solveVelocityConstraints(
			std::span<const size_t> springIndices,
			std::span<const SpringConstraintData> constraintDataContainer
		);

		void solveConstraintsThreaded(uint32_t springIterations);
	};
}