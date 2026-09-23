#pragma once
#include "SolverBase.h"

#include "Physics/SoA/BodySoA.h"
#include "Physics/SoA/Constraints/SliderJointSoA.h"

#include <span>
#include <vector>

namespace PS_AGONY
{
	// Solves slider joints: B's anchor is kept on a line through A's anchor. The line's
	// axis rotates with body A. Relative rotation is NOT constrained, plus an optional hard
	// travel limit along the axis. The perpendicular translation is a soft scalar row
	// (stiffness/damping), same soft-constraint technique as SpringSolver.
	// The axis limit is a separate hard, one-sided, impulse-clamped stop.
	class SliderJointSolver : public SolverBase
	{
		// Structures / classes.

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

		// Member fields.

		SliderJointSoAViewer sliderJoints;

		std::vector<ObjectPair> sliderJointBodyPairs;

		std::vector<size_t> orderedSliderJointIndices;

		std::vector<SliderJointConstraintData> sliderJointConstraintContainer;
	public:
		SliderJointSolver() = default;
		~SliderJointSolver() override = default;
		SliderJointSolver(const SliderJointSolver&) = delete;
		SliderJointSolver& operator=(const SliderJointSolver&) = delete;
		SliderJointSolver(SliderJointSolver&&) = delete;
		SliderJointSolver& operator=(SliderJointSolver&&) = delete;

		void setDataViewers(
			BodySoA& bodies,
			const SliderJointSoAViewer& sliderJoints
		);

		size_t getMemoryUsage() const override;

		void solveSliderJoints(
			Real deltaTime,
			uint32_t iterations,
			bool sliderJointsWereChanged
		);
	private:
		// Slider joint constraint solving.

		size_t planWorkerCount(size_t jointCount) const;

		void computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices);

		void solveVelocityConstraints(SolvingPlan::Pass pass);

		void solveConstraintsThreaded(uint32_t iterations);
	};
}
