#include "MotorSolver.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
	void MotorSolver::computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices, size_t outOffset)
	{
		TRACY_SCOPE_NC("Compute motor constraint data", PS_AGONY2D::Profiling::Color::Chocolate);

		// Body data is read-only here; the warm start is applied separately in warmStart().
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		const ObjectIndex* ECSTASY_RESTRICT bodyIndexPtr = viewer.bodyIndex;
		const Real* ECSTASY_RESTRICT targetSpeedPtr = viewer.targetSpeed;
		const Real* ECSTASY_RESTRICT maxTorquePtr = viewer.maxTorque;
		Real* ECSTASY_RESTRICT accumulatedImpulsePtr = const_cast<Real*>(viewer.accumulatedImpulse);

		const size_t count = constraintIndices.size();
		const size_t* ECSTASY_RESTRICT indicesPtr = constraintIndices.data();

		for (size_t n = 0; n < count; n++)
		{
			const size_t i = indicesPtr[n];
			MotorConstraintData& data = constraintContainer[outOffset + n];

			const ObjectIndex bodyIndex = bodyIndexPtr[i];
			const Real invInertia = invInertiaPtr[bodyIndex];

			data.bodyIndex = bodyIndex;
			data.soaIndex = i;

			if (isStaticPtr[bodyIndex] || invInertia <= Real(0)) [[unlikely]]
			{
				data.invInertia = Real(0);
				accumulatedImpulsePtr[i] = Real(0);
				continue;
			}

			const Real maxImpulse = maxTorquePtr[i] * deltaTime;

			// Clamp to the (possibly changed) limit.
			const Real impulse = glm::clamp(accumulatedImpulsePtr[i], -maxImpulse, maxImpulse);

			data.invInertia = invInertia;
			data.inertia = Real(1) / invInertia;
			data.targetSpeed = targetSpeedPtr[i];
			data.maxImpulse = maxImpulse;
			data.impulse = impulse;
			accumulatedImpulsePtr[i] = impulse;
		}
	}

	void MotorSolver::warmStart(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Warm start motors", PS_AGONY2D::Profiling::Color::Violet);

		const MotorConstraintData* ECSTASY_RESTRICT dataPtr = constraintContainer.data() + pass.start;
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();

		for (size_t n = 0; n < pass.size; n++)
		{
			const MotorConstraintData& data = dataPtr[n];
			if (data.invInertia == Real(0)) continue;
			angularVelocityPtr[data.bodyIndex] += data.impulse * data.invInertia;
		}
	}

	void MotorSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve motor velocity constraints", PS_AGONY2D::Profiling::Color::Violet);

		MotorConstraintData* ECSTASY_RESTRICT dataPtr = constraintContainer.data() + pass.start;
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		Real* ECSTASY_RESTRICT accumulatedImpulsePtr = const_cast<Real*>(viewer.accumulatedImpulse);

		const size_t count = pass.size;

		for (size_t n = 0; n < count; n++)
		{
			MotorConstraintData& data = dataPtr[n];
			const Real invInertia = data.invInertia;
			if (invInertia == Real(0)) continue;

			const Real w = angularVelocityPtr[data.bodyIndex];

			// Effective mass is the cached inertia.
			const Real lambda = (data.targetSpeed - w) * data.inertia;

			// Clamp the accumulated impulse, apply only the delta.
			const Real oldImpulse = data.impulse;
			const Real newImpulse = glm::clamp(oldImpulse + lambda, -data.maxImpulse, data.maxImpulse);
			data.impulse = newImpulse;
			accumulatedImpulsePtr[data.soaIndex] = newImpulse;

			angularVelocityPtr[data.bodyIndex] = w + (newImpulse - oldImpulse) * invInertia;
		}
	}
}