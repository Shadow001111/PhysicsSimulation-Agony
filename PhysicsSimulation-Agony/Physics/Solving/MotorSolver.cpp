#include "MotorSolver.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
	void MotorSolver::computeConstraintData(Real deltaTime, std::span<const size_t> constraintIndices)
	{
		TRACY_SCOPE_NC("Compute motor constraint data", Ecstasy::Core::Color::Chocolate);

		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		const ObjectIndex* ECSTASY_RESTRICT bodyIndexPtr = viewer.bodyIndex;
		const Real* ECSTASY_RESTRICT targetSpeedPtr = viewer.targetSpeed;
		const Real* ECSTASY_RESTRICT maxTorquePtr = viewer.maxTorque;
		Real* ECSTASY_RESTRICT accumulatedImpulsePtr = const_cast<Real*>(viewer.accumulatedImpulse);

		const size_t count = constraintIndices.size();
		constraintContainer.resize(count);

		const size_t* ECSTASY_RESTRICT indicesPtr = constraintIndices.data();

		for (size_t n = 0; n < count; n++)
		{
			const size_t i = indicesPtr[n];
			MotorConstraintData& data = constraintContainer[n];

			const ObjectIndex bodyIndex = bodyIndexPtr[i];
			const Real invInertia = invInertiaPtr[bodyIndex];

			data.bodyIndex = bodyIndex;
			data.soaIndex = i;

			if (invInertia <= Real(0)) [[unlikely]]
			{
				data.invInertia = Real(0);
				accumulatedImpulsePtr[i] = Real(0);
				continue;
			}

			const Real maxImpulse = maxTorquePtr[i] * deltaTime;

			// Clamp to the (possibly changed) limit and warm-start.
			const Real impulse = glm::clamp(accumulatedImpulsePtr[i], -maxImpulse, maxImpulse);
			angularVelocityPtr[bodyIndex] += impulse * invInertia;

			data.invInertia = invInertia;
			data.inertia = Real(1) / invInertia;
			data.targetSpeed = targetSpeedPtr[i];
			data.maxImpulse = maxImpulse;
			data.impulse = impulse;
			accumulatedImpulsePtr[i] = impulse;
		}
	}

	void MotorSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve motor velocity constraints", Ecstasy::Core::Color::Violet);

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
