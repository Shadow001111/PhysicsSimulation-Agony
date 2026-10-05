#include "SpringSolver.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
	void SpringSolver::computeConstraintData(Real deltaTime, std::span<const size_t> springIndices, size_t outOffset)
	{
		TRACY_SCOPE_NC("Compute spring constraint data", Ecstasy::Core::Color::Chocolate);

		auto rotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
			{
				return { v.x * cos - v.y * sin, v.x * sin + v.y * cos };
			};

		const Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		const Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		const Real* ECSTASY_RESTRICT springRestLengthPtr = viewer.restLength;
		const Real* ECSTASY_RESTRICT springStiffnessPtr = viewer.stiffness;
		const Real* ECSTASY_RESTRICT springDampingPtr = viewer.damping;

		const size_t springCount = springIndices.size();

		const size_t* ECSTASY_RESTRICT springIndicesPtr = springIndices.data();

		// Compute constant data.
		for (size_t springIndex = 0; springIndex < springCount; springIndex++)
		{
			const size_t i = springIndicesPtr[springIndex];
			SpringConstraintData& constraintData = constraintContainer[outOffset + springIndex];

			const ObjectIndex bodyIndexA = viewer.bodyIndexA[i];
			const ObjectIndex bodyIndexB = viewer.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];

			// Both attachments are static anchors -> nothing to solve.
			if (isStaticPtr[bodyIndexA] && isStaticPtr[bodyIndexB]) [[unlikely]]
			{
				constraintData.invEffectiveMass = Real(0);
				continue;
			}

			const Vec2 worldCenterA{
				positionXPtr[bodyIndexA] + localCenterOfMassXPtr[bodyIndexA],
				positionYPtr[bodyIndexA] + localCenterOfMassYPtr[bodyIndexA]
			};
			const Vec2 worldCenterB{
				positionXPtr[bodyIndexB] + localCenterOfMassXPtr[bodyIndexB],
				positionYPtr[bodyIndexB] + localCenterOfMassYPtr[bodyIndexB]
			};

			const Vec2 rotatedAnchorA = rotate(viewer.localAnchorA[i], rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 rotatedAnchorB = rotate(viewer.localAnchorB[i], rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

			const Vec2 worldAnchorA = worldCenterA + rotatedAnchorA;
			const Vec2 worldAnchorB = worldCenterB + rotatedAnchorB;

			// Compute current length and direction.
			const Vec2 delta = worldAnchorB - worldAnchorA;
			const Real currentLengthSq = glm::dot(delta, delta);
			if (currentLengthSq < Real(1e-12)) [[unlikely]]
			{
				constraintData.invEffectiveMass = Real(0);
				continue;
			}

			const Real currentLength = glm::sqrt(currentLengthSq);
			const Vec2 dir = delta / currentLength;

			// Constraint error.
			const Real C = currentLength - springRestLengthPtr[i];

			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			const Real raCn = (rotatedAnchorA.x * dir.y) - (rotatedAnchorA.y * dir.x);
			const Real rbCn = (rotatedAnchorB.x * dir.y) - (rotatedAnchorB.y * dir.x);

			// Compute effective mass.
			const Real effectiveMass = invMassA + invMassB + invInertiaA * raCn * raCn + invInertiaB * rbCn * rbCn;
			if (effectiveMass <= Real(0)) [[unlikely]]
			{
				constraintData.invEffectiveMass = Real(0);
				continue;
			}

			// Soft constraint parameters from physical stiffness/damping.
			const Real k = springStiffnessPtr[i];
			const Real c = springDampingPtr[i];

			Real gamma = 0;
			Real beta = 0;
			{
				const Real denom = deltaTime * (c + deltaTime * k);
				if (denom > Real(1e-12)) [[likely]]
				{
					gamma = Real(1) / denom;
					beta = deltaTime * k * gamma;
				}
			}

			constraintData.dir = dir;
			constraintData.rotatedAnchorA = rotatedAnchorA;
			constraintData.rotatedAnchorB = rotatedAnchorB;
			constraintData.invEffectiveMass = Real(1) / (effectiveMass + gamma);
			constraintData.bias = beta * C;
		}
	}

	void SpringSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve spring velocity constraints", Ecstasy::Core::Color::Violet);

		std::span<const SpringConstraintData> constraintDataSpan(constraintContainer.data() + pass.start, pass.size);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		// Main loop.
		const size_t springCount = constraintDataSpan.size();

		for (size_t c = 0; c < springCount; c++)
		{
			const SpringConstraintData& constraintData = constraintDataSpan[c];
			if (constraintData.invEffectiveMass == Real(0)) continue;

			// Get body indices.
			const ObjectIndex bodyIndexA = constraintData.bodyIndexA;
			const ObjectIndex bodyIndexB = constraintData.bodyIndexB;

			// Get body data.
			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			const Vec2 rotatedAnchorA = constraintData.rotatedAnchorA;
			const Vec2 rotatedAnchorB = constraintData.rotatedAnchorB;
			const Vec2 dir = constraintData.dir;

			Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			// Compute linear velocities at points.
			const Vec2 angularLinearVelA = Vec2(-rotatedAnchorA.y, rotatedAnchorA.x) * angularVelocityA;
			const Vec2 angularLinearVelB = Vec2(-rotatedAnchorB.y, rotatedAnchorB.x) * angularVelocityB;

			const Vec2 relativeVelocity =
				(linearVelocityB + angularLinearVelB) -
				(linearVelocityA + angularLinearVelA);

			const Real Cdot = glm::dot(relativeVelocity, dir);

			// Compute impulse.
			const Real impulseMag = (Cdot + constraintData.bias) * constraintData.invEffectiveMass;
			const Vec2 impulse = dir * impulseMag;

			// Apply impulse.
			if (!isStaticPtr[bodyIndexA]) [[likely]]
			{
				linearVelocityA += impulse * invMassA;

				const Real torqueA = rotatedAnchorA.x * impulse.y - rotatedAnchorA.y * impulse.x;
				angularVelocityA += torqueA * invInertiaA;

				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;

				angularVelocityPtr[bodyIndexA] = angularVelocityA;
			}
			if (!isStaticPtr[bodyIndexB]) [[likely]]
			{
				linearVelocityB -= impulse * invMassB;

				const Real torqueB = rotatedAnchorB.x * impulse.y - rotatedAnchorB.y * impulse.x;
				angularVelocityB -= torqueB * invInertiaB;

				velocityXPtr[bodyIndexB] = linearVelocityB.x;
				velocityYPtr[bodyIndexB] = linearVelocityB.y;

				angularVelocityPtr[bodyIndexB] = angularVelocityB;
			}
		}
	}
}