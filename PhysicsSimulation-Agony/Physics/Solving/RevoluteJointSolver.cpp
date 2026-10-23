#include "RevoluteJointSolver.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY2D
{
	void RevoluteJointSolver::computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices, size_t outOffset)
	{
		TRACY_SCOPE_NC("Compute joint constraint data", PS_AGONY2D::Profiling::Color::Chocolate);

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

		const Real* ECSTASY_RESTRICT jointStiffnessPtr = viewer.stiffness;
		const Real* ECSTASY_RESTRICT jointDampingPtr = viewer.damping;

		const size_t jointCount = jointIndices.size();

		const size_t* ECSTASY_RESTRICT jointIndicesPtr = jointIndices.data();

		// Compute constant data.
		for (size_t jointIndex = 0; jointIndex < jointCount; jointIndex++)
		{
			const size_t i = jointIndicesPtr[jointIndex];
			RevoluteJointConstraintData& constraintData = constraintContainer[outOffset + jointIndex];

			const ObjectIndex bodyIndexA = viewer.bodyIndexA[i];
			const ObjectIndex bodyIndexB = viewer.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			// Both attachments are static anchors -> nothing to solve.
			if (isStaticPtr[bodyIndexA] && isStaticPtr[bodyIndexB]) [[unlikely]]
			{
				constraintData.invK11 = constraintData.invK12 = constraintData.invK22 = Real(0);
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

			// Positional error: the two anchors should coincide.
			const Vec2 positionError = worldAnchorB - worldAnchorA;

			// Effective mass matrix (standard 2-DOF point constraint), symmetric.
			const Real k11 = invMassA + invMassB + invInertiaA * rotatedAnchorA.y * rotatedAnchorA.y + invInertiaB * rotatedAnchorB.y * rotatedAnchorB.y;
			const Real k22 = invMassA + invMassB + invInertiaA * rotatedAnchorA.x * rotatedAnchorA.x + invInertiaB * rotatedAnchorB.x * rotatedAnchorB.x;
			const Real k12 = -invInertiaA * rotatedAnchorA.x * rotatedAnchorA.y - invInertiaB * rotatedAnchorB.x * rotatedAnchorB.y;

			// Soft constraint parameters from physical stiffness/damping (same derivation as SpringSolver).
			const Real k = jointStiffnessPtr[i];
			const Real c = jointDampingPtr[i];

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

			// Regularize the diagonal (soft constraint), then invert the 2x2 matrix.
			const Real k11Soft = k11 + gamma;
			const Real k22Soft = k22 + gamma;

			const Real det = k11Soft * k22Soft - k12 * k12;
			if (det <= Real(1e-12)) [[unlikely]]
			{
				constraintData.invK11 = constraintData.invK12 = constraintData.invK22 = Real(0);
				continue;
			}
			const Real invDet = Real(1) / det;

			constraintData.rotatedAnchorA = rotatedAnchorA;
			constraintData.rotatedAnchorB = rotatedAnchorB;
			constraintData.invK11 = k22Soft * invDet;
			constraintData.invK22 = k11Soft * invDet;
			constraintData.invK12 = -k12 * invDet;
			constraintData.bias = beta * positionError;
		}
	}

	void RevoluteJointSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve joint velocity constraints", PS_AGONY2D::Profiling::Color::Violet);

		std::span<const RevoluteJointConstraintData> constraintDataSpan(constraintContainer.data() + pass.start, pass.size);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		// Main loop.
		const size_t jointCount = constraintDataSpan.size();

		for (size_t c = 0; c < jointCount; c++)
		{
			const RevoluteJointConstraintData& constraintData = constraintDataSpan[c];
			if (constraintData.invK11 == Real(0) && constraintData.invK12 == Real(0) && constraintData.invK22 == Real(0)) continue;

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

			Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			// Compute linear velocities at the anchor points.
			const Vec2 angularLinearVelA = Vec2(-rotatedAnchorA.y, rotatedAnchorA.x) * angularVelocityA;
			const Vec2 angularLinearVelB = Vec2(-rotatedAnchorB.y, rotatedAnchorB.x) * angularVelocityB;

			const Vec2 relativeVelocity =
				(linearVelocityB + angularLinearVelB) -
				(linearVelocityA + angularLinearVelA);

			const Vec2 rhs = relativeVelocity + constraintData.bias;

			// Apply the inverse effective mass matrix (symmetric: [invK11 invK12; invK12 invK22]).
			const Vec2 impulse
			{
				constraintData.invK11 * rhs.x + constraintData.invK12 * rhs.y,
				constraintData.invK12 * rhs.x + constraintData.invK22 * rhs.y
			};

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