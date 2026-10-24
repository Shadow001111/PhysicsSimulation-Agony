#include "PrismaticJointSolver.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <cmath>
#include <algorithm>

namespace PS_AGONY2D
{
	namespace
	{
		constexpr Real PJ_PI = Real(3.14159265358979323846);
		constexpr Real PJ_TWO_PI = Real(2) * PJ_PI;

		// Wraps an angle (radians) into (-pi, pi].
		inline Real wrapAngleToPi(Real angle)
		{
			angle -= PJ_TWO_PI * std::floor((angle + PJ_PI) / PJ_TWO_PI);
			return angle;
		}
	}

	void PrismaticJointSolver::computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices, size_t outOffset)
	{
		TRACY_SCOPE_NC("Compute prismatic joint constraint data", PS_AGONY2D::Profiling::Color::Chocolate);

		auto rotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
			{
				return { v.x * cos - v.y * sin, v.x * sin + v.y * cos };
			};
		auto cross = [](const Vec2& a, const Vec2& b) -> Real
			{
				return a.x * b.y - a.y * b.x;
			};

		const Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		const Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT rotationPtr = bodies->rotation.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		const Vec2* ECSTASY_RESTRICT localAxisAPtr = viewer.localAxisA;
		const Real* ECSTASY_RESTRICT referenceAnglePtr = viewer.referenceAngle;
		const Real* ECSTASY_RESTRICT stiffnessPtr = viewer.stiffness;
		const Real* ECSTASY_RESTRICT dampingPtr = viewer.damping;
		const Real* ECSTASY_RESTRICT lowerLimitPtr = viewer.lowerLimit;
		const Real* ECSTASY_RESTRICT upperLimitPtr = viewer.upperLimit;
		const uint8_t* ECSTASY_RESTRICT enableLimitPtr = viewer.enableLimit;

		const size_t jointCount = jointIndices.size();

		const size_t* ECSTASY_RESTRICT jointIndicesPtr = jointIndices.data();

		const Real invDeltaTime = deltaTime > Real(1e-12) ? Real(1) / deltaTime : Real(0);

		constexpr Real LIMIT_SLOP = Real(0.005);
		constexpr Real LIMIT_BAUMGARTE = Real(0.2);

		// Compute constant data.
		for (size_t jointIndex = 0; jointIndex < jointCount; jointIndex++)
		{
			const size_t i = jointIndicesPtr[jointIndex];
			PrismaticJointConstraintData& constraintData = constraintContainer[outOffset + jointIndex];

			const ObjectDenseIndex bodyIndexA = viewer.bodyIndexA[i];
			const ObjectDenseIndex bodyIndexB = viewer.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;
			constraintData.blockValid = false;
			constraintData.limitDirection = 0;
			constraintData.axisInvEffectiveMass = 0;
			constraintData.accumulatedAxisImpulse = 0;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			// Both bodies are entirely fixed -> nothing to solve.
			if (isStaticPtr[bodyIndexA] && isStaticPtr[bodyIndexB]) [[unlikely]]
			{
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

			const Real cosA = rotationCosPtr[bodyIndexA];
			const Real sinA = rotationSinPtr[bodyIndexA];
			const Real cosB = rotationCosPtr[bodyIndexB];
			const Real sinB = rotationSinPtr[bodyIndexB];

			const Vec2 rotatedAnchorA = rotate(viewer.localAnchorA[i], cosA, sinA);
			const Vec2 rotatedAnchorB = rotate(viewer.localAnchorB[i], cosB, sinB);

			const Vec2 worldAnchorA = worldCenterA + rotatedAnchorA;
			const Vec2 worldAnchorB = worldCenterB + rotatedAnchorB;

			const Vec2 d = worldAnchorB - worldAnchorA;

			// Axis lives on body A and rotates with it.
			const Vec2 axis = rotate(localAxisAPtr[i], cosA, sinA);
			const Vec2 perp{ -axis.y, axis.x };

			constraintData.axis = axis;
			constraintData.perp = perp;

			const Real s1 = cross(d + rotatedAnchorA, perp);
			const Real s2 = cross(rotatedAnchorB, perp);

			constraintData.s1 = s1;
			constraintData.s2 = s2;

			// Perpendicular translation + relative-angle block (soft).
			{
				const Real k11 = invMassA + invMassB + invInertiaA * s1 * s1 + invInertiaB * s2 * s2;
				const Real k12 = invInertiaA * s1 + invInertiaB * s2;
				Real k22 = invInertiaA + invInertiaB;
				if (k22 == Real(0)) k22 = Real(1); // Avoid a singular angular row when both bodies are angularly fixed.

				const Real k = stiffnessPtr[i];
				const Real c = dampingPtr[i];

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

				const Real k11Soft = k11 + gamma;
				const Real k22Soft = k22 + gamma;

				const Real det = k11Soft * k22Soft - k12 * k12;
				if (det > Real(1e-12)) [[likely]]
				{
					const Real invDet = Real(1) / det;

					constraintData.invK11 = k22Soft * invDet;
					constraintData.invK22 = k11Soft * invDet;
					constraintData.invK12 = -k12 * invDet;
					constraintData.blockValid = true;

					// Perpendicular position error: how far the anchors have drifted off the axis line.
					const Real C1 = glm::dot(perp, d);

					// Relative angle error, robust to each body's rotation wrapping independently.
					const Real currentRelAngle = wrapAngleToPi(rotationPtr[bodyIndexB] - rotationPtr[bodyIndexA]);
					const Real C2 = wrapAngleToPi(currentRelAngle - referenceAnglePtr[i]);

					constraintData.bias1 = beta * C1;
					constraintData.bias2 = beta * C2;
				}
			}

			// Axis limit (hard, one-sided stop).
			if (enableLimitPtr[i])
			{
				const Real translation = glm::dot(axis, d);
				const Real lower = lowerLimitPtr[i];
				const Real upper = upperLimitPtr[i];

				Real direction = 0;
				Real C = 0;
				if (translation <= lower + LIMIT_SLOP)
				{
					direction = Real(1);
					C = translation - lower;
				}
				else if (translation >= upper - LIMIT_SLOP)
				{
					direction = Real(-1);
					C = translation - upper;
				}

				if (direction != Real(0))
				{
					const Real a1 = cross(d + rotatedAnchorA, axis);
					const Real a2 = cross(rotatedAnchorB, axis);

					const Real kAxis = invMassA + invMassB + invInertiaA * a1 * a1 + invInertiaB * a2 * a2;
					if (kAxis > Real(1e-12)) [[likely]]
					{
						constraintData.a1 = a1;
						constraintData.a2 = a2;
						constraintData.axisInvEffectiveMass = Real(1) / kAxis;
						constraintData.limitDirection = direction;
						constraintData.axisBias = LIMIT_BAUMGARTE * invDeltaTime * C;
					}
				}
			}
		}
	}

	void PrismaticJointSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve prismatic joint velocity constraints", PS_AGONY2D::Profiling::Color::Violet);

		std::span<PrismaticJointConstraintData> constraintDataSpan(constraintContainer.data() + pass.start, pass.size);

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
			PrismaticJointConstraintData& constraintData = constraintDataSpan[c];
			if (!constraintData.blockValid && constraintData.limitDirection == Real(0)) continue;

			const ObjectDenseIndex bodyIndexA = constraintData.bodyIndexA;
			const ObjectDenseIndex bodyIndexB = constraintData.bodyIndexB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			Vec2 linearVelocityA{ velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB{ velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			// --- Perpendicular translation + relative-angle block. ---
			if (constraintData.blockValid)
			{
				const Vec2 perp = constraintData.perp;
				const Real s1 = constraintData.s1;
				const Real s2 = constraintData.s2;

				const Real Cdot1 = glm::dot(perp, linearVelocityB - linearVelocityA) + s2 * angularVelocityB - s1 * angularVelocityA;
				const Real Cdot2 = angularVelocityB - angularVelocityA;

				const Real rhs1 = Cdot1 + constraintData.bias1;
				const Real rhs2 = Cdot2 + constraintData.bias2;

				const Real lambda1 = -(constraintData.invK11 * rhs1 + constraintData.invK12 * rhs2);
				const Real lambda2 = -(constraintData.invK12 * rhs1 + constraintData.invK22 * rhs2);

				const Vec2 P = perp * lambda1;
				const Real LA = lambda1 * s1 + lambda2;
				const Real LB = lambda1 * s2 + lambda2;

				linearVelocityA -= P * invMassA;
				angularVelocityA -= LA * invInertiaA;

				linearVelocityB += P * invMassB;
				angularVelocityB += LB * invInertiaB;
			}

			// --- Axis limit (hard, one-sided stop). Impulse is accumulated and clamped
			//     across iterations so the stop can only push (never pull) in its direction. ---
			if (constraintData.limitDirection != Real(0))
			{
				const Vec2 axis = constraintData.axis;
				const Real a1 = constraintData.a1;
				const Real a2 = constraintData.a2;

				const Real Cdot = glm::dot(axis, linearVelocityB - linearVelocityA) + a2 * angularVelocityB - a1 * angularVelocityA;

				const Real rawImpulse = -(Cdot + constraintData.axisBias) * constraintData.axisInvEffectiveMass;

				const Real oldImpulse = constraintData.accumulatedAxisImpulse;
				const Real newImpulse = constraintData.limitDirection > Real(0)
					? std::max(oldImpulse + rawImpulse, Real(0))
					: std::min(oldImpulse + rawImpulse, Real(0));

				const Real appliedImpulse = newImpulse - oldImpulse;
				constraintData.accumulatedAxisImpulse = newImpulse;

				const Vec2 P = axis * appliedImpulse;
				const Real LA = appliedImpulse * a1;
				const Real LB = appliedImpulse * a2;

				linearVelocityA -= P * invMassA;
				angularVelocityA -= LA * invInertiaA;

				linearVelocityB += P * invMassB;
				angularVelocityB += LB * invInertiaB;
			}

			// Apply impulse.
			if (!isStaticPtr[bodyIndexA]) [[likely]]
			{
				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;

				angularVelocityPtr[bodyIndexA] = angularVelocityA;
			}
			if (!isStaticPtr[bodyIndexB]) [[likely]]
			{
				velocityXPtr[bodyIndexB] = linearVelocityB.x;
				velocityYPtr[bodyIndexB] = linearVelocityB.y;

				angularVelocityPtr[bodyIndexB] = angularVelocityB;
			}
		}
	}
}