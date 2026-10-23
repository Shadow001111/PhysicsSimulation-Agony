#include "SliderJointSolver.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <algorithm>

namespace PS_AGONY2D
{
	void SliderJointSolver::computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices, size_t outOffset)
	{
		TRACY_SCOPE_NC("Compute slider joint constraint data", PS_AGONY2D::Profiling::Color::Chocolate);

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
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const uint8_t* ECSTASY_RESTRICT isStaticPtr = bodies->isStatic.data();

		const Vec2* ECSTASY_RESTRICT localAxisAPtr = viewer.localAxisA;
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
			SliderJointConstraintData& constraintData = constraintContainer[outOffset + jointIndex];

			const ObjectIndex bodyIndexA = viewer.bodyIndexA[i];
			const ObjectIndex bodyIndexB = viewer.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;
			constraintData.invK = 0;
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

			// Perpendicular translation (soft scalar row).
			{
				const Real k = invMassA + invMassB + invInertiaA * s1 * s1 + invInertiaB * s2 * s2;

				const Real stiffness = stiffnessPtr[i];
				const Real damping = dampingPtr[i];

				Real gamma = 0;
				Real beta = 0;
				{
					const Real denom = deltaTime * (damping + deltaTime * stiffness);
					if (denom > Real(1e-12)) [[likely]]
					{
						gamma = Real(1) / denom;
						beta = deltaTime * stiffness * gamma;
					}
				}

				const Real kSoft = k + gamma;
				if (kSoft > Real(1e-12)) [[likely]]
				{
					constraintData.invK = Real(1) / kSoft;

					// Perpendicular position error: how far the anchors have drifted off the axis line.
					constraintData.bias = beta * glm::dot(perp, d);
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

	void SliderJointSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve slider joint velocity constraints", PS_AGONY2D::Profiling::Color::Violet);

		std::span<SliderJointConstraintData> constraintDataSpan(constraintContainer.data() + pass.start, pass.size);

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
			SliderJointConstraintData& constraintData = constraintDataSpan[c];
			const bool perpActive = constraintData.invK > Real(0);
			if (!perpActive && constraintData.limitDirection == Real(0)) continue;

			const ObjectIndex bodyIndexA = constraintData.bodyIndexA;
			const ObjectIndex bodyIndexB = constraintData.bodyIndexB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			Vec2 linearVelocityA{ velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB{ velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			// --- Perpendicular translation (soft). ---
			if (perpActive)
			{
				const Vec2 perp = constraintData.perp;
				const Real s1 = constraintData.s1;
				const Real s2 = constraintData.s2;

				const Real Cdot = glm::dot(perp, linearVelocityB - linearVelocityA) + s2 * angularVelocityB - s1 * angularVelocityA;

				const Real lambda = -constraintData.invK * (Cdot + constraintData.bias);

				const Vec2 P = perp * lambda;
				const Real LA = lambda * s1;
				const Real LB = lambda * s2;

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