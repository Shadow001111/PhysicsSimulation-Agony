#include "BodyCollisionSolver.h"
#include "../Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
	void BodyCollisionSolver::setDataViewers(
		BodySoA& bodiesIn,
		const std::vector<Material>& materialsIn,
		SolvingPlanner& solvingPlannerIn
	)
	{
		SolverBase::setResources(bodiesIn, solvingPlannerIn);
		materials = &materialsIn;
	}

	size_t BodyCollisionSolver::getMemoryUsage() const
	{
		size_t total = sizeof(BodyCollisionSolver);

		total += getVectorMemoryUsage(positionConstraintContainer);
		total += getVectorMemoryUsage(velocityConstraintContainer);
		total += getVectorMemoryUsage(frictionDataContainer);

		total += getVectorMemoryUsage(collidingBodyPairs);
		total += getVectorMemoryUsage(orderedCollisionData);

		return total;
	}

	void BodyCollisionSolver::solveCollisions(
		const std::vector<BodyCollisionData>& narrowPhaseCollisions,
		uint32_t velocityIterations,
		uint32_t positionIterations
	)
	{
		const size_t workerCount = planWorkerCount(narrowPhaseCollisions.size());

		// Single-threaded path.
		if (workerCount <= 1)
		{
			TRACY_SCOPE_NC("Solve collisions (Single-threaded)", Ecstasy::Color::OliveDrab);

			computeConstraintData(narrowPhaseCollisions);

			for (uint32_t i = 0; i < velocityIterations; i++)
			{
				solveVelocityConstraints(i == 0, narrowPhaseCollisions, velocityConstraintContainer, frictionDataContainer);
			}
			for (uint32_t i = 0; i < positionIterations; i++)
			{
				solvePositionConstraints(narrowPhaseCollisions, positionConstraintContainer);
			}
			return;
		}

		// Multi-threading path.
		TRACY_SCOPE_NC("Solve collisions (Multi-threaded)", Ecstasy::Color::OliveDrab);
		{
			TRACY_SCOPE_NC("Collect body pairs from collision data", Ecstasy::Color::Red);

			const size_t collisionCount = narrowPhaseCollisions.size();

			collidingBodyPairs.resize(collisionCount);
			for (size_t i = 0; i < collisionCount; i++)
			{
				const BodyCollisionData& collData = narrowPhaseCollisions[i];
				collidingBodyPairs[i] = { collData.bodyA, collData.bodyB };
			}
		}
		{
			TRACY_SCOPE_NC("Plan execution", Ecstasy::Color::Blue);
			solvingPlanner->setWorkerCount(workerCount);

			solvingPlanner->planStandardExecution(collidingBodyPairs, bodies->getCount());

			//solvingPlanner->planCacheLineAwareExecution(
			//	collidingBodyPairs, bodies->getCount(),
			//	sizeof(Real),
			//	std::hardware_destructive_interference_size
			//);
		}
		{
			TRACY_SCOPE_N("Reorder collision data");

			const auto& flatIndices = solvingPlanner->getFlatIndices();
			const size_t mappedCount = flatIndices.size();
			const size_t* ECSTASY_RESTRICT flatIndicesPtr = flatIndices.data();

			orderedCollisionData.resize(mappedCount);
			for (size_t i = 0; i < mappedCount; i++)
			{
				const size_t originalIndex = flatIndicesPtr[i];
				orderedCollisionData[i] = narrowPhaseCollisions[originalIndex];
			}
		}
		computeConstraintData(orderedCollisionData);

		solveConstraintsThreaded(orderedCollisionData, velocityIterations, positionIterations);

		// Scatter persistent contact data back to the detector's original container.
		if (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
		{
			TRACY_SCOPE_N("Scatter persistent contact data back");
			const auto& flatIndices = solvingPlanner->getFlatIndices();
			const size_t mappedCount = flatIndices.size();
			for (size_t i = 0; i < mappedCount; i++)
			{
				const size_t originalIndex = flatIndices[i];
				// narrowPhaseCollisions is const&, but persistentContactData is mutable -> legal write.
				narrowPhaseCollisions[originalIndex].persistentContactData = orderedCollisionData[i].persistentContactData;
			}
		}
	}

	size_t BodyCollisionSolver::planWorkerCount(size_t collisionCount) const
	{
		static constexpr size_t COLLISION_COUNT_PER_WORKER = 830 * 2;
		static constexpr size_t MIN_COLLISION_COUNT_FOR_THREADING = COLLISION_COUNT_PER_WORKER * 3;

		// Single-threaded path.
		if (collisionCount < MIN_COLLISION_COUNT_FOR_THREADING)
		{
			return 0;
		}

		// Multi-threading path.
		auto& threadPool = Threading::getGlobalThreadPool();

		const size_t availableWorkerCount = threadPool.getThreadCount();
		const size_t neededWorkerCount = collisionCount / COLLISION_COUNT_PER_WORKER;

		return std::min(availableWorkerCount, neededWorkerCount);
	}

	void BodyCollisionSolver::computeConstraintData(const std::vector<BodyCollisionData>& collisionDataContainer)
	{
		TRACY_SCOPE_NC("Compute constraint data", Ecstasy::Color::Chocolate);

		Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const MaterialIndex* ECSTASY_RESTRICT materialIndexPtr = bodies->materialIndex.data();
		const Material* ECSTASY_RESTRICT materialPtr = materials->data();

		auto getCenterOfMass = [&](ObjectIndex bodyIndex) -> Vec2
			{
				return {
					positionXPtr[bodyIndex] + localCenterOfMassXPtr[bodyIndex],
					positionYPtr[bodyIndex] + localCenterOfMassYPtr[bodyIndex]
				};
			};

		auto invRotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
			{
				return { v.x * cos + v.y * sin, -v.x * sin + v.y * cos };
			};

		// Resize containers.
		const size_t collisionCount = collisionDataContainer.size();
		positionConstraintContainer.resize(collisionCount);
		velocityConstraintContainer.resize(collisionCount);
		frictionDataContainer.resize(collisionCount);

		const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

		// Compute constant data.
		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = collisionDataPtr[c];
			const Vec2 centerOfMassA = getCenterOfMass(data.bodyA);
			const Vec2 centerOfMassB = getCenterOfMass(data.bodyB);
			{
				const Vec2 contactPoint = data.contactPoints[0]; // Only the first point is used.

				positionConstraintContainer[c].localAnchorA = invRotate(contactPoint - centerOfMassA, rotationCosPtr[data.bodyA], rotationSinPtr[data.bodyA]);
				positionConstraintContainer[c].localAnchorB = invRotate(contactPoint - centerOfMassB, rotationCosPtr[data.bodyB], rotationSinPtr[data.bodyB]);
			}
			{
				auto& velocityConstraint = velocityConstraintContainer[c];
				auto& frictionData = frictionDataContainer[c];

				const MaterialIndex materialIndexA = materialIndexPtr[data.bodyA];
				const MaterialIndex materialIndexB = materialIndexPtr[data.bodyB];
				const Material* materialA = materialPtr + materialIndexA;
				const Material* materialB = materialPtr + materialIndexB;

				const Real elasticityPlusOne = (materialA->elasticity + materialB->elasticity) * Real(0.5) + Real(1.0);
				frictionData.staticFriction = std::sqrt(std::fmax(Real(0), materialA->staticFriction * materialB->staticFriction));
				frictionData.dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

				const Real invMassA = invMassPtr[data.bodyA];
				const Real invMassB = invMassPtr[data.bodyB];
				const Real invInertiaA = invInertiaPtr[data.bodyA];
				const Real invInertiaB = invInertiaPtr[data.bodyB];
				const Real effectiveMass = invMassA + invMassB;

				const Vec2 normal = data.normal;
				const Vec2 tangent{ -normal.y, normal.x };

				velocityConstraint.points[0].rAPerp = Vec2();
				velocityConstraint.points[0].rBPerp = Vec2();
				velocityConstraint.points[1].rAPerp = Vec2();
				velocityConstraint.points[1].rBPerp = Vec2();

				for (uint32_t i = 0; i < data.contactCount; i++)
				{
					const Vec2 contactPoint = data.contactPoints[i];
					auto& point = velocityConstraint.points[i];

					const Vec2 rA = contactPoint - centerOfMassA;
					const Vec2 rB = contactPoint - centerOfMassB;
					point.rAPerp = { -rA.y, rA.x };
					point.rBPerp = { -rB.y, rB.x };

					const Real rAPerpDotN = glm::dot(point.rAPerp, normal);
					const Real rBPerpDotN = glm::dot(point.rBPerp, normal);
					const Real normalDenom = effectiveMass
						+ rAPerpDotN * rAPerpDotN * invInertiaA
						+ rBPerpDotN * rBPerpDotN * invInertiaB;
					point.normalMassXElasticityFactor = normalDenom > Real(0) ? elasticityPlusOne / normalDenom : Real(0);

					const Real rAPerpDotT = glm::dot(point.rAPerp, tangent);
					const Real rBPerpDotT = glm::dot(point.rBPerp, tangent);
					const Real tangentDenom = effectiveMass
						+ rAPerpDotT * rAPerpDotT * invInertiaA
						+ rBPerpDotT * rBPerpDotT * invInertiaB;
					point.tangentMass = tangentDenom > Real(0) ? Real(1) / tangentDenom : Real(0);
				}
			}
		}
	}

	void BodyCollisionSolver::applyWarmStartingForCollisions(
		std::span<const BodyCollisionData> collisionDataContainer,
		std::span<const VelocityConstraintData> constraintDataContainer
	)
	{
		TRACY_SCOPE_NC("Apply warm starting for collisions", Ecstasy::Color::Violet);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t collisionCount = collisionDataContainer.size();
		for (size_t c = 0; c < collisionCount; c++)
		{
			const BodyCollisionData& collisionData = collisionDataContainer[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

			// Get body indices.
			const ObjectIndex bodyIndexA = collisionData.bodyA;
			const ObjectIndex bodyIndexB = collisionData.bodyB;

			// Get body data.
			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];

			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			//
			const Vec2 normal = collisionData.normal;
			const uint32_t contactCount = collisionData.contactCount;

			const Vec2 tangent = { -normal.y, normal.x };

			//
			std::array<Vec2, 2> impulseArray{};
			for (uint32_t i = 0; i < contactCount; i++)
			{
				Real oldJn = collisionData.persistentContactData[i].normalImpulseAccumulator;
				Real oldJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;

				const Vec2 warmStartImpulse = (oldJn * normal) + (oldJt * tangent);
				const bool isValid = oldJn > Real(0) || std::fabs(oldJt) > Real(0); // TODO: Check if it's right!

				impulseArray[i] = warmStartImpulse * Real(isValid);
			}
			{ // Can apply sum of impulses, because they don't change outcome.
				const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

				linearVelocityA -= impulseSum * invMassA;
				linearVelocityB += impulseSum * invMassB;

				const Real dotSumA =
					glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

				const Real dotSumB =
					glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

				angularVelocityA -= dotSumA * invInertiaA;
				angularVelocityB += dotSumB * invInertiaB;
			}

			// Store velocities.
			velocityXPtr[bodyIndexA] = linearVelocityA.x;
			velocityYPtr[bodyIndexA] = linearVelocityA.y;
			velocityXPtr[bodyIndexB] = linearVelocityB.x;
			velocityYPtr[bodyIndexB] = linearVelocityB.y;

			angularVelocityPtr[bodyIndexA] = angularVelocityA;
			angularVelocityPtr[bodyIndexB] = angularVelocityB;
		}
	}

	void BodyCollisionSolver::solveVelocityConstraints(
		bool firstIteration,
		std::span<const BodyCollisionData> collisionDataContainer,
		std::span<const VelocityConstraintData> constraintDataContainer,
		std::span<const FrictionData> frictionDataContainer
	)
	{
		TRACY_SCOPE_NC("Solve collision velocity constraints", Ecstasy::Color::Violet);

		// Optional warm-starting.
		if constexpr (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
		{
			if (firstIteration)
			{
				applyWarmStartingForCollisions(collisionDataContainer, constraintDataContainer);
			}
		}

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t collisionCount = collisionDataContainer.size();
		for (size_t c = 0; c < collisionCount; c++)
		{
			const BodyCollisionData& collisionData = collisionDataContainer[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

			// Get body indices.
			const ObjectIndex bodyIndexA = collisionData.bodyA;
			const ObjectIndex bodyIndexB = collisionData.bodyB;

			// Get body data.
			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];

			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			//
			const Vec2 normal = collisionData.normal;
			const uint32_t contactCount = collisionData.contactCount;

			const Vec2 tangent = { -normal.y, normal.x };

			[[maybe_unused]] std::array<Vec2, 2> impulseArray{};
			std::array<Real, 2> jnArray{};

			// Collision impulses.
			bool noContacts = true;
			for (uint32_t i = 0; i < contactCount; i++)
			{
				const auto& contactData = velocityConstraintData.points[i];

				const Vec2 rAPerp = contactData.rAPerp;
				const Vec2 rBPerp = contactData.rBPerp;

				const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
				const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

				const Vec2 relativeVelocity =
					(linearVelocityB + angularLinearVelB) -
					(linearVelocityA + angularLinearVelA);

				const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

				Real& accumulatedJn = collisionData.persistentContactData[i].normalImpulseAccumulator;

				if (velocityAlongNormal > Real(0) && accumulatedJn <= Real(0)) continue; // TODO: Check if that is okay!

				const Real jn = -velocityAlongNormal * contactData.normalMassXElasticityFactor;

				const Real oldJn = accumulatedJn;
				accumulatedJn = std::fmax(Real(0), oldJn + jn);
				const Real deltaJn = accumulatedJn - oldJn;

				jnArray[i] = accumulatedJn;
				noContacts = false;

				if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplyImpulsesSequentially)
				{
					const Vec2 impulse = deltaJn * normal;
					linearVelocityA -= impulse * invMassA;
					angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

					linearVelocityB += impulse * invMassB;
					angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
				}
				else if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
				{
					impulseArray[i] = deltaJn * normal;
				}
			}

			// Check if there is at least one valid contact.
			if (noContacts)
			{
				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;
				velocityXPtr[bodyIndexB] = linearVelocityB.x;
				velocityYPtr[bodyIndexB] = linearVelocityB.y;
				continue;
			}

			if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
			{
				const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

				linearVelocityA -= impulseSum * invMassA;
				linearVelocityB += impulseSum * invMassB;

				const Real dotSumA =
					glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

				const Real dotSumB =
					glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

				angularVelocityA -= dotSumA * invInertiaA;
				angularVelocityB += dotSumB * invInertiaB;

				// Reset impulse array for friction accumulation step.
				impulseArray[0] = Vec2();
				impulseArray[1] = Vec2();
			}

			// Friction impulses.
			const FrictionData& frictionData = frictionDataContainer[c];
			const Real staticFriction = frictionData.staticFriction;
			const Real dynamicFriction = frictionData.dynamicFriction;
			for (uint32_t i = 0; i < contactCount; i++)
			{
				const auto& contactData = velocityConstraintData.points[i];

				const Vec2 rAPerp = contactData.rAPerp;
				const Vec2 rBPerp = contactData.rBPerp;

				const Vec2 angularLinearVelA = rAPerp * angularVelocityA;
				const Vec2 angularLinearVelB = rBPerp * angularVelocityB;

				const Vec2 relativeVelocity =
					(linearVelocityB + angularLinearVelB) -
					(linearVelocityA + angularLinearVelA);

				const Real currentSlipVel = glm::dot(relativeVelocity, tangent);

				const Real jt = -currentSlipVel * contactData.tangentMass;

				Real& accumulatedJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;
				const Real oldJt = accumulatedJt;
				Real targetJt = oldJt + jt;

				const Real jn = jnArray[i];

				if (std::fabs(targetJt) <= jn * staticFriction)
				{
					accumulatedJt = targetJt; // Static friction.
				}
				else
				{
					const Real maxDynamic = jn * dynamicFriction;
					accumulatedJt = std::clamp(targetJt, -maxDynamic, maxDynamic); // Dynamic friction.
				}

				const Real deltaJt = accumulatedJt - oldJt;

				const Vec2 impulse = deltaJt * tangent;
				if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplyImpulsesSequentially)
				{
					linearVelocityA -= impulse * invMassA;
					angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;

					linearVelocityB += impulse * invMassB;
					angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
				}
				else if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
				{
					impulseArray[i] = impulse;
				}
			}

			if constexpr (VELOCITY_SOLVER_TYPE == VelocitySolverType::ApplySumOfImpulses)
			{
				const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

				linearVelocityA -= impulseSum * invMassA;
				linearVelocityB += impulseSum * invMassB;

				const Real dotSumA =
					glm::dot(velocityConstraintData.points[0].rAPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rAPerp, impulseArray[1]);

				const Real dotSumB =
					glm::dot(velocityConstraintData.points[0].rBPerp, impulseArray[0]) +
					glm::dot(velocityConstraintData.points[1].rBPerp, impulseArray[1]);

				angularVelocityA -= dotSumA * invInertiaA;
				angularVelocityB += dotSumB * invInertiaB;
			}

			// Store velocities.
			velocityXPtr[bodyIndexA] = linearVelocityA.x;
			velocityYPtr[bodyIndexA] = linearVelocityA.y;
			velocityXPtr[bodyIndexB] = linearVelocityB.x;
			velocityYPtr[bodyIndexB] = linearVelocityB.y;

			angularVelocityPtr[bodyIndexA] = angularVelocityA;
			angularVelocityPtr[bodyIndexB] = angularVelocityB;
		}
	}

	void BodyCollisionSolver::solvePositionConstraints(
		std::span<const BodyCollisionData> collisionDataContainer,
		std::span<const PositionConstraintData> constraintDataContainer
	)
	{
		TRACY_SCOPE_NC("Solve collision position constraints", Ecstasy::Color::Indigo);

		Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();

		auto getCenterOfMass = [&](ObjectIndex bodyIndex) -> Vec2
			{
				return {
					positionXPtr[bodyIndex] + localCenterOfMassXPtr[bodyIndex],
					positionYPtr[bodyIndex] + localCenterOfMassYPtr[bodyIndex]
				};
			};

		auto rotate = [](const Vec2& v, Real cos, Real sin) -> Vec2
			{
				return { v.x * cos - v.y * sin, v.x * sin + v.y * cos };
			};

		const size_t collisionCount = collisionDataContainer.size();

		// Note: I tried to use Simd, it was slower, probably because of the gather-scatter, or just memory intensive.
		//       plus it was invalid because same body index could appear multiple times in simd batch (on same worker).
		//       remaking solving planner to account for that would be hell!

		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = collisionDataContainer[c];
			const ObjectIndex bodyIndexA = data.bodyA;
			const ObjectIndex bodyIndexB = data.bodyB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real effectiveMass = invMassA + invMassB;

			// Re-derive the world anchors from the current (possibly already corrected) transforms.
			const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
			const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

			const PositionConstraintData& positionConstraintData = constraintDataContainer[c];
			const Vec2 worldAnchorA = centerOfMassA + rotate(positionConstraintData.localAnchorA, rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 worldAnchorB = centerOfMassB + rotate(positionConstraintData.localAnchorB, rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

			// The anchors coincided (drift = 0) when depth was measured, so drift is exactly
			// how much penetration has already been resolved since then.
			const Real drift = glm::dot(worldAnchorB - worldAnchorA, data.normal);
			const Real currentPenetration = data.penetration - drift;

			const Real correctionDepth = currentPenetration - simulationSettings.positionCorrectionSlop;
			if (correctionDepth <= Real(0)) continue;

			const Real totalCorrection = correctionDepth / effectiveMass * simulationSettings.positionCorrectionPercent;
			const Real correctionA = invMassA * totalCorrection;
			const Real correctionB = invMassB * totalCorrection;

			const Vec2 correctionAVec = data.normal * correctionA;
			const Vec2 correctionBVec = data.normal * correctionB;

			positionXPtr[bodyIndexA] -= correctionAVec.x;
			positionYPtr[bodyIndexA] -= correctionAVec.y;
			positionXPtr[bodyIndexB] += correctionBVec.x;
			positionYPtr[bodyIndexB] += correctionBVec.y;
		}
	}

	void BodyCollisionSolver::solveConstraintsThreaded(
		const std::vector<BodyCollisionData>& collisionDataContainer,
		uint32_t velocityIterations,
		uint32_t positionIterations
	)
	{
		const size_t workerCount = solvingPlanner->getWorkerCount();

		const auto& passOffsets = solvingPlanner->getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0)) [[unlikely]]
		{
			return;
		}

		const uint32_t positionSolvingStartTick = static_cast<uint32_t>(waveCount) * velocityIterations;
		const uint32_t totalTicks = positionSolvingStartTick + static_cast<uint32_t>(waveCount) * positionIterations;

		const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

		runThreadedWaves(workerCount, waveCount, totalTicks,
			[&](size_t waveIndex, size_t workerIndex, uint32_t passTicket)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];

				// Get work from current wave and execute it. If empty, skip.
				if (passOffset.size == 0) [[unlikely]] return;

				std::span<const BodyCollisionData> collisionSlice(collisionDataPtr + passOffset.start, passOffset.size);

				if (passTicket >= positionSolvingStartTick)
				{
					std::span<const PositionConstraintData> constraintSlice(positionConstraintContainer.data() + passOffset.start, passOffset.size);
					solvePositionConstraints(collisionSlice, constraintSlice);
				}
				else
				{
					std::span<const VelocityConstraintData> constraintSlice(velocityConstraintContainer.data() + passOffset.start, passOffset.size);
					std::span<const FrictionData> frictionDataSlice(frictionDataContainer.data() + passOffset.start, passOffset.size);

					solveVelocityConstraints(passTicket == 0, collisionSlice, constraintSlice, frictionDataSlice);
				}
			}
		);
	}
}