#include "BodyCollisionSolver.h"

#include "Physics/Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

namespace PS_AGONY
{
	void BodyCollisionSolver::setDataViewers(
		BodySoA& bodiesIn,
		const ColliderSoAViewer& collidersIn,
		const std::vector<Material>& materialsIn
	)
	{
		SolverBase::setResources(bodiesIn);
		colliders = collidersIn;
		materials = &materialsIn;
	}

	void BodyCollisionSolver::solveCollisions(
		const std::vector<BodyCollisionData>& narrowPhaseCollisions,
		uint32_t velocityIterations,
		uint32_t positionIterations
	)
	{
		TRACY_SCOPE_NC("Solve collisions", Ecstasy::Core::Color::OliveDrab);

		const size_t workerCount = planWorkerCount(narrowPhaseCollisions.size());

		// Single-threaded path.
		if (workerCount <= 1)
		{
			activeCollisionData = narrowPhaseCollisions;
			computeConstraintData(narrowPhaseCollisions);

			{
				TRACY_SCOPE_N("Solve collision constraints (Single-threaded)");

				const SolvingPlan::Pass fullPass{ 0, static_cast<uint32_t>(narrowPhaseCollisions.size()) };

				applyWarmStarting(fullPass);

				for (uint32_t i = 0; i < velocityIterations; i++)
				{
					solveVelocityConstraints(fullPass);
				}
				for (uint32_t i = 0; i < positionIterations; i++)
				{
					solvePositionConstraints(fullPass);
				}
			}
			return;
		}

		// Multi-threading path.
		{
			TRACY_SCOPE_NC("Collect body pairs from collision data", Ecstasy::Core::Color::Red);

			const size_t collisionCount = narrowPhaseCollisions.size();

			collidingBodyPairs.resize(collisionCount);
			for (size_t i = 0; i < collisionCount; i++)
			{
				const BodyCollisionData& collData = narrowPhaseCollisions[i];
				collidingBodyPairs[i] = { collData.bodyA, collData.bodyB };
			}
		}
		{
			TRACY_SCOPE_NC("Plan execution", Ecstasy::Core::Color::Blue);

			SolvingPlanner::threadLocal().planStandardExecution(collidingBodyPairs, bodies->getCount(), workerCount, solvingPlan);

			//SolvingPlanner::threadLocal().planCacheLineAwareExecution(
			//	collidingBodyPairs, bodies->getCount(), workerCount,
			//	sizeof(Real),
			//	std::hardware_destructive_interference_size,
			//	solvingPlan
			//);
		}
		{
			TRACY_SCOPE_N("Reorder collision data");

			const auto& flatIndices = solvingPlan.getFlatIndices();
			const size_t mappedCount = flatIndices.size();
			const size_t* ECSTASY_RESTRICT flatIndicesPtr = flatIndices.data();

			orderedCollisionData.resize(mappedCount);
			for (size_t i = 0; i < mappedCount; i++)
			{
				const size_t originalIndex = flatIndicesPtr[i];
				orderedCollisionData[i] = narrowPhaseCollisions[originalIndex];
			}
		}
		activeCollisionData = orderedCollisionData;
		computeConstraintData(orderedCollisionData);

		{
			TRACY_SCOPE_N("Solve collision constraints (Multi-threaded)");
			solveConstraintsThreaded(orderedCollisionData, velocityIterations, positionIterations);
		}

		// Scatter persistent contact data back to the detector's original container.
		{
			TRACY_SCOPE_N("Scatter persistent contact data back");
			const auto& flatIndices = solvingPlan.getFlatIndices();
			const size_t mappedCount = flatIndices.size();
			for (size_t i = 0; i < mappedCount; i++)
			{
				const size_t originalIndex = flatIndices[i];
				// narrowPhaseCollisions is const&, but persistentContactData is mutable -> legal write.
				narrowPhaseCollisions[originalIndex].persistentContactData = orderedCollisionData[i].persistentContactData;
			}
		}
	}

	void BodyCollisionSolver::reportNoCollisions()
	{
		activeCollisionData = {};
	}

	Vec2 BodyCollisionSolver::computeConstraintErrors() const
	{
		TRACY_SCOPE_NC("Compute (body collision) constraint errors", Ecstasy::Core::Color::Gray);

		const size_t collisionCount = activeCollisionData.size();
		if (collisionCount == 0) return { Real(0), Real(0) };

		const Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		const Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		const Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();

		Real totalApproachSpeed = Real(0);
		Real totalPenetration = Real(0);

		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = activeCollisionData[c];
			const ObjectIndex bodyIndexA = data.bodyA;
			const ObjectIndex bodyIndexB = data.bodyB;

			// Compute world anchors.
			const Vec2 positionA = { positionXPtr[bodyIndexA], positionYPtr[bodyIndexA] };
			const Vec2 positionB = { positionXPtr[bodyIndexB], positionYPtr[bodyIndexB] };

			const PositionConstraintData& positionConstraintData = positionConstraintContainer[c];
			const Vec2 worldAnchorA = positionA + positionConstraintData.transformedAnchorA;
			const Vec2 worldAnchorB = positionB + positionConstraintData.transformedAnchorB;

			// Compute drift.
			const Real drift = glm::dot(worldAnchorB - worldAnchorA, data.normal);
			const Real currentPenetration = data.penetration - drift;

			totalPenetration += std::fmax(currentPenetration, Real(0));

			// Velocity error: sum of remaining closing speed along the normal at each active contact point.
			const VelocityConstraintData& velocityConstraintData = velocityConstraintContainer[c];

			const Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			const Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			const Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			const Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			for (uint32_t i = 0; i < data.contactCount; i++)
			{
				const auto& contactData = velocityConstraintData.points[i];

				const Vec2 relativeVelocity = (linearVelocityB + contactData.rBPerp * angularVelocityB) -
					(linearVelocityA + contactData.rAPerp * angularVelocityA);

				const Real velocityAlongNormal = glm::dot(relativeVelocity, data.normal);
				const Real approachSpeed = -velocityAlongNormal; // Positive when bodies are still closing.

				totalApproachSpeed += approachSpeed * (approachSpeed > Real(0));
			}
		}

		return { totalApproachSpeed, totalPenetration };
	}

	size_t BodyCollisionSolver::getMemoryUsage() const
	{
		size_t total = sizeof(BodyCollisionSolver);

		total += getVectorMemoryUsage(positionConstraintContainer);
		total += getVectorMemoryUsage(velocityConstraintContainer);
		total += getVectorMemoryUsage(frictionDataContainer);

		total += getVectorMemoryUsage(collidingBodyPairs);
		total += getVectorMemoryUsage(orderedCollisionData);

		total += solvingPlan.getMemoryUsage();

		return total;
	}

	size_t BodyCollisionSolver::planWorkerCount(size_t collisionCount) const
	{
		// Having two additional threads drops execution time to 83% only.
		// I think cache-sharing is happening between threads.
		// I have SolvingPlanner function to arrange them by cache lines, but it's heavily reliant on body indices ordering.
		// It struggles much if collision graph is dense and random. It also makes wave count explode.
		// Efficiency metric matters (I guess), but wave count matters more (I guess).
		//return 0;

		static constexpr size_t COLLISION_COUNT_PER_WORKER = 830 * 4;
		static constexpr size_t MIN_COLLISION_COUNT_FOR_THREADING = COLLISION_COUNT_PER_WORKER * 2;

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
		TRACY_SCOPE_NC("Compute constraint data", Ecstasy::Core::Color::Chocolate);

		Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		const Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const MaterialIndex* ECSTASY_RESTRICT colliderMaterialIndexPtr = colliders.materialIndex;
		const Material* ECSTASY_RESTRICT materialPtr = materials->data();

		// Resize containers.
		const size_t collisionCount = collisionDataContainer.size();
		positionConstraintContainer.resize(collisionCount);
		velocityConstraintContainer.resize(collisionCount);
		frictionDataContainer.resize(collisionCount);

		const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

		// Compute constant data.
		// No need to zero-initialize point data. Active points will have their data overwritten.
		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = collisionDataPtr[c];

			const Vec2 positionA = { positionXPtr[data.bodyA], positionYPtr[data.bodyA] };
			const Vec2 positionB = { positionXPtr[data.bodyB], positionYPtr[data.bodyB] };

			const Vec2 localCOMA = { localCenterOfMassXPtr[data.bodyA], localCenterOfMassYPtr[data.bodyA] };
			const Vec2 localCOMB = { localCenterOfMassXPtr[data.bodyB], localCenterOfMassYPtr[data.bodyB] };

			const Vec2 centerOfMassA = positionA + localCOMA;
			const Vec2 centerOfMassB = positionB + localCOMB;
			{
				const Vec2 contactPoint = data.contactPoints[0]; // Only the first point is used.

				positionConstraintContainer[c].transformedAnchorA = contactPoint - positionA;
				positionConstraintContainer[c].transformedAnchorB = contactPoint - positionB;
			}
			{
				auto& velocityConstraint = velocityConstraintContainer[c];
				auto& frictionData = frictionDataContainer[c];

				const MaterialIndex materialIndexA = colliderMaterialIndexPtr[data.colliderA];
				const MaterialIndex materialIndexB = colliderMaterialIndexPtr[data.colliderB];
				const Material* materialA = materialPtr + materialIndexA;
				const Material* materialB = materialPtr + materialIndexB;

				const Real elasticity = (materialA->elasticity + materialB->elasticity) * Real(0.5);
				frictionData.staticFriction = std::sqrt(std::fmax(Real(0), materialA->staticFriction * materialB->staticFriction));
				frictionData.dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

				const Real invMassA = invMassPtr[data.bodyA];
				const Real invMassB = invMassPtr[data.bodyB];
				const Real invInertiaA = invInertiaPtr[data.bodyA];
				const Real invInertiaB = invInertiaPtr[data.bodyB];
				const Real effectiveMass = invMassA + invMassB;

				const Vec2 normal = data.normal;
				const Vec2 tangent{ -normal.y, normal.x };

				const Vec2 linearVelocityA = { velocityXPtr[data.bodyA], velocityYPtr[data.bodyA] };
				const Vec2 linearVelocityB = { velocityXPtr[data.bodyB], velocityYPtr[data.bodyB] };
				const Real angularVelocityA = angularVelocityPtr[data.bodyA];
				const Real angularVelocityB = angularVelocityPtr[data.bodyB];

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
					point.normalMass = normalDenom > Real(0) ? Real(1) / normalDenom : Real(0);

					const Real rAPerpDotT = glm::dot(point.rAPerp, tangent);
					const Real rBPerpDotT = glm::dot(point.rBPerp, tangent);
					const Real tangentDenom = effectiveMass
						+ rAPerpDotT * rAPerpDotT * invInertiaA
						+ rBPerpDotT * rBPerpDotT * invInertiaB;
					point.tangentMass = tangentDenom > Real(0) ? Real(1) / tangentDenom : Real(0);

					const Vec2 relativeVelocity =
						(linearVelocityB + point.rBPerp * angularVelocityB) -
						(linearVelocityA + point.rAPerp * angularVelocityA);

					const Real negVelocityAlongNormal = -glm::dot(relativeVelocity, normal);

					point.velocityBias = (negVelocityAlongNormal > SimulationSettings::RESTITUTION_VELOCITY_THRESHOLD) ? elasticity * negVelocityAlongNormal : Real(0);
				}
			}
		}
	}

	void BodyCollisionSolver::applyWarmStarting(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Apply warm starting (body collisions)", Ecstasy::Core::Color::Violet);

		std::span<const BodyCollisionData> collisionDataSpan = activeCollisionData.subspan(pass.start, pass.size);
		std::span<const VelocityConstraintData> constraintDataSpan(velocityConstraintContainer.data() + pass.start, pass.size);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t collisionCount = collisionDataSpan.size();
		for (size_t c = 0; c < collisionCount; c++)
		{
			const BodyCollisionData& collisionData = collisionDataSpan[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataSpan[c];

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

			const Vec2 normal = collisionData.normal;
			const uint32_t contactCount = collisionData.contactCount;

			const Vec2 tangent = { -normal.y, normal.x };

			std::array<Vec2, 2> impulseArray{};
			for (uint32_t i = 0; i < contactCount; i++)
			{
				Real oldJn = collisionData.persistentContactData[i].normalImpulseAccumulator;
				Real oldJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;

				impulseArray[i] = (oldJn * normal) + (oldJt * tangent);
			}
			{ // Can apply sum of impulses, because they don't change outcome.
				const Vec2 impulseSum = impulseArray[0] + impulseArray[1];

				linearVelocityA -= impulseSum * invMassA;
				linearVelocityB += impulseSum * invMassB;

				// Even if points will have incorrent values when contact count is 1, impulse from array at index 1 will be zero.

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
			if (invMassA > Real(0)) [[likely]]
			{
				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;
				angularVelocityPtr[bodyIndexA] = angularVelocityA;
			}
			if (invMassB > Real(0)) [[likely]]
			{
				velocityXPtr[bodyIndexB] = linearVelocityB.x;
				velocityYPtr[bodyIndexB] = linearVelocityB.y;
				angularVelocityPtr[bodyIndexB] = angularVelocityB;
			}
		}
	}

	void BodyCollisionSolver::solveVelocityConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve collision velocity constraints", Ecstasy::Core::Color::Violet);

		std::span<const BodyCollisionData> collisionDataSpan = activeCollisionData.subspan(pass.start, pass.size);
		std::span<const VelocityConstraintData> constraintDataSpan(velocityConstraintContainer.data() + pass.start, pass.size);
		std::span<const FrictionData> frictionDataSpan(frictionDataContainer.data() + pass.start, pass.size);

		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		const size_t collisionCount = collisionDataSpan.size();
		for (size_t c = 0; c < collisionCount; c++)
		{
			const BodyCollisionData& collisionData = collisionDataSpan[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataSpan[c];

			const ObjectIndex bodyIndexA = collisionData.bodyA;
			const ObjectIndex bodyIndexB = collisionData.bodyB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			Vec2 linearVelocityA = { velocityXPtr[bodyIndexA], velocityYPtr[bodyIndexA] };
			Vec2 linearVelocityB = { velocityXPtr[bodyIndexB], velocityYPtr[bodyIndexB] };
			Real angularVelocityA = angularVelocityPtr[bodyIndexA];
			Real angularVelocityB = angularVelocityPtr[bodyIndexB];

			const Vec2 normal = collisionData.normal;
			const uint32_t contactCount = collisionData.contactCount;
			const Vec2 tangent = { -normal.y, normal.x };

			const FrictionData& frictionData = frictionDataSpan[c];
			const Real staticFriction = frictionData.staticFriction;
			const Real dynamicFriction = frictionData.dynamicFriction;

			if (contactCount == 1)
			{
				const auto& contactData = velocityConstraintData.points[0];
				const Vec2 rAPerp = contactData.rAPerp;
				const Vec2 rBPerp = contactData.rBPerp;

				// Normal.
				Real jnSingle = 0;
				{
					const Vec2 relativeVelocity =
						(linearVelocityB + rBPerp * angularVelocityB) -
						(linearVelocityA + rAPerp * angularVelocityA);

					const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

					Real& accumulatedJn = collisionData.persistentContactData[0].normalImpulseAccumulator;
					if (!(velocityAlongNormal > 0 && accumulatedJn == 0))
					{
						const Real jn = (contactData.velocityBias - velocityAlongNormal) * contactData.normalMass;
						const Real oldJn = accumulatedJn;
						accumulatedJn = std::fmax(Real(0), oldJn + jn);
						const Real deltaJn = accumulatedJn - oldJn;

						jnSingle = accumulatedJn;

						const Vec2 impulse = deltaJn * normal;
						linearVelocityA -= impulse * invMassA;
						angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;
						linearVelocityB += impulse * invMassB;
						angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
					}
				}

				// Friction.
				{
					const Vec2 relativeVelocity =
						(linearVelocityB + rBPerp * angularVelocityB) -
						(linearVelocityA + rAPerp * angularVelocityA);

					const Real currentSlipVel = glm::dot(relativeVelocity, tangent);

					const Real jt = -currentSlipVel * contactData.tangentMass;

					Real& accumulatedJt = collisionData.persistentContactData[0].tangentImpulseAccumulator;
					const Real oldJt = accumulatedJt;
					Real targetJt = oldJt + jt;
					const Real jn = jnSingle;

					if (std::fabs(targetJt) <= jn * staticFriction)
					{
						accumulatedJt = targetJt;
					}
					else
					{
						const Real maxDynamic = jn * dynamicFriction;
						accumulatedJt = std::clamp(targetJt, -maxDynamic, maxDynamic);
					}

					const Real deltaJt = accumulatedJt - oldJt;
					const Vec2 impulse = deltaJt * tangent;

					linearVelocityA -= impulse * invMassA;
					angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;
					linearVelocityB += impulse * invMassB;
					angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
				}
			}
			else
			{
				const Real effectiveMass = invMassA + invMassB;

				// Normal.
				std::array<Real, 2> jnArray{};
				{
					const auto& contactData0 = velocityConstraintData.points[0];
					const auto& contactData1 = velocityConstraintData.points[1];

					const Vec2 relativeVelocity0 =
						(linearVelocityB + contactData0.rBPerp * angularVelocityB) -
						(linearVelocityA + contactData0.rAPerp * angularVelocityA);
					const Vec2 relativeVelocity1 =
						(linearVelocityB + contactData1.rBPerp * angularVelocityB) -
						(linearVelocityA + contactData1.rAPerp * angularVelocityA);

					const Real velocityAlongNormal0 = glm::dot(relativeVelocity0, normal);
					const Real velocityAlongNormal1 = glm::dot(relativeVelocity1, normal);

					Real& accumulatedImpulse0 = collisionData.persistentContactData[0].normalImpulseAccumulator;
					Real& accumulatedImpulse1 = collisionData.persistentContactData[1].normalImpulseAccumulator;

					const Real rnA0 = glm::dot(contactData0.rAPerp, normal);
					const Real rnB0 = glm::dot(contactData0.rBPerp, normal);
					const Real rnA1 = glm::dot(contactData1.rAPerp, normal);
					const Real rnB1 = glm::dot(contactData1.rBPerp, normal);

					const Real K00 = effectiveMass + rnA0 * rnA0 * invInertiaA + rnB0 * rnB0 * invInertiaB;
					const Real K11 = effectiveMass + rnA1 * rnA1 * invInertiaA + rnB1 * rnB1 * invInertiaB;
					const Real K01 = effectiveMass + rnA0 * rnA1 * invInertiaA + rnB0 * rnB1 * invInertiaB;
					//const Real K10 = K01;

					const Real bPrime0 = contactData0.velocityBias - velocityAlongNormal0 + K00 * accumulatedImpulse0 + K01 * accumulatedImpulse1;
					const Real bPrime1 = contactData1.velocityBias - velocityAlongNormal1 + K01 * accumulatedImpulse0 + K11 * accumulatedImpulse1;

					Real x0 = 0, x1 = 0;

					auto solveNormalBlock = [&]()
					{
						const Real det = K00 * K11 - K01 * K01;
						if (std::fabs(det) > Real(1e-6))
						{
							const Real invDet = Real(1) / det;
							x0 = (K11 * bPrime0 - K01 * bPrime1) * invDet;
							x1 = (K00 * bPrime1 - K01 * bPrime0) * invDet;
							if (x0 >= 0 && x1 >= 0) return;
						}
						{
							x0 = 0;
							x1 = K11 > 0 ? bPrime1 / K11 : 0;
							if (x1 >= 0 && (K01 * x1 - bPrime0) >= 0) return;
						}
						{
							x0 = K00 > 0 ? bPrime0 / K00 : 0;
							x1 = 0;
							if (x0 >= 0 && (K01 * x0 - bPrime1) >= 0) return;
						}
						// 'x1' is already set to zero in previous scope.
						x0 = 0;
						//x1 = 0;
					};

					solveNormalBlock();

					const Real deltaJn0 = x0 - accumulatedImpulse0;
					const Real deltaJn1 = x1 - accumulatedImpulse1;

					accumulatedImpulse0 = x0;
					accumulatedImpulse1 = x1;

					jnArray[0] = x0;
					jnArray[1] = x1;

					const Vec2 impulse0 = deltaJn0 * normal;
					const Vec2 impulse1 = deltaJn1 * normal;
					const Vec2 impulseSum = impulse0 + impulse1;

					linearVelocityA -= impulseSum * invMassA;
					linearVelocityB += impulseSum * invMassB;

					angularVelocityA -= (glm::dot(contactData0.rAPerp, impulse0) + glm::dot(contactData1.rAPerp, impulse1)) * invInertiaA;
					angularVelocityB += (glm::dot(contactData0.rBPerp, impulse0) + glm::dot(contactData1.rBPerp, impulse1)) * invInertiaB;
				}

				// Friction.
				if (SimulationSettings::FRICTION_SOLVER_TYPE == SimulationSettings::FrictionSolverType::Block)
				{
					const auto& contactData0 = velocityConstraintData.points[0];
					const auto& contactData1 = velocityConstraintData.points[1];

					const Vec2 relativeVelocity0 =
						(linearVelocityB + contactData0.rBPerp * angularVelocityB) -
						(linearVelocityA + contactData0.rAPerp * angularVelocityA);
					const Vec2 relativeVelocity1 =
						(linearVelocityB + contactData1.rBPerp * angularVelocityB) -
						(linearVelocityA + contactData1.rAPerp * angularVelocityA);

					const Real velocityAlongTangent0 = glm::dot(relativeVelocity0, tangent);
					const Real velocityAlongTangent1 = glm::dot(relativeVelocity1, tangent);

					Real& accumulatedImpulse0 = collisionData.persistentContactData[0].tangentImpulseAccumulator;
					Real& accumulatedImpulse1 = collisionData.persistentContactData[1].tangentImpulseAccumulator;

					const Real rtA0 = glm::dot(contactData0.rAPerp, tangent);
					const Real rtB0 = glm::dot(contactData0.rBPerp, tangent);
					const Real rtA1 = glm::dot(contactData1.rAPerp, tangent);
					const Real rtB1 = glm::dot(contactData1.rBPerp, tangent);

					const Real K00 = effectiveMass + rtA0 * rtA0 * invInertiaA + rtB0 * rtB0 * invInertiaB;
					const Real K11 = effectiveMass + rtA1 * rtA1 * invInertiaA + rtB1 * rtB1 * invInertiaB;
					const Real K01 = effectiveMass + rtA0 * rtA1 * invInertiaA + rtB0 * rtB1 * invInertiaB;

					const Real bPrime0 = -velocityAlongTangent0 + K00 * accumulatedImpulse0 + K01 * accumulatedImpulse1;
					const Real bPrime1 = -velocityAlongTangent1 + K01 * accumulatedImpulse0 + K11 * accumulatedImpulse1;

					const Real jn0 = jnArray[0];
					const Real jn1 = jnArray[1];
					const Real limit0 = jn0 * staticFriction;  // Free (sticking) acceptance.
					const Real limit1 = jn1 * staticFriction;
					const Real max0 = jn0 * dynamicFriction;   // Saturation value.
					const Real max1 = jn1 * dynamicFriction;

					Real x0 = 0, x1 = 0;

					auto solveTangentBlock = [&]()
					{
						constexpr Real ones[2] = { 1, -1 };

						const Real det = K00 * K11 - K01 * K01;
						const Real maxK = std::fmax(K00, K11);
						if (maxK * maxK < SimulationSettings::MAX_CONDITION_NUMBER * det)
						{
							const Real invDet = Real(1) / det;
							x0 = (K11 * bPrime0 - K01 * bPrime1) * invDet;
							x1 = (K00 * bPrime1 - K01 * bPrime0) * invDet;
							if (std::fabs(x0) <= limit0 && std::fabs(x1) <= limit1) return;
						}
						for (Real s : ones)
						{
							x0 = s * max0;
							x1 = (bPrime1 - K01 * x0) / K11;
							const Real w0 = K00 * x0 + K01 * x1 - bPrime0;
							if (std::fabs(x1) <= limit1 && s * w0 <= 0) return;
						}
						for (Real s : ones)
						{
							x1 = s * max1;
							x0 = (bPrime0 - K01 * x1) / K00;
							const Real w1 = K01 * x0 + K11 * x1 - bPrime1;
							if (std::fabs(x0) <= limit0 && s * w1 <= 0) return;
						}
						for (Real s0 : ones)
						{
							for (Real s1 : ones)
							{
								x0 = s0 * max0;
								x1 = s1 * max1;
								const Real w0 = K00 * x0 + K01 * x1 - bPrime0;
								const Real w1 = K01 * x0 + K11 * x1 - bPrime1;
								if (s0 * w0 <= 0 && s1 * w1 <= 0) return;
							}
						}
						// Unreachable for a valid LCP; keep zero.
						x0 = 0;
						x1 = 0;
					};

					solveTangentBlock();

					const Real deltaJt0 = x0 - accumulatedImpulse0;
					const Real deltaJt1 = x1 - accumulatedImpulse1;

					accumulatedImpulse0 = x0;
					accumulatedImpulse1 = x1;

					const Vec2 impulse0 = deltaJt0 * tangent;
					const Vec2 impulse1 = deltaJt1 * tangent;
					const Vec2 impulseSum = impulse0 + impulse1;

					linearVelocityA -= impulseSum * invMassA;
					linearVelocityB += impulseSum * invMassB;

					angularVelocityA -= (glm::dot(contactData0.rAPerp, impulse0) + glm::dot(contactData1.rAPerp, impulse1)) * invInertiaA;
					angularVelocityB += (glm::dot(contactData0.rBPerp, impulse0) + glm::dot(contactData1.rBPerp, impulse1)) * invInertiaB;
				}
				else
				{
					for (uint32_t i = 0; i < 2; i++)
					{
						const auto& contactData = velocityConstraintData.points[i];
						const Vec2 rAPerp = contactData.rAPerp;
						const Vec2 rBPerp = contactData.rBPerp;

						const Vec2 relativeVelocity =
							(linearVelocityB + rBPerp * angularVelocityB) -
							(linearVelocityA + rAPerp * angularVelocityA);

						const Real currentSlipVel = glm::dot(relativeVelocity, tangent);
						const Real jt = -currentSlipVel * contactData.tangentMass;

						Real& accumulatedJt = collisionData.persistentContactData[i].tangentImpulseAccumulator;
						const Real oldJt = accumulatedJt;
						Real targetJt = oldJt + jt;
						const Real jn = jnArray[i];

						if (std::fabs(targetJt) <= jn * staticFriction)
						{
							accumulatedJt = targetJt;
						}
						else
						{
							const Real maxDynamic = jn * dynamicFriction;
							accumulatedJt = std::clamp(targetJt, -maxDynamic, maxDynamic);
						}

						const Real deltaJt = accumulatedJt - oldJt;
						const Vec2 impulse = deltaJt * tangent;

						linearVelocityA -= impulse * invMassA;
						angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;
						linearVelocityB += impulse * invMassB;
						angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
					}
				}
			}

			// Store results.
			if (invMassA > 0) [[likely]]
			{
				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;
				angularVelocityPtr[bodyIndexA] = angularVelocityA;
			}
			if (invMassB > 0) [[likely]]
			{
				velocityXPtr[bodyIndexB] = linearVelocityB.x;
				velocityYPtr[bodyIndexB] = linearVelocityB.y;
				angularVelocityPtr[bodyIndexB] = angularVelocityB;
			}
		}
	}

	void BodyCollisionSolver::solvePositionConstraints(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Solve collision position constraints", Ecstasy::Core::Color::Indigo);

		std::span<const BodyCollisionData> collisionDataSpan = activeCollisionData.subspan(pass.start, pass.size);
		std::span<const PositionConstraintData> constraintDataSpan(positionConstraintContainer.data() + pass.start, pass.size);

		Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();

		const size_t collisionCount = collisionDataSpan.size();

		// Note: I tried to use Simd, it was slower, probably because of the gather-scatter, or just memory intensive.

		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = collisionDataSpan[c];
			const ObjectIndex bodyIndexA = data.bodyA;
			const ObjectIndex bodyIndexB = data.bodyB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real effectiveMass = invMassA + invMassB;

			// Re-derive the world anchors from the current (possibly already corrected) transforms.
			const Vec2 positionA = { positionXPtr[bodyIndexA], positionYPtr[bodyIndexA] };
			const Vec2 positionB = { positionXPtr[bodyIndexB], positionYPtr[bodyIndexB] };

			const PositionConstraintData& positionConstraintData = constraintDataSpan[c];
			const Vec2 worldAnchorA = positionA + positionConstraintData.transformedAnchorA;
			const Vec2 worldAnchorB = positionB + positionConstraintData.transformedAnchorB;

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
		const size_t workerCount = solvingPlan.getWorkerCount();

		const auto& passOffsets = solvingPlan.getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0)) [[unlikely]]
		{
			return;
		}

		const uint32_t positionSolvingStartTick = static_cast<uint32_t>(waveCount) * velocityIterations;
		const uint32_t totalTicks = positionSolvingStartTick + static_cast<uint32_t>(waveCount) * positionIterations;

		runThreadedWaves(workerCount, waveCount, static_cast<uint32_t>(waveCount),
			[&](size_t waveIndex, size_t workerIndex, uint32_t passTicket)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];
				if (passOffset.size == 0) return;

				applyWarmStarting(passOffset);
			}
		);

		runThreadedWaves(workerCount, waveCount, totalTicks,
			[&](size_t waveIndex, size_t workerIndex, uint32_t passTicket)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];

				// Get work from current wave and execute it. If empty, skip.
				if (passOffset.size == 0) [[unlikely]] return;

				if (passTicket >= positionSolvingStartTick)
				{
					solvePositionConstraints(passOffset);
				}
				else
				{
					solveVelocityConstraints(passOffset);
				}
			}
		);
	}
}