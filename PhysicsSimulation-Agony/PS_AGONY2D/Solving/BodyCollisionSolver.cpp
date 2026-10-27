#include "BodyCollisionSolver.h"

#include "PS_AGONY2D/Threading.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>

namespace PS_AGONY2D
{
	void BodyCollisionSolver::setDataViewers(
		BodySoA& bodiesIn,
		const ColliderSoAViewer& collidersIn,
		const Material* materials
	)
	{
		SolverBase::setResources(bodiesIn);
		colliders = collidersIn;
		this->materials = materials;
	}

	void BodyCollisionSolver::solveCollisions(
		std::span<BodyCollisionData> collisionData,
		uint32_t velocityIterations,
		uint32_t positionIterations
	)
	{
		TRACY_SCOPE_NC("Solve collisions", PS_AGONY2D::Profiling::Color::OliveDrab);

		const size_t collisionCount = collisionData.size();
		const size_t workerCount = planWorkerCount(collisionCount);

		collisionEvents.clear();

		// Single-threaded path.
		if (workerCount <= 1)
		{
			activeCollisionCount = collisionCount;
			computeConstraintData(collisionData, nullptr);

			{
				TRACY_SCOPE_N("Solve collision constraints (Single-threaded)");

				const SolvingPlan::Pass fullPass{ 0, static_cast<uint32_t>(collisionCount) };

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

			scatterImpulses(collisionData, nullptr);
			return;
		}

		// Multi-threading path.
		{
			TRACY_SCOPE_NC("Collect body pairs from collision data", PS_AGONY2D::Profiling::Color::Red);

			collidingBodyPairs.resize(collisionCount);
			for (size_t i = 0; i < collisionCount; i++)
			{
				const BodyCollisionData& collData = collisionData[i];
				collidingBodyPairs[i] = { collData.bodyA, collData.bodyB };
			}
		}
		{
			TRACY_SCOPE_NC("Plan execution", PS_AGONY2D::Profiling::Color::Blue);

			//SolvingPlanner::threadLocal().planStandardExecution(
			//	solvingPlan,
			//	collidingBodyPairs,
			//	bodies->getCount(),
			//	bodies->isStatic.data(),
			//	false,
			//	workerCount
			//);

			// Rating: Sometimes results in faster execution, but I think it may have huge worst-case scenario dependent on body ordering.
			SolvingPlanner::threadLocal().planCacheLineAwareExecution(
				solvingPlan,
				collidingBodyPairs, bodies->getCount(),
				bodies->isStatic.data(), false,
				workerCount,
				sizeof(Real)
			);
			
			// Rating: Worse than upper method in planning time and speeding up solving.
			//SolvingPlanner::threadLocal().planCacheSharingAwareExecution(
			//	solvingPlan,
			//	collidingBodyPairs, bodies->getCount(),
			//	bodies->isStatic.data(), false,
			//	workerCount,
			//	sizeof(Real)
			//);
		}

		// Constraint data is gathered straight through the plan's indices, so no separate reorder copy of the full data is needed.
		const auto& flatIndices = solvingPlan.getFlatIndices();
		const size_t* flatIndicesPtr = flatIndices.data();

		activeCollisionCount = collisionCount;
		computeConstraintData(collisionData, flatIndicesPtr);

		{
			TRACY_SCOPE_N("Solve collision constraints (Multi-threaded)");
			solveConstraintsThreaded(velocityIterations, positionIterations);
		}

		scatterImpulses(collisionData, flatIndicesPtr);
	}

	void BodyCollisionSolver::reportNoCollisions()
	{
		activeCollisionCount = 0;
		collisionEvents.clear();
	}

	void BodyCollisionSolver::setCollisionEventRecording(bool enabled)
	{
		recordCollisionEvents = enabled;
		if (enabled && collisionEvents.capacity() == 0)
			collisionEvents.reserve(128);
	}

	Vec2 BodyCollisionSolver::computeConstraintErrors() const
	{
		TRACY_SCOPE_NC("Compute (body collision) constraint errors", PS_AGONY2D::Profiling::Color::Gray);

		const size_t collisionCount = activeCollisionCount;
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
			const VelocityHotData& data = velocityHotContainer[c];
			const ObjectDenseIndex bodyIndexA = data.bodyA;
			const ObjectDenseIndex bodyIndexB = data.bodyB;

			const Vec2 positionA = { positionXPtr[bodyIndexA], positionYPtr[bodyIndexA] };
			const Vec2 positionB = { positionXPtr[bodyIndexB], positionYPtr[bodyIndexB] };

			const Real currentPenetration = positionHotContainer[c].penetrationBase - glm::dot(positionB - positionA, data.normal);

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

		total += getVectorMemoryUsage(velocityHotContainer);
		total += getVectorMemoryUsage(positionHotContainer);
		total += getVectorMemoryUsage(velocityConstraintContainer);
		total += getVectorMemoryUsage(frictionDataContainer);

		total += getVectorMemoryUsage(collidingBodyPairs);
		total += getVectorMemoryUsage(collisionEvents);

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
		auto& threadingBackend = Threading::getThreadingBackend();

		const size_t availableWorkerCount = threadingBackend.getThreadCount() + 1; // Include main thread.
		const size_t neededWorkerCount = collisionCount / COLLISION_COUNT_PER_WORKER;

		return std::min(availableWorkerCount, neededWorkerCount);
	}

	void BodyCollisionSolver::computeConstraintData(std::span<BodyCollisionData> source, const size_t* order)
	{
		TRACY_SCOPE_NC("Compute constraint data", PS_AGONY2D::Profiling::Color::Chocolate);

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
		const CollisionCallbackIndex* ECSTASY_RESTRICT colliderCallbackIdPtr = colliders.callbackId;
		const Real* ECSTASY_RESTRICT colliderEventThresholdPtr = colliders.eventThreshold;
		const bool recordEvents = recordCollisionEvents;

		const size_t collisionCount = source.size();

		// Resize containers.
		velocityHotContainer.resize(collisionCount);
		positionHotContainer.resize(collisionCount);
		velocityConstraintContainer.resize(collisionCount);
		frictionDataContainer.resize(collisionCount);

		const BodyCollisionData* ECSTASY_RESTRICT sourcePtr = source.data();

		// Compute constant data.
		// No need to zero-initialize point data. Active points will have their data overwritten.
		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = sourcePtr[order ? order[c] : c];

			const Vec2 positionA = { positionXPtr[data.bodyA], positionYPtr[data.bodyA] };
			const Vec2 positionB = { positionXPtr[data.bodyB], positionYPtr[data.bodyB] };

			const Vec2 localCOMA = { localCenterOfMassXPtr[data.bodyA], localCenterOfMassYPtr[data.bodyA] };
			const Vec2 localCOMB = { localCenterOfMassXPtr[data.bodyB], localCenterOfMassYPtr[data.bodyB] };

			const Vec2 centerOfMassA = positionA + localCOMA;
			const Vec2 centerOfMassB = positionB + localCOMB;

			// Anchors cancel out (translation-only drift), so only the initial relative position along the normal matters.
			{
				VelocityHotData& velocityHot = velocityHotContainer[c];
				velocityHot.bodyA = data.bodyA;
				velocityHot.bodyB = data.bodyB;
				velocityHot.normal = data.normal;
				velocityHot.contactCount = data.contactCount;
				velocityHot.impulses = data.persistentContactData;
			}
			{
				PositionHotData& positionHot = positionHotContainer[c];
				positionHot.bodyA = data.bodyA;
				positionHot.bodyB = data.bodyB;
				positionHot.normal = data.normal;
				positionHot.penetrationBase = data.penetration + glm::dot(positionB - positionA, data.normal) - simulationSettings.positionCorrectionSlop;
			}
			{
				auto& velocityConstraint = velocityConstraintContainer[c];
				auto& frictionData = frictionDataContainer[c];

				const MaterialIndex materialIndexA = colliderMaterialIndexPtr[data.colliderA];
				const MaterialIndex materialIndexB = colliderMaterialIndexPtr[data.colliderB];
				const Material* materialA = materials + materialIndexA;
				const Material* materialB = materials + materialIndexB;

				const Real elasticity = (materialA->elasticity + materialB->elasticity) * Real(0.5);
				frictionData.staticFriction = std::sqrt(std::fmax(Real(0), materialA->staticFriction * materialB->staticFriction));
				frictionData.dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

				const Real invMassA = invMassPtr[data.bodyA];
				const Real invMassB = invMassPtr[data.bodyB];
				const Real invInertiaA = invInertiaPtr[data.bodyA];
				const Real invInertiaB = invInertiaPtr[data.bodyB];
				const Real effectiveMass = invMassA + invMassB;
				Real impactImpulse = Real(0);

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

					if (recordEvents)
						impactImpulse = std::fmax(impactImpulse, negVelocityAlongNormal * point.normalMass);
				}

				if (impactImpulse > Real(0)) // && recordEvents
				{
					const CollisionCallbackIndex callbackA = colliderCallbackIdPtr[data.colliderA];
					const CollisionCallbackIndex callbackB = colliderCallbackIdPtr[data.colliderB];

					const bool fireA = callbackA != 0 && impactImpulse >= colliderEventThresholdPtr[data.colliderA];
					const bool fireB = callbackB != 0 && impactImpulse >= colliderEventThresholdPtr[data.colliderB];

					if (fireA || fireB)
					{
						const Vec2 eventPoint = data.contactCount > 1
							? (data.contactPoints[0] + data.contactPoints[1]) * Real(0.5)
							: data.contactPoints[0];

						if (fireA)
							collisionEvents.push_back({ data.colliderA, data.colliderB, data.bodyA, data.bodyB, eventPoint, data.normal, impactImpulse, callbackA });
						if (fireB)
							collisionEvents.push_back({ data.colliderB, data.colliderA, data.bodyB, data.bodyA, eventPoint, -data.normal, impactImpulse, callbackB });
					}
				}
			}
		}
	}

	void BodyCollisionSolver::scatterImpulses(std::span<BodyCollisionData> target, const size_t* order) const
	{
		TRACY_SCOPE_N("Scatter persistent contact data back");

		const VelocityHotData* ECSTASY_RESTRICT hotPtr = velocityHotContainer.data();
		BodyCollisionData* ECSTASY_RESTRICT targetPtr = target.data();

		const size_t collisionCount = target.size();
		if (order)
		{
			for (size_t i = 0; i < collisionCount; i++)
			{
				targetPtr[order[i]].persistentContactData = hotPtr[i].impulses;
			}
		}
		else
		{
			for (size_t i = 0; i < collisionCount; i++)
			{
				targetPtr[i].persistentContactData = hotPtr[i].impulses;
			}
		}
	}

	void BodyCollisionSolver::applyWarmStarting(SolvingPlan::Pass pass)
	{
		TRACY_SCOPE_NC("Apply warm starting (body collisions)", PS_AGONY2D::Profiling::Color::Violet);

		std::span<const VelocityHotData> collisionDataSpan(velocityHotContainer.data() + pass.start, pass.size);
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
			const VelocityHotData& collisionData = collisionDataSpan[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataSpan[c];

			// Get body indices.
			const ObjectDenseIndex bodyIndexA = collisionData.bodyA;
			const ObjectDenseIndex bodyIndexB = collisionData.bodyB;

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
				Real oldJn = collisionData.impulses[i].normalImpulseAccumulator;
				Real oldJt = collisionData.impulses[i].tangentImpulseAccumulator;

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
		TRACY_SCOPE_NC("Solve collision velocity constraints", PS_AGONY2D::Profiling::Color::Violet);

		std::span<VelocityHotData> collisionDataSpan(velocityHotContainer.data() + pass.start, pass.size);
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
			VelocityHotData& collisionData = collisionDataSpan[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataSpan[c];

			const ObjectDenseIndex bodyIndexA = collisionData.bodyA;
			const ObjectDenseIndex bodyIndexB = collisionData.bodyB;

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

					Real& accumulatedJn = collisionData.impulses[0].normalImpulseAccumulator;
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

					Real& accumulatedJt = collisionData.impulses[0].tangentImpulseAccumulator;
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
				std::array<Real, 2> jnArray{};

				// Normal.
				auto solveNormalIterative = [&]()
					{
						for (uint32_t i = 0; i < 2; i++)
						{
							const auto& contactData = velocityConstraintData.points[i];
							const Vec2 rAPerp = contactData.rAPerp;
							const Vec2 rBPerp = contactData.rBPerp;

							const Vec2 relativeVelocity =
								(linearVelocityB + rBPerp * angularVelocityB) -
								(linearVelocityA + rAPerp * angularVelocityA);

							const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);
							const Real jn = (contactData.velocityBias - velocityAlongNormal) * contactData.normalMass;

							Real& accumulatedJn = collisionData.impulses[i].normalImpulseAccumulator;
							const Real oldJn = accumulatedJn;
							accumulatedJn = std::fmax(Real(0), oldJn + jn);
							const Real deltaJn = accumulatedJn - oldJn;

							jnArray[i] = accumulatedJn;

							const Vec2 impulse = deltaJn * normal;

							linearVelocityA -= impulse * invMassA;
							angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;
							linearVelocityB += impulse * invMassB;
							angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
						}
					};

				auto solveNormalBlock = [&]()
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

						Real& accumulatedImpulse0 = collisionData.impulses[0].normalImpulseAccumulator;
						Real& accumulatedImpulse1 = collisionData.impulses[1].normalImpulseAccumulator;

						// TODO: K00/K01/K11 (and det/invDet) are constant per manifold; precompute in computeConstraintData.
						// Tried once, no significant gain, so left as is.
						const Real rnA0 = glm::dot(contactData0.rAPerp, normal);
						const Real rnB0 = glm::dot(contactData0.rBPerp, normal);
						const Real rnA1 = glm::dot(contactData1.rAPerp, normal);
						const Real rnB1 = glm::dot(contactData1.rBPerp, normal);

						const Real K00 = effectiveMass + rnA0 * rnA0 * invInertiaA + rnB0 * rnB0 * invInertiaB;
						const Real K11 = effectiveMass + rnA1 * rnA1 * invInertiaA + rnB1 * rnB1 * invInertiaB;
						const Real K01 = effectiveMass + rnA0 * rnA1 * invInertiaA + rnB0 * rnB1 * invInertiaB;

						const Real bPrime0 = contactData0.velocityBias - velocityAlongNormal0 + K00 * accumulatedImpulse0 + K01 * accumulatedImpulse1;
						const Real bPrime1 = contactData1.velocityBias - velocityAlongNormal1 + K01 * accumulatedImpulse0 + K11 * accumulatedImpulse1;

						Real x0 = 0, x1 = 0;

						// Solves the 2x2 normal LCP:  x >= 0,  w = Kx - b' >= 0,  x.w = 0.
						// Returns true if one of the four complementarity cases gave a valid solution.
						// Returns false only for an ill-conditioned K or rounding at a case boundary.
						auto trySolveBlock = [&]() -> bool
							{
								constexpr Real EPS = Real(1e-6);

								// Case 1: both points active. x = K^-1 * b'.
								const Real det = K00 * K11 - K01 * K01;
								const Real maxK = std::fmax(K00, K11);
								if (maxK * maxK < SimulationSettings::MAX_CONDITION_NUMBER * det) // Implies det > 0.
								{
									const Real invDet = Real(1) / det;
									const Real s0 = (K11 * bPrime0 - K01 * bPrime1) * invDet;
									const Real s1 = (K00 * bPrime1 - K01 * bPrime0) * invDet;
									if (s0 >= -EPS && s1 >= -EPS)
									{
										x0 = std::fmax(s0, Real(0));
										x1 = std::fmax(s1, Real(0));
										return true;
									}
								}

								// Case 2: only point 1 active. Point 0 needs w0 = K01 * x1 - b'0 >= 0.
								if (K11 > 0)
								{
									const Real s1 = bPrime1 / K11;
									if (s1 >= -EPS && (K01 * s1 - bPrime0) >= -EPS)
									{
										x0 = 0;
										x1 = std::fmax(s1, Real(0));
										return true;
									}
								}

								// Case 3: only point 0 active. Point 1 needs w1 = K01 * x0 - b'1 >= 0.
								if (K00 > 0)
								{
									const Real s0 = bPrime0 / K00;
									if (s0 >= -EPS && (K01 * s0 - bPrime1) >= -EPS)
									{
										x0 = std::fmax(s0, Real(0));
										x1 = 0;
										return true;
									}
								}

								// Case 4: both points inactive. Valid when w = -b' >= 0 for both.
								if (bPrime0 <= EPS && bPrime1 <= EPS)
								{
									x0 = 0;
									x1 = 0;
									return true;
								}

								return false;
							};

						if (!trySolveBlock())
						{
							solveNormalIterative();
						}
						else
						{
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
					};

				// Friction.
				auto solveFrictionIterative = [&]()
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

							Real& accumulatedJt = collisionData.impulses[i].tangentImpulseAccumulator;
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
					};

				auto solveFrictionBlock = [&]()
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

						Real& accumulatedImpulse0 = collisionData.impulses[0].tangentImpulseAccumulator;
						Real& accumulatedImpulse1 = collisionData.impulses[1].tangentImpulseAccumulator;

						// TODO: K00/K01/K11 (and det/invDet) are constant per manifold; precompute in computeConstraintData.
						// Tried once, no significant gain, so left as is.
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

						// Solves the 2x2 friction box-LCP:  x_i in [-max_i, max_i],
						// where a point is either free (|x_i| <= limit_i, w_i = 0) or saturated (x_i = s * max_i, s * w_i <= 0),
						// with w = Kx - b'.
						// Returns true if one of the complementarity cases gave a valid solution.
						// Returns false only for an ill-conditioned K or rounding at a case boundary.
						auto trySolveBlock = [&]() -> bool
							{
								constexpr Real EPS = Real(1e-6);
								constexpr Real signs[2] = { 1, -1 };

								// Case 1: both points free (sticking). x = K^-1 * b'.
								const Real det = K00 * K11 - K01 * K01;
								const Real maxK = std::fmax(K00, K11);
								if (maxK * maxK < SimulationSettings::MAX_CONDITION_NUMBER * det) // Implies det > 0.
								{
									const Real invDet = Real(1) / det;
									const Real s0 = (K11 * bPrime0 - K01 * bPrime1) * invDet;
									const Real s1 = (K00 * bPrime1 - K01 * bPrime0) * invDet;
									if (std::fabs(s0) <= limit0 + EPS && std::fabs(s1) <= limit1 + EPS)
									{
										x0 = std::clamp(s0, -limit0, limit0);
										x1 = std::clamp(s1, -limit1, limit1);
										return true;
									}
								}

								// Case 2: point 0 saturated at +-max0, point 1 free.
								// Point 0 needs s * w0 <= 0.
								if (K11 > 0)
								{
									for (const Real s : signs)
									{
										const Real t0 = s * max0;
										const Real t1 = (bPrime1 - K01 * t0) / K11;
										const Real w0 = K00 * t0 + K01 * t1 - bPrime0;
										if (std::fabs(t1) <= limit1 + EPS && s * w0 <= EPS)
										{
											x0 = t0;
											x1 = std::clamp(t1, -limit1, limit1);
											return true;
										}
									}
								}

								// Case 3: point 1 saturated at +-max1, point 0 free.
								// Point 1 needs s * w1 <= 0.
								if (K00 > 0)
								{
									for (const Real s : signs)
									{
										const Real t1 = s * max1;
										const Real t0 = (bPrime0 - K01 * t1) / K00;
										const Real w1 = K01 * t0 + K11 * t1 - bPrime1;
										if (std::fabs(t0) <= limit0 + EPS && s * w1 <= EPS)
										{
											x0 = std::clamp(t0, -limit0, limit0);
											x1 = t1;
											return true;
										}
									}
								}

								// Case 4: both points saturated. Each needs s_i * w_i <= 0.
								for (const Real s0 : signs)
								{
									for (const Real s1 : signs)
									{
										const Real t0 = s0 * max0;
										const Real t1 = s1 * max1;
										const Real w0 = K00 * t0 + K01 * t1 - bPrime0;
										const Real w1 = K01 * t0 + K11 * t1 - bPrime1;
										if (s0 * w0 <= EPS && s1 * w1 <= EPS)
										{
											x0 = t0;
											x1 = t1;
											return true;
										}
									}
								}

								return false;
							};

						if (!trySolveBlock())
						{
							solveFrictionIterative();
						}
						else
						{
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
					};

				if (SimulationSettings::NORMAL_SOLVER_TYPE == SimulationSettings::SolverType::Block)
				{
					solveNormalBlock();
				}
				else
				{
					solveNormalIterative();
				}

				if (SimulationSettings::FRICTION_SOLVER_TYPE == SimulationSettings::SolverType::Block)
				{
					solveFrictionBlock();
				}
				else
				{
					solveFrictionIterative();
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
		TRACY_SCOPE_NC("Solve collision position constraints", PS_AGONY2D::Profiling::Color::Indigo);

		std::span<const PositionHotData> collisionDataSpan(positionHotContainer.data() + pass.start, pass.size);

		Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();

		const size_t collisionCount = collisionDataSpan.size();

		// Note: I tried to use Simd, it was slower, probably because of the gather-scatter, or just memory intensive.

		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = collisionDataSpan[c];
			const ObjectDenseIndex bodyIndexA = data.bodyA;
			const ObjectDenseIndex bodyIndexB = data.bodyB;

			const Vec2 positionA = { positionXPtr[bodyIndexA], positionYPtr[bodyIndexA] };
			const Vec2 positionB = { positionXPtr[bodyIndexB], positionYPtr[bodyIndexB] };

			// Penetration already resolved since measurement is the change of relative position along the normal.
			const Real currentPenetration = data.penetrationBase - glm::dot(positionB - positionA, data.normal);

			if (currentPenetration <= Real(0)) continue;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];

			const Real totalCorrection = currentPenetration / (invMassA + invMassB) * simulationSettings.positionCorrectionPercent;
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