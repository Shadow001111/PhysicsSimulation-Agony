#include "BodyCollisionSolver.h"
#include "../Threading.h"

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
			debugCollisionDataContainer = narrowPhaseCollisions;
			computeConstraintData(narrowPhaseCollisions);

			{
				TRACY_SCOPE_N("Solve collision constraints (Single-threaded)");

				applyWarmStarting(narrowPhaseCollisions, velocityConstraintContainer);

				for (uint32_t i = 0; i < velocityIterations; i++)
				{
					solveVelocityConstraints(narrowPhaseCollisions, velocityConstraintContainer, frictionDataContainer);
				}
				for (uint32_t i = 0; i < positionIterations; i++)
				{
					solvePositionConstraints(narrowPhaseCollisions, positionConstraintContainer);
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
			solvingPlanner.setWorkerCount(workerCount);

			solvingPlanner.planStandardExecution(collidingBodyPairs, bodies->getCount());
		}
		{
			TRACY_SCOPE_N("Reorder collision data");

			const auto& flatIndices = solvingPlanner.getFlatIndices();
			const size_t mappedCount = flatIndices.size();
			const size_t* ECSTASY_RESTRICT flatIndicesPtr = flatIndices.data();

			orderedCollisionData.resize(mappedCount);
			for (size_t i = 0; i < mappedCount; i++)
			{
				const size_t originalIndex = flatIndicesPtr[i];
				orderedCollisionData[i] = narrowPhaseCollisions[originalIndex];
			}
		}
		debugCollisionDataContainer = orderedCollisionData;
		computeConstraintData(orderedCollisionData);

		{
			TRACY_SCOPE_N("Solve collision constraints (Multi-threaded)");
			solveConstraintsThreaded(orderedCollisionData, velocityIterations, positionIterations);
		}

		// Scatter persistent contact data back to the detector's original container.
		if (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
		{
			TRACY_SCOPE_N("Scatter persistent contact data back");
			const auto& flatIndices = solvingPlanner.getFlatIndices();
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
		debugCollisionDataContainer = {};
	}

	Vec2 BodyCollisionSolver::computeConstraintErrors() const
	{
		TRACY_SCOPE_NC("Compute (body collision) constraint errors", Ecstasy::Core::Color::Gray);

		const size_t collisionCount = debugCollisionDataContainer.size();
		if (collisionCount == 0) return { Real(0), Real(0) };

		const Real* ECSTASY_RESTRICT positionXPtr = bodies->offsetX.data();
		const Real* ECSTASY_RESTRICT positionYPtr = bodies->offsetY.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies->localCenterOfMassX.data();
		const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies->localCenterOfMassY.data();
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		const Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();

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

		Real totalApproachSpeed = Real(0);
		Real totalPenetration = Real(0);

		for (size_t c = 0; c < collisionCount; c++)
		{
			const auto& data = debugCollisionDataContainer[c];
			const ObjectIndex bodyIndexA = data.bodyA;
			const ObjectIndex bodyIndexB = data.bodyB;

			// Compute centers of mass.
			const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
			const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

			// Position error.
			const PositionConstraintData& positionConstraintData = positionConstraintContainer[c];
			const Vec2 worldAnchorA = centerOfMassA + rotate(positionConstraintData.localAnchorA, rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 worldAnchorB = centerOfMassB + rotate(positionConstraintData.localAnchorB, rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

			// Compute drift.
			const Real drift = glm::dot(worldAnchorB - worldAnchorA, data.normal);
			const Real currentPenetration = data.penetration - drift;

			totalPenetration += currentPenetration * (currentPenetration > Real(0));

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

		total += solvingPlanner.getMemoryUsage();

		return total;
	}

	size_t BodyCollisionSolver::planWorkerCount(size_t collisionCount) const
	{
		// Disabled multi-threading.Because having two additional threads drops execution time to 83% only.
		// I think cache-sharing is happening between threads.
		// I have SolvingPlanner function to arrange them by cache lines, but it's heavily reliant on body indices ordering.
		// It struggles much if collision graph is dense and random. It also makes wave count explode.
		// Efficiency metric matters (I guess), but wave count matters more (I guess).
		return 0;

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
		const Real* ECSTASY_RESTRICT rotationCosPtr = bodies->rotationCos.data();
		const Real* ECSTASY_RESTRICT rotationSinPtr = bodies->rotationSin.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();
		const MaterialIndex* ECSTASY_RESTRICT colliderMaterialIndexPtr = colliders.materialIndex;
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
					point.normalMass = normalDenom > Real(0) ? Real(1) / normalDenom : Real(0);

					const Real rAPerpDotT = glm::dot(point.rAPerp, tangent);
					const Real rBPerpDotT = glm::dot(point.rBPerp, tangent);
					const Real tangentDenom = effectiveMass
						+ rAPerpDotT * rAPerpDotT * invInertiaA
						+ rBPerpDotT * rBPerpDotT * invInertiaB;
					point.tangentMass = tangentDenom > Real(0) ? Real(1) / tangentDenom : Real(0);

					const Vec2 linearVelocityA = { velocityXPtr[data.bodyA], velocityYPtr[data.bodyA] };
					const Vec2 linearVelocityB = { velocityXPtr[data.bodyB], velocityYPtr[data.bodyB] };
					const Real angularVelocityA = angularVelocityPtr[data.bodyA];
					const Real angularVelocityB = angularVelocityPtr[data.bodyB];
					const Vec2 relativeVelocity =
						(linearVelocityB + point.rBPerp * angularVelocityB) -
						(linearVelocityA + point.rAPerp * angularVelocityA);

					const Real negVelocityAlongNormal = -glm::dot(relativeVelocity, normal);

					point.velocityBias = (negVelocityAlongNormal > SimulationSettings::RESTITUTION_VELOCITY_THRESHOLD) ? elasticity * negVelocityAlongNormal : Real(0);
				}
			}
		}
	}

	void BodyCollisionSolver::applyWarmStarting(
		std::span<const BodyCollisionData> collisionDataContainer,
		std::span<const VelocityConstraintData> constraintDataContainer
	)
	{
		TRACY_SCOPE_NC("Apply warm starting (body collisions)", Ecstasy::Core::Color::Violet);

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

				impulseArray[i] = (oldJn * normal) + (oldJt * tangent);
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
		std::span<const BodyCollisionData> collisionDataContainer,
		std::span<const VelocityConstraintData> constraintDataContainer,
		std::span<const FrictionData> frictionDataContainer
	)
	{
		TRACY_SCOPE_NC("Solve collision velocity constraints", Ecstasy::Core::Color::Violet);

		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		const size_t collisionCount = collisionDataContainer.size();
		for (size_t c = 0; c < collisionCount; c++)
		{
			const BodyCollisionData& collisionData = collisionDataContainer[c];
			const VelocityConstraintData& velocityConstraintData = constraintDataContainer[c];

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

			std::array<Real, 2> jnArray{};

			if (contactCount == 1)
			{
				const auto& contactData = velocityConstraintData.points[0];
				const Vec2 rAPerp = contactData.rAPerp;
				const Vec2 rBPerp = contactData.rBPerp;

				const Vec2 relativeVelocity =
					(linearVelocityB + rBPerp * angularVelocityB) -
					(linearVelocityA + rAPerp * angularVelocityA);
				const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

				Real& accumulatedJn = collisionData.persistentContactData[0].normalImpulseAccumulator;
				if (!(velocityAlongNormal > Real(0) && accumulatedJn == Real(0)))
				{
					const Real jn = (contactData.velocityBias - velocityAlongNormal) * contactData.normalMass;
					const Real oldJn = accumulatedJn;
					accumulatedJn = std::fmax(Real(0), oldJn + jn);
					const Real deltaJn = accumulatedJn - oldJn;

					jnArray[0] = accumulatedJn;

					const Vec2 impulse = deltaJn * normal;
					linearVelocityA -= impulse * invMassA;
					angularVelocityA -= glm::dot(rAPerp, impulse) * invInertiaA;
					linearVelocityB += impulse * invMassB;
					angularVelocityB += glm::dot(rBPerp, impulse) * invInertiaB;
				}
			}
			else
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

				Real& a0 = collisionData.persistentContactData[0].normalImpulseAccumulator;
				Real& a1 = collisionData.persistentContactData[1].normalImpulseAccumulator;

				const Real effectiveMass = invMassA + invMassB;
				const Real rnA0 = glm::dot(contactData0.rAPerp, normal);
				const Real rnB0 = glm::dot(contactData0.rBPerp, normal);
				const Real rnA1 = glm::dot(contactData1.rAPerp, normal);
				const Real rnB1 = glm::dot(contactData1.rBPerp, normal);

				const Real K00 = effectiveMass + rnA0 * rnA0 * invInertiaA + rnB0 * rnB0 * invInertiaB;
				const Real K11 = effectiveMass + rnA1 * rnA1 * invInertiaA + rnB1 * rnB1 * invInertiaB;
				const Real K01 = effectiveMass + rnA0 * rnA1 * invInertiaA + rnB0 * rnB1 * invInertiaB;
				const Real K10 = K01;

				const Real b0 = contactData0.velocityBias - velocityAlongNormal0;
				const Real b1 = contactData1.velocityBias - velocityAlongNormal1;

				const Real bPrime0 = b0 + K00 * a0 + K01 * a1;
				const Real bPrime1 = b1 + K10 * a0 + K11 * a1;

				Real x0 = Real(0), x1 = Real(0);

				const Real det = K00 * K11 - K01 * K10;
				if (std::fabs(det) > Real(1e-6))
				{
					const Real invDet = Real(1.0) / det;
					x0 = (K11 * bPrime0 - K01 * bPrime1) * invDet;
					x1 = (K00 * bPrime1 - K10 * bPrime0) * invDet;
					if (x0 >= Real(0) && x1 >= Real(0)) goto solved;
				}
				{
					x0 = Real(0);
					x1 = K11 > Real(0) ? bPrime1 / K11 : Real(0);
					if (x1 >= Real(0) && (K01 * x1 - bPrime0) >= Real(0)) goto solved;
				}
				{
					x0 = K00 > Real(0) ? bPrime0 / K00 : Real(0);
					x1 = Real(0);
					if (x0 >= Real(0) && (K10 * x0 - bPrime1) >= Real(0)) goto solved;
				}
				{
					x0 = Real(0);
					x1 = Real(0);
					if (bPrime0 <= Real(0) && bPrime1 <= Real(0)) goto solved;
				}
			solved:
				const Real deltaJn0 = x0 - a0;
				const Real deltaJn1 = x1 - a1;

				a0 = x0;
				a1 = x1;

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

			const FrictionData& frictionData = frictionDataContainer[c];
			const Real staticFriction = frictionData.staticFriction;
			const Real dynamicFriction = frictionData.dynamicFriction;

			for (uint32_t i = 0; i < contactCount; i++)
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
		TRACY_SCOPE_NC("Solve collision position constraints", Ecstasy::Core::Color::Indigo);

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
		const size_t workerCount = solvingPlanner.getWorkerCount();

		const auto& passOffsets = solvingPlanner.getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || (velocityIterations == 0 && positionIterations == 0)) [[unlikely]]
		{
			return;
		}

		const uint32_t positionSolvingStartTick = static_cast<uint32_t>(waveCount) * velocityIterations;
		const uint32_t totalTicks = positionSolvingStartTick + static_cast<uint32_t>(waveCount) * positionIterations;

		const BodyCollisionData* ECSTASY_RESTRICT collisionDataPtr = collisionDataContainer.data();

		if constexpr (NarrowPhaseCollisionDetector::ENABLE_WARM_STARTING)
		{
			runThreadedWaves(workerCount, waveCount, static_cast<uint32_t>(waveCount),
				[&](size_t waveIndex, size_t workerIndex, uint32_t passTicket)
				{
					const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
					const auto& passOffset = passOffsetsPtr[passGlobalIndex];
					if (passOffset.size == 0) return;

					std::span<const BodyCollisionData> collisionSlice(collisionDataPtr + passOffset.start, passOffset.size);
					std::span<const VelocityConstraintData> constraintSlice(velocityConstraintContainer.data() + passOffset.start, passOffset.size);

					applyWarmStarting(collisionSlice, constraintSlice);
				}
			);
		}

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

					solveVelocityConstraints(collisionSlice, constraintSlice, frictionDataSlice);
				}
			}
		);
	}
}