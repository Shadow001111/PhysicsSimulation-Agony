#include "SpringSolver.h"
#include "../Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
	void SpringSolver::setDataViewers(
		BodySoA& bodiesIn,
		const SpringSoAViewer& springsIn,
		SolvingPlanner& solvingPlannerIn
	)
	{
		setSharedResources(bodiesIn, solvingPlannerIn);
		springs = springsIn;
	}

	size_t SpringSolver::getMemoryUsage() const
	{
		size_t total = sizeof(SpringSolver);

		total += getVectorMemoryUsage(springBodyPairs);
		total += getVectorMemoryUsage(orderedSpringIndices);
		total += getVectorMemoryUsage(springConstraintContainer);

		return total;
	}

	void SpringSolver::solveSprings(
		Real deltaTime,
		uint32_t springIterations,
		bool springsWereChanged
	)
	{
		const size_t springCount = springs.getCount();
		const size_t workerCount = planWorkerCount(springCount);

		// Single-threaded path.
		if (workerCount <= 1)
		{
			TRACY_SCOPE_NC("Solve springs (Single-threaded)", Ecstasy::Color::OliveDrab);

			orderedSpringIndices.resize(springCount);
			for (size_t i = 0; i < springCount; i++)
			{
				orderedSpringIndices[i] = i;
			}

			computeConstantData(deltaTime, orderedSpringIndices);

			for (uint32_t i = 0; i < springIterations; i++)
			{
				solveVelocityConstraints(orderedSpringIndices, springConstraintContainer);
			}
			return;
		}

		// Multi-threading path.
		TRACY_SCOPE_NC("Solve springs (Multi-threaded)", Ecstasy::Color::OliveDrab);
		{
			TRACY_SCOPE_NC("Collect body pairs from springs", Ecstasy::Color::Red);

			springBodyPairs.resize(springCount);
			for (size_t i = 0; i < springCount; i++)
			{
				springBodyPairs[i] = { springs.bodyIndexA[i], springs.bodyIndexB[i] };
			}
		}
		if (springsWereChanged || solvingPlanner->getFlatIndices().size() != springCount)
		{
			TRACY_SCOPE_NC("Plan execution", Ecstasy::Color::Blue);
			solvingPlanner->setWorkerCount(workerCount);
			solvingPlanner->planExecution(springBodyPairs, bodies->getCount());
		}
		{
			TRACY_SCOPE_N("Reorder spring indices");

			const auto& flatIndices = solvingPlanner->getFlatIndices();
			orderedSpringIndices.assign(flatIndices.begin(), flatIndices.end());
		}
		computeConstantData(deltaTime, orderedSpringIndices);

		solveConstraintsThreaded(springIterations);
	}

	size_t SpringSolver::planWorkerCount(size_t springCount) const
	{
		// Note: tuned for collisions; re-profile once springs are a bottleneck on their own.
		static constexpr size_t SPRING_COUNT_PER_WORKER = 830;
		static constexpr size_t MIN_SPRING_COUNT_FOR_THREADING = SPRING_COUNT_PER_WORKER * 3;

		// Single-threaded path.
		if (springCount < MIN_SPRING_COUNT_FOR_THREADING)
		{
			return 0;
		}

		// Multi-threading path.
		auto& threadPool = Threading::getGlobalThreadPool();

		const size_t availableWorkerCount = threadPool.getThreadCount();
		const size_t neededWorkerCount = springCount / SPRING_COUNT_PER_WORKER;

		return std::min(availableWorkerCount, neededWorkerCount);
	}

	void SpringSolver::computeConstantData(Real deltaTime, std::span<const size_t> springIndices)
	{
		TRACY_SCOPE_NC("Compute constant data", Ecstasy::Color::Chocolate);

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

		const Real* ECSTASY_RESTRICT springRestLengthPtr = springs.restLength;
		const Real* ECSTASY_RESTRICT springStiffnessPtr = springs.stiffness;
		const Real* ECSTASY_RESTRICT springDampingPtr = springs.damping;

		// Resize container.
		const size_t springCount = springIndices.size();
		springConstraintContainer.resize(springCount);

		const size_t* ECSTASY_RESTRICT springIndicesPtr = springIndices.data();

		// Compute constant data.
		for (size_t springIndex = 0; springIndex < springCount; springIndex++)
		{
			const size_t i = springIndicesPtr[springIndex];
			SpringConstraintData& constraintData = springConstraintContainer[springIndex];

			const BodyIndex bodyIndexA = springs.bodyIndexA[i];
			const BodyIndex bodyIndexB = springs.bodyIndexB[i];

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];

			// Both attachments are static anchors -> nothing to solve.
			if (invMassA == Real(0) && invMassB == Real(0)) [[unlikely]]
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

			const Vec2 rotatedAnchorA = rotate(springs.localAnchorA[i], rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 rotatedAnchorB = rotate(springs.localAnchorB[i], rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

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

	void SpringSolver::solveVelocityConstraints(
		std::span<const size_t> springIndices,
		std::span<const SpringConstraintData> constraintDataContainer
	)
	{
		TRACY_SCOPE_NC("Solve spring velocity constraints", Ecstasy::Color::Violet);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t springCount = springIndices.size();
		const size_t* ECSTASY_RESTRICT springIndicesPtr = springIndices.data();

		for (size_t c = 0; c < springCount; c++)
		{
			const SpringConstraintData& constraintData = constraintDataContainer[c];
			if (constraintData.invEffectiveMass == Real(0)) continue;

			const size_t i = springIndicesPtr[c];

			// Get body indices.
			const BodyIndex bodyIndexA = springs.bodyIndexA[i];
			const BodyIndex bodyIndexB = springs.bodyIndexB[i];

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
			if (invMassA > Real(0)) [[likely]]
			{
				linearVelocityA += impulse * invMassA;

				const Real torqueA = rotatedAnchorA.x * impulse.y - rotatedAnchorA.y * impulse.x;
				angularVelocityA += torqueA * invInertiaA;

				velocityXPtr[bodyIndexA] = linearVelocityA.x;
				velocityYPtr[bodyIndexA] = linearVelocityA.y;

				angularVelocityPtr[bodyIndexA] = angularVelocityA;
			}
			if (invMassB > Real(0)) [[likely]]
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

	void SpringSolver::solveConstraintsThreaded(uint32_t springIterations)
	{
		const size_t workerCount = solvingPlanner->getWorkerCount();

		const auto& passOffsets = solvingPlanner->getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || springIterations == 0) [[unlikely]]
		{
			return;
		}

		const uint32_t totalTicks = static_cast<uint32_t>(waveCount) * springIterations;

		const size_t* ECSTASY_RESTRICT orderedSpringIndicesPtr = orderedSpringIndices.data();

		runThreadedWaves(workerCount, waveCount, totalTicks,
			[&](size_t waveIndex, size_t workerIndex, uint32_t /*passTicket*/)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];

				// Get work from current wave and execute it. If empty, skip.
				if (passOffset.size == 0) return;

				std::span<const size_t> indexSlice(orderedSpringIndicesPtr + passOffset.start, passOffset.size);
				std::span<const SpringConstraintData> constraintSlice(springConstraintContainer.data() + passOffset.start, passOffset.size);

				solveVelocityConstraints(indexSlice, constraintSlice);
			}
		);
	}
}