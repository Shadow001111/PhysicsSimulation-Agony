#include "RodSolver.h"
#include "../Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
	void RodSolver::setDataViewers(
		BodySoA& bodiesIn,
		const RodSoAViewer& rodsIn
	)
	{
		setResources(bodiesIn);
		rods = rodsIn;
	}

	size_t RodSolver::getMemoryUsage() const
	{
		size_t total = 0;

		total += getVectorMemoryUsage(rodBodyPairs);
		total += getVectorMemoryUsage(orderedRodIndices);
		total += getVectorMemoryUsage(rodConstraintContainer);

		total += solvingPlanner.getMemoryUsage();

		return total;
	}

	void RodSolver::solveRods(
		Real deltaTime,
		uint32_t rodIterations,
		bool rodsWereChanged
	)
	{
		const size_t rodCount = rods.getCount();
		if (rodCount == 0) return;

		const size_t workerCount = planWorkerCount(rodCount);

		// Single-threaded path.
		if (workerCount <= 1)
		{
			TRACY_SCOPE_NC("Solve rods (Single-threaded)", Ecstasy::Core::Color::OliveDrab);

			if (orderedRodIndices.size() != rodCount)
			{
				orderedRodIndices.resize(rodCount);
				for (size_t i = 0; i < rodCount; i++)
				{
					orderedRodIndices[i] = i;
				}
			}

			computeConstraintData(deltaTime, orderedRodIndices);

			const SolvingPlanner::Pass fullPass{ 0, static_cast<uint32_t>(rodCount) };

			for (uint32_t i = 0; i < rodIterations; i++)
			{
				solveVelocityConstraints(fullPass);
			}
			return;
		}

		// Multi-threading path.
		TRACY_SCOPE_NC("Solve rods (Multi-threaded)", Ecstasy::Core::Color::OliveDrab);
		if (rodsWereChanged || solvingPlanner.getFlatIndices().size() != rodCount)
		{
			{
				TRACY_SCOPE_NC("Collect body pairs from rods", Ecstasy::Core::Color::Red);
				rodBodyPairs.resize(rodCount);
				for (size_t i = 0; i < rodCount; i++)
				{
					rodBodyPairs[i] = { rods.bodyIndexA[i], rods.bodyIndexB[i] };
				}
			}
			{
				TRACY_SCOPE_NC("Plan execution", Ecstasy::Core::Color::Blue);
				solvingPlanner.setWorkerCount(workerCount);

				solvingPlanner.planCacheLineAwareExecution(
					rodBodyPairs, bodies->getCount(),
					sizeof(Real),
					std::hardware_destructive_interference_size
				);
			}
		}
		{
			TRACY_SCOPE_N("Reorder rod indices");

			const auto& flatIndices = solvingPlanner.getFlatIndices();
			orderedRodIndices.assign(flatIndices.begin(), flatIndices.end());
		}
		computeConstraintData(deltaTime, orderedRodIndices);

		solveConstraintsThreaded(rodIterations);
	}

	size_t RodSolver::planWorkerCount(size_t rodCount) const
	{
		// Note: placeholder thresholds copied in shape from SpringSolver, not in value -
		// rod's per-constraint cost/count profile hasn't been measured yet. Re-profile
		// before relying on these once rods are used at scale.
		static constexpr size_t ROD_COUNT_PER_WORKER = 7'000;
		static constexpr size_t MIN_ROD_COUNT_FOR_THREADING = ROD_COUNT_PER_WORKER * 3;

		// Single-threaded path.
		if (rodCount < MIN_ROD_COUNT_FOR_THREADING)
		{
			return 0;
		}

		// Multi-threading path.
		auto& threadPool = Threading::getGlobalThreadPool();

		const size_t availableWorkerCount = threadPool.getThreadCount();
		const size_t neededWorkerCount = rodCount / ROD_COUNT_PER_WORKER;

		return std::min(availableWorkerCount, neededWorkerCount);
	}

	void RodSolver::computeConstraintData(Real deltaTime, std::span<const size_t> rodIndices)
	{
		TRACY_SCOPE_NC("Compute rod constraint data", Ecstasy::Core::Color::Chocolate);

		// Baumgarte stabilization factor for the rigid distance constraint.
		// Note: placeholder value - not yet tuned/profiled for stability vs. stiffness at
		// this engine's iteration counts and typical deltaTime.
		static constexpr Real BAUMGARTE_FACTOR = Real(1.0);

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

		const Real* ECSTASY_RESTRICT rodLengthPtr = rods.length;

		// Resize container.
		const size_t rodCount = rodIndices.size();
		rodConstraintContainer.resize(rodCount);

		const size_t* ECSTASY_RESTRICT rodIndicesPtr = rodIndices.data();

		const Real invDeltaTime = (deltaTime > Real(0)) ? Real(1) / deltaTime : Real(0);

		// Compute constant data.
		for (size_t rodIndex = 0; rodIndex < rodCount; rodIndex++)
		{
			const size_t i = rodIndicesPtr[rodIndex];
			RodConstraintData& constraintData = rodConstraintContainer[rodIndex];

			const ObjectIndex bodyIndexA = rods.bodyIndexA[i];
			const ObjectIndex bodyIndexB = rods.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;

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

			const Vec2 rotatedAnchorA = rotate(rods.localAnchorA[i], rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 rotatedAnchorB = rotate(rods.localAnchorB[i], rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

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
			const Real C = currentLength - rodLengthPtr[i];

			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			const Real raCn = (rotatedAnchorA.x * dir.y) - (rotatedAnchorA.y * dir.x);
			const Real rbCn = (rotatedAnchorB.x * dir.y) - (rotatedAnchorB.y * dir.x);

			// Compute effective mass. No compliance term (rod is rigid, unlike a spring).
			const Real effectiveMass = invMassA + invMassB + invInertiaA * raCn * raCn + invInertiaB * rbCn * rbCn;
			if (effectiveMass <= Real(0)) [[unlikely]]
			{
				constraintData.invEffectiveMass = Real(0);
				continue;
			}

			constraintData.dir = dir;
			constraintData.rotatedAnchorA = rotatedAnchorA;
			constraintData.rotatedAnchorB = rotatedAnchorB;
			constraintData.invEffectiveMass = Real(1) / effectiveMass;
			constraintData.bias = BAUMGARTE_FACTOR * invDeltaTime * C;
		}
	}

	void RodSolver::solveVelocityConstraints(SolvingPlanner::Pass pass)
	{
		TRACY_SCOPE_NC("Solve rod velocity constraints", Ecstasy::Core::Color::Violet);

		std::span<const RodConstraintData> constraintDataSpan(rodConstraintContainer.data() + pass.start, pass.size);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t rodCount = constraintDataSpan.size();

		for (size_t c = 0; c < rodCount; c++)
		{
			const RodConstraintData& constraintData = constraintDataSpan[c];
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

	void RodSolver::solveConstraintsThreaded(uint32_t rodIterations)
	{
		const size_t workerCount = solvingPlanner.getWorkerCount();

		const auto& passOffsets = solvingPlanner.getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || rodIterations == 0) [[unlikely]]
		{
			return;
		}

		const uint32_t totalTicks = static_cast<uint32_t>(waveCount) * rodIterations;

		runThreadedWaves(workerCount, waveCount, totalTicks,
			[&](size_t waveIndex, size_t workerIndex, uint32_t /*passTicket*/)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];

				// Get work from current wave and execute it. If empty, skip.
				if (passOffset.size == 0) return;

				solveVelocityConstraints(passOffset);
			}
		);
	}
}