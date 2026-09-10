#include "JointSolver.h"
#include "../Threading.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
	void JointSolver::setDataViewers(
		BodySoA& bodiesIn,
		const JointSoAViewer& jointsIn
	)
	{
		setResources(bodiesIn);
		joints = jointsIn;
	}

	size_t JointSolver::getMemoryUsage() const
	{
		size_t total = 0;

		total += getVectorMemoryUsage(jointBodyPairs);
		total += getVectorMemoryUsage(orderedJointIndices);
		total += getVectorMemoryUsage(jointConstraintContainer);

		total += solvingPlanner.getMemoryUsage();

		return total;
	}

	void JointSolver::solveJoints(
		Real deltaTime,
		uint32_t jointIterations,
		bool jointsWereChanged
	)
	{
		const size_t jointCount = joints.getCount();
		if (jointCount == 0) return;

		const size_t workerCount = planWorkerCount(jointCount);

		// Single-threaded path.
		if (workerCount <= 1)
		{
			TRACY_SCOPE_NC("Solve joints (Single-threaded)", Ecstasy::Core::Color::OliveDrab);

			if (orderedJointIndices.size() != jointCount)
			{
				orderedJointIndices.resize(jointCount);
				for (size_t i = 0; i < jointCount; i++)
				{
					orderedJointIndices[i] = i;
				}
			}

			computeConstraintData(deltaTime, orderedJointIndices);

			for (uint32_t i = 0; i < jointIterations; i++)
			{
				solveVelocityConstraints(jointConstraintContainer);
			}
			return;
		}

		// Multi-threading path.
		TRACY_SCOPE_NC("Solve joints (Multi-threaded)", Ecstasy::Core::Color::OliveDrab);
		if (jointsWereChanged || solvingPlanner.getFlatIndices().size() != jointCount)
		{
			{
				TRACY_SCOPE_NC("Collect body pairs from joints", Ecstasy::Core::Color::Red);
				jointBodyPairs.resize(jointCount);
				for (size_t i = 0; i < jointCount; i++)
				{
					jointBodyPairs[i] = { joints.bodyIndexA[i], joints.bodyIndexB[i] };
				}
			}
			{
				TRACY_SCOPE_NC("Plan execution", Ecstasy::Core::Color::Blue);
				solvingPlanner.setWorkerCount(workerCount);

				solvingPlanner.planCacheLineAwareExecution(
					jointBodyPairs, bodies->getCount(),
					sizeof(Real),
					std::hardware_destructive_interference_size
				);
			}
		}
		{
			TRACY_SCOPE_N("Reorder joint indices");

			const auto& flatIndices = solvingPlanner.getFlatIndices();
			orderedJointIndices.assign(flatIndices.begin(), flatIndices.end());
		}
		computeConstraintData(deltaTime, orderedJointIndices);

		solveConstraintsThreaded(jointIterations);
	}

	size_t JointSolver::planWorkerCount(size_t jointCount) const
	{
		// Note: mirrors SpringSolver's threshold as-is; re-profile once joints are a bottleneck
		// on their own (a joint's constraint data is heavier than a spring's, so this may be
		// too generous).
		static constexpr size_t JOINT_COUNT_PER_WORKER = 7'000;
		static constexpr size_t MIN_JOINT_COUNT_FOR_THREADING = JOINT_COUNT_PER_WORKER * 3;

		// Single-threaded path.
		if (jointCount < MIN_JOINT_COUNT_FOR_THREADING)
		{
			return 0;
		}

		// Multi-threading path.
		auto& threadPool = Threading::getGlobalThreadPool();

		const size_t availableWorkerCount = threadPool.getThreadCount();
		const size_t neededWorkerCount = jointCount / JOINT_COUNT_PER_WORKER;

		return std::min(availableWorkerCount, neededWorkerCount);
	}

	void JointSolver::computeConstraintData(Real deltaTime, std::span<const size_t> jointIndices)
	{
		TRACY_SCOPE_NC("Compute joint constraint data", Ecstasy::Core::Color::Chocolate);

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

		const Real* ECSTASY_RESTRICT jointStiffnessPtr = joints.stiffness;
		const Real* ECSTASY_RESTRICT jointDampingPtr = joints.damping;

		// Resize container.
		const size_t jointCount = jointIndices.size();
		jointConstraintContainer.resize(jointCount);

		const size_t* ECSTASY_RESTRICT jointIndicesPtr = jointIndices.data();

		// Compute constant data.
		for (size_t jointIndex = 0; jointIndex < jointCount; jointIndex++)
		{
			const size_t i = jointIndicesPtr[jointIndex];
			JointConstraintData& constraintData = jointConstraintContainer[jointIndex];

			const ObjectIndex bodyIndexA = joints.bodyIndexA[i];
			const ObjectIndex bodyIndexB = joints.bodyIndexB[i];

			constraintData.bodyIndexA = bodyIndexA;
			constraintData.bodyIndexB = bodyIndexB;

			const Real invMassA = invMassPtr[bodyIndexA];
			const Real invMassB = invMassPtr[bodyIndexB];
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			// Both attachments are static anchors -> nothing to solve.
			if (invMassA == Real(0) && invMassB == Real(0)) [[unlikely]]
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

			const Vec2 rotatedAnchorA = rotate(joints.localAnchorA[i], rotationCosPtr[bodyIndexA], rotationSinPtr[bodyIndexA]);
			const Vec2 rotatedAnchorB = rotate(joints.localAnchorB[i], rotationCosPtr[bodyIndexB], rotationSinPtr[bodyIndexB]);

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

	void JointSolver::solveVelocityConstraints(
		std::span<const JointConstraintData> constraintDataContainer
	)
	{
		TRACY_SCOPE_NC("Solve joint velocity constraints", Ecstasy::Core::Color::Violet);

		// Get pointers.
		Real* ECSTASY_RESTRICT velocityXPtr = bodies->velocityX.data();
		Real* ECSTASY_RESTRICT velocityYPtr = bodies->velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies->angularVelocity.data();
		const Real* ECSTASY_RESTRICT invMassPtr = bodies->invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies->invInertia.data();

		// Main loop.
		const size_t jointCount = constraintDataContainer.size();

		for (size_t c = 0; c < jointCount; c++)
		{
			const JointConstraintData& constraintData = constraintDataContainer[c];
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

	void JointSolver::solveConstraintsThreaded(uint32_t jointIterations)
	{
		const size_t workerCount = solvingPlanner.getWorkerCount();

		const auto& passOffsets = solvingPlanner.getPassOffsets();
		const size_t waveCount = passOffsets.size() / workerCount;
		const auto* ECSTASY_RESTRICT passOffsetsPtr = passOffsets.data();

		if (waveCount == 0 || jointIterations == 0) [[unlikely]]
		{
			return;
		}

		const uint32_t totalTicks = static_cast<uint32_t>(waveCount) * jointIterations;

		runThreadedWaves(workerCount, waveCount, totalTicks,
			[&](size_t waveIndex, size_t workerIndex, uint32_t /*passTicket*/)
			{
				const size_t passGlobalIndex = waveIndex * workerCount + workerIndex;
				const auto& passOffset = passOffsetsPtr[passGlobalIndex];

				// Get work from current wave and execute it. If empty, skip.
				if (passOffset.size == 0) return;

				std::span<const JointConstraintData> constraintSlice(jointConstraintContainer.data() + passOffset.start, passOffset.size);

				solveVelocityConstraints(constraintSlice);
			}
		);
	}
}