#include "Simulation.h"

#include "Core/TracyProfiler.h"
#include "Core/Portablity.h"

#include <iostream>

namespace PS_AGONY
{
    static Real calculateCircleInertia(Real mass, Real radius)
    {
        return Real(0.5) * radius * radius * mass;
    }

    static Real calculateBoxInertia(Real mass, Real width, Real height)
    {
        constexpr Real div = 1.0 / 12.0;
        return div * mass * (width * width + height * height);
    }


    static __forceinline Vec2 rotate2D(Vec2 point, Real cos, Real sin)
    {
        const float nx = point.x * cos - point.y * sin;
        const float ny = point.x * sin + point.y * cos;
        return { nx, ny };
    }

    static __forceinline Vec2 rotate2D(Vec2 point, Real angle)
    {
        return rotate2D(point, std::cos(angle), std::sin(angle));
    }
    

    Simulation::Simulation()
    {
        materials.reserve(16);
        materials.emplace_back(); // Default material.
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");

        // Physics steps.
        updateTimeAccumulator += deltaTime;

        uint32_t stepCount = std::floor(updateTimeAccumulator / simulationSettings.updateInterval);
        updateTimeAccumulator -= stepCount * simulationSettings.updateInterval;

        stepCount = std::min(stepCount, simulationSettings.maxIterationsPerUpdateCall);
        const Real fixedDeltaTime = simulationSettings.updateInterval * simulationSettings.timeScale;
        for (uint32_t i = 0; i < stepCount; i++)
        {
            physicsStep(fixedDeltaTime);
        }

        // Debug data.
        {
            debugDataResetTimeAccumulator += deltaTime;
            if (debugDataResetTimeAccumulator > 1.0)
            {
                debugDataSnaphot = runtimeDebugData;

                debugDataResetTimeAccumulator = 0.0;

                runtimeDebugData.updatesHappened = 0;

                collectMemoryUsage(debugDataSnaphot);
            }

            runtimeDebugData.updatesHappened += stepCount;
            runtimeDebugData.updatesSupposedToHappen = std::floor(Real(1.0) / simulationSettings.updateInterval);
        }
    }

    BodyIndex Simulation::createCircle(Vec2 position, Vec2 velocity, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex, Real radius)
    {
        mass = std::max(Real(0.0), mass);
        radius = std::max(Real(0.0), radius);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = circles.getCount();

        const Real inertia = calculateCircleInertia(mass, radius);

        bodies.append(
            position,
            velocity,
            rotation,
            angularVelocity,
            mass, mass == 0.0 ? 0.0 : 1.0 / mass,
            inertia, inertia == 0.0 ? 0.0 : 1.0 / inertia,
			Vec2(0.0, 0.0),
            materialIndex < materials.size() ? materialIndex : 0,
            BodyType::Circle,
            newShapeIndex
		);

        circles.append(
            newBodyIndex,
            radius
		);

        return newBodyIndex;
    }

    BodyIndex Simulation::createBox(Vec2 position, Vec2 velocity, Real rotation, Real angularVelocity, Real mass, MaterialIndex materialIndex, Vec2 size)
    {
        mass = std::max(Real(0.0), mass);
        const Real width = std::max(Real(0.0), size.x);
        const Real height = std::max(Real(0.0), size.y);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = boxes.getCount();

        const Real inertia = calculateBoxInertia(mass, width, height);

        bodies.append(
            position,
            velocity,
            rotation,
            angularVelocity,
            mass, mass == 0.0 ? 0.0 : 1.0 / mass,
            inertia, inertia == 0.0 ? 0.0 : 1.0 / inertia,
            Vec2(0.0, 0.0),
            materialIndex < materials.size() ? materialIndex : 0,
            BodyType::Box,
            newShapeIndex
        );

        const Real halfWidth = width * Real(0.5);
        const Real halfHeight = height * Real(0.5);

        boxes.append(
            newBodyIndex,
            halfWidth,
            halfHeight
        );

        return newBodyIndex;
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::fetchBroadPhaseAABBs(std::vector<AABB>& outAABBs) const
    {
		broadPhaseCollisionDetector.fetchAABBs(outAABBs);
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_N("Physics step");

        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        if (materials.empty()) [[unlikely]]
        {
            std::cerr << "[AGONY][Simulation]: Material count is zero, which must be impossible.\n";
            materials.emplace_back(); // Default material.
        }

        applyExternalForces(bodyCount, deltaTime);
        integrate(bodyCount, deltaTime);
        iterativeCollisionSolving();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Apply external forces");

        Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* CORE_RESTRICT invMassPtr = bodies.invMass.data();

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };

        const RealSimd zeros = RealSimd(Real(0));

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            RealSimd velX = RealSimd::load(velocityXPtr + i);
            RealSimd velY = RealSimd::load(velocityYPtr + i);

            const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            const auto movableMask = invMassV != zeros;

            RealSimd newVelX = velX + gravityDeltaXV;
            RealSimd newVelY = velY + gravityDeltaYV;

            velX = RealSimd::blendv(velX, newVelX, movableMask);
            velY = RealSimd::blendv(velY, newVelY, movableMask);

            velX.store(velocityXPtr + i);
            velY.store(velocityYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real invMass = invMassPtr[i];
            const Real movableMask = invMass != Real(0.0);
        
            velocityXPtr[i] += gravityDelta.x * movableMask;
            velocityYPtr[i] += gravityDelta.y * movableMask;
        }
    }

    void Simulation::integrate(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Intergrate");

        const RealSimd deltaTimeV{ deltaTime };

        // Position.
        {
            Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
            Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
            const Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
            const Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();

            // Note: having single loop (x and y interleaved) is a very-little faster than doing two separate passes.
            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd velX = RealSimd::load(velocityXPtr + i);
                const RealSimd velY = RealSimd::load(velocityYPtr + i);

                RealSimd posX = RealSimd::load(positionXPtr + i);
                RealSimd posY = RealSimd::load(positionYPtr + i);

                posX = RealSimd::mul_add(velX, deltaTimeV, posX);
                posY = RealSimd::mul_add(velY, deltaTimeV, posY);

                posX.store(positionXPtr + i);
                posY.store(positionYPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                positionXPtr[i] += velocityXPtr[i] * deltaTime;
                positionYPtr[i] += velocityYPtr[i] * deltaTime;
            }
        }

        // Rotation.
        {
            Real* CORE_RESTRICT rotationPtr = bodies.rotation.data();
            const Real* CORE_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd angularVel = RealSimd::load(angularVelocityPtr + i);
                
                RealSimd rot = RealSimd::load(rotationPtr + i);
                
                rot = RealSimd::mul_add(angularVel, deltaTimeV, rot);
                
                rot.store(rotationPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                rotationPtr[i] += angularVelocityPtr[i] * deltaTime;
            }
        }
    }

    void Simulation::iterativeCollisionSolving()
    {
        const size_t bodyCount = bodies.getCount();

        // Rebuild AABBs.
        buildAABBs();

		// Compute rotation cos/sin for all bodies, which is used in collision resolution.
		computeRotationCosSin();

        // Early return.
        if (bodyCount < 2) return;

        // Solves until runs out of iterations or no collision is found.
        for (uint32_t i = 0; i < simulationSettings.collisionSolvingIterations; i++)
        {
            // Broad phase.
            const std::vector<BodyPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions(AABBSoAViewer(bodies.aabb));
            if (broadCollisionData.empty()) return;

            // Narrow phase.
            narrowPhaseCollisionDetector.setDataViewers(
                BodySoAViewer(bodies),
                CircleSoAViewer(circles),
                BoxSoAViewer(boxes)
            );
            const std::vector<BodyCollisionData>& narrowCollisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisionData);
            if (narrowCollisionData.empty()) return;

            // Collision resolution.
            resolveCollisions(narrowCollisionData);

			// Rebuild AABBs.
            buildAABBs();
        }
    }

    void Simulation::buildAABBs()
    {
        TRACY_SCOPE_N("Build AABBs");
        buildCircleAABBs();
        buildBoxAABBs();
    }

    void Simulation::buildCircleAABBs()
    {
        TRACY_SCOPE_N("Build circle AABBs");

        const size_t count = circles.getCount();
        if (count == 0) return;

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();

        const BodyIndex* CORE_RESTRICT bodyIndexPtr = circles.bodyIndices.data();
        const Real* CORE_RESTRICT radiusPtr = circles.radius.data();

        Real* CORE_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* CORE_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* CORE_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* CORE_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real radius = radiusPtr[i];

            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];

            aabbMinXPtr[bodyIndex] = x - radius;
            aabbMinYPtr[bodyIndex] = y - radius;
            aabbMaxXPtr[bodyIndex] = x + radius;
            aabbMaxYPtr[bodyIndex] = y + radius;
        }
    }

    void Simulation::buildBoxAABBs()
    {
        TRACY_SCOPE_N("Build box AABBs");

        const size_t count = boxes.getCount();
        if (count == 0) return;

        const Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
        const Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
        const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const BodyIndex* CORE_RESTRICT bodyIndexPtr = boxes.bodyIndices.data();
        const Real* CORE_RESTRICT halfWidthPtr = boxes.halfWidth.data();
        const Real* CORE_RESTRICT halfHeightPtr = boxes.halfHeight.data();

        Real* CORE_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* CORE_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* CORE_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* CORE_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real widthHalf = halfWidthPtr[i];
            const Real heightHalf = halfHeightPtr[i];

            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];
            const Real cos = rotationCosPtr[bodyIndex];
            const Real sin = rotationSinPtr[bodyIndex];

            const Real absCos = std::abs(cos);
            const Real absSin = std::abs(sin);

            const Real ex = absCos * widthHalf + absSin * heightHalf;
            const Real ey = absSin * widthHalf + absCos * heightHalf;

            aabbMinXPtr[bodyIndex] = x - ex;
            aabbMinYPtr[bodyIndex] = y - ey;
            aabbMaxXPtr[bodyIndex] = x + ex;
            aabbMaxYPtr[bodyIndex] = y + ey;
        }
    }

    void Simulation::computeRotationCosSin()
    {
        TRACY_SCOPE_N("Compute rotation cos/sin");

        const Real* CORE_RESTRICT rotationPtr = bodies.rotation.data();
        Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const size_t bodyCount = bodies.getCount();
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real rot = rotationPtr[i];
            rotationCosPtr[i] = std::cos(rot);
            rotationSinPtr[i] = std::sin(rot);
		}
    }

    void Simulation::resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        TRACY_SCOPE_N("Resolve collisions");

		constexpr Real frictionEpsilon = Real(0.0001);

        // Get pointers.
        Real* CORE_RESTRICT positionXPtr = bodies.positionX.data();
        Real* CORE_RESTRICT positionYPtr = bodies.positionY.data();
        Real* CORE_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* CORE_RESTRICT velocityYPtr = bodies.velocityY.data();
		Real* CORE_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();
        const Real* CORE_RESTRICT invMassPtr = bodies.invMass.data();
		const Real* CORE_RESTRICT invInertiaPtr = bodies.invInertia.data();
		const Real* CORE_RESTRICT localCenterOfMassXPtr = bodies.localCenterOfMassX.data();
		const Real* CORE_RESTRICT localCenterOfMassYPtr = bodies.localCenterOfMassY.data();
		const Real* CORE_RESTRICT rotationCosPtr = bodies.rotationCos.data();
		const Real* CORE_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const MaterialIndex* CORE_RESTRICT materialIndexPtr = bodies.materialIndex.data();
        const Material* CORE_RESTRICT materialPtr = materials.data();

        // Lambdas.
        auto getLinearVelocity = [&](BodyIndex bodyIndex) -> Vec2
        {
            const Real velocityX = velocityXPtr[bodyIndex];
            const Real velocityY = velocityYPtr[bodyIndex];
            return { velocityX, velocityY };
			};

        auto getCenterOfMass = [&](BodyIndex bodyIndex) -> Vec2
        {
			const Real positionX = positionXPtr[bodyIndex];
			const Real positionY = positionYPtr[bodyIndex];
            const Real localCenterOfMassX = localCenterOfMassXPtr[bodyIndex];
            const Real localCenterOfMassY = localCenterOfMassYPtr[bodyIndex];
            const Real cosRot = rotationCosPtr[bodyIndex];
            const Real sinRot = rotationSinPtr[bodyIndex];
            const Real centerOfMassX = positionX + localCenterOfMassX * cosRot - localCenterOfMassY * sinRot;
            const Real centerOfMassY = positionY + localCenterOfMassX * sinRot + localCenterOfMassY * cosRot;
            return { centerOfMassX, centerOfMassY };
			};

        auto applyImpulse = [&](BodyIndex bodyIndex, Vec2 impulse, Vec2 rPerp, Real invMass, Real invInertia)
        {
            velocityXPtr[bodyIndex] += impulse.x * invMass;
            velocityYPtr[bodyIndex] += impulse.y * invMass;
			angularVelocityPtr[bodyIndex] += glm::dot(rPerp, impulse) * invInertia;
		};

        // Main loop.
        for (const auto& data : narrowPhaseCollisions)
        {
			// Get body indices.
            const BodyIndex bodyIndexA = data.bodyA;
            const BodyIndex bodyIndexB = data.bodyB;

            // Get inv masses.
            const Real invMassA = invMassPtr[bodyIndexA];
            const Real invMassB = invMassPtr[bodyIndexB];

            const Real totalInvMass = invMassA + invMassB;
            if (totalInvMass <= Real(0)) [[unlikely]]
            {
                continue;
            }

			// Get materials.
            const MaterialIndex materialIndexA = materialIndexPtr[bodyIndexA];
            const MaterialIndex materialIndexB = materialIndexPtr[bodyIndexB];

            const Material* materialA = materialPtr + materialIndexA;
            const Material* materialB = materialPtr + materialIndexB;

            const Real elasticityPlusOne = (materialA->elasticity + materialB->elasticity) * Real(0.5) + Real(1.0); // Hoping for fused multiply-add. Adding here instead of adding in impulse calculation.
            
            // TODO: Maybe put this after applying collision impulses.
            const Real staticFriction = std::sqrt(materialA->staticFriction * materialB->staticFriction);
            const Real dynamicFriction = std::sqrt(materialA->dynamicFriction * materialB->dynamicFriction);

			// Compute world centers of mass.
			const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
			const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

            //
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			const Vec2 normal = data.normal;
            const Real depth = data.depth;

            // Calculate collision impulses.
            Vec2 impulses[2];
            Vec2 rAPerpArray[2];
            Vec2 rBPerpArray[2];
            Real jnArray[2];
            uint32_t validContactCount = 0;
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];

				const uint32_t contactCount = std::min(data.contactCount, 2u);

                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 contactPoint = data.contacts[i];

                    const Vec2 rA = contactPoint - centerOfMassA;
                    const Vec2 rB = contactPoint - centerOfMassB;

                    const Vec2 rAPerp = { -rA.y, rA.x };
                    const Vec2 rBPerp = { -rB.y, rB.x };

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

                    const Vec2 relativeVelocity = (linearVelocityB + angularLinearVelB) - (linearVelocityA + angularLinearVelA);

                    const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);
                    if (velocityAlongNormal > Real(0))
                    {
                        continue;
                    }

                    const Real rAPerpDotN = glm::dot(rAPerp, normal);
                    const Real rBPerpDotN = glm::dot(rBPerp, normal);

                    const Real inertiaTermA = rAPerpDotN * rAPerpDotN * invInertiaA;
                    const Real inertiaTermB = rBPerpDotN * rBPerpDotN * invInertiaB;

                    const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
                    const Real jn = -elasticityPlusOne * velocityAlongNormal / denom;

                    impulses[validContactCount] = jn * normal;
                    rAPerpArray[validContactCount] = rAPerp;
                    rBPerpArray[validContactCount] = rBPerp;
                    jnArray[validContactCount] = jn;

                    validContactCount++;
                }
            }

			// Check if there is at least one valid contact.
            if (validContactCount == 0) continue;

            // Apply collision impulses.
            {
				const Real impulseScale = Real(1) / static_cast<Real>(validContactCount);
                for (uint32_t i = 0; i < validContactCount; i++)
                {
                    const Vec2 impulse = impulses[i] * impulseScale;
                    applyImpulse(bodyIndexA, -impulse, rAPerpArray[i], invMassA, invInertiaA);
					applyImpulse(bodyIndexB,  impulse, rBPerpArray[i], invMassB, invInertiaB);

					jnArray[i] *= impulseScale; // Scale the normal impulse for later use in friction calculation.
                }
            }

			// Calculate friction impulses.
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];
                for (uint32_t i = 0; i < validContactCount; i++)
                {
                    const Vec2 rAPerp = rAPerpArray[i];
                    const Vec2 rBPerp = rBPerpArray[i];

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

                    const Vec2 relativeVelocity = (linearVelocityB + angularLinearVelB) - (linearVelocityA + angularLinearVelA);

					Vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
					const Real tangentLengthSq = glm::dot(tangent, tangent);
                    if (tangentLengthSq < frictionEpsilon)
                    {
						impulses[i] = Vec2(0.0, 0.0);
                        continue;
                    }

					tangent /= std::sqrt(tangentLengthSq);

					const Real rAPerpDotT = glm::dot(rAPerp, tangent);
					const Real rBPerpDotT = glm::dot(rBPerp, tangent);

					const Real inertiaTermA = rAPerpDotT * rAPerpDotT * invInertiaA;
					const Real inertiaTermB = rBPerpDotT * rBPerpDotT * invInertiaB;

                    const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
					const Real jt = -glm::dot(relativeVelocity, tangent) / denom;

					const Real jn = jnArray[i];
                    if (std::abs(jt) <= jn * staticFriction)
                    {
                        impulses[i] = jt * tangent; // Static friction.
                    }
                    else
                    {
                        impulses[i] = -jn * dynamicFriction * tangent; // Dynamic friction.
					}
                }
            }

			// Apply friction impulses.
            {
                for (uint32_t i = 0; i < validContactCount; i++)
                {
                    const Vec2 impulse = impulses[i];
                    applyImpulse(bodyIndexA, -impulse, rAPerpArray[i], invMassA, invInertiaA);
                    applyImpulse(bodyIndexB, impulse, rBPerpArray[i], invMassB, invInertiaB);
                }
            }

            // Position resolution.
            const Real correction = (depth - simulationSettings.slop) * simulationSettings.positionCorrectionPercent;
            if (correction <= Real(0)) [[unlikely]]
            {
                continue;
            }

            const Vec2 correctionVec2 = normal * correction;

			const Real invTotalInvMass = Real(1) / totalInvMass;
            const Real ratioA = invMassA * invTotalInvMass;
            const Real ratioB = invMassB * invTotalInvMass;

            positionXPtr[bodyIndexA] -= correctionVec2.x * ratioA;
            positionYPtr[bodyIndexA] -= correctionVec2.y * ratioA;

            positionXPtr[bodyIndexB] += correctionVec2.x * ratioB;
            positionYPtr[bodyIndexB] += correctionVec2.y * ratioB;
        }
    }

    void Simulation::collectMemoryUsage(DebugData& data) const
    {
        // Memory
        {
            auto& total = data.bodyDataMemoryUsage;
            total = sizeof(BodySoA);
            total += bodies.getMemoryUsage();
        }
        {
            auto& total = data.circleDataMemoryUsage;
            total = sizeof(CircleSoA);
            total += circles.getMemoryUsage();
        }
        {
            auto& total = data.boxDataMemoryUsage;
            total = sizeof(BoxSoA);
            total += boxes.getMemoryUsage();
        }
        data.materialDataMemoryUsage = materials.capacity() * sizeof(materials[0]);
        data.broadPhaseDetectorMemoryUsage = sizeof(BroadPhaseCollisionDetector) + broadPhaseCollisionDetector.getMemoryUsage();
        data.narrowPhaseDetectorMemoryUsage = sizeof(NarrowPhaseCollisionDetector) + narrowPhaseCollisionDetector.getMemoryUsage();
    }
}