#include "Simulation.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <iostream>
#include <algorithm>

namespace PS_AGONY
{
    static Real calculateCircleInertia(Real mass, Real radius, Vec2 centerOfMass)
    {
        const Real radiusSquared = radius * radius;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return (Real(0.5) * radiusSquared + deltaSquared) * mass;
    }

    static Real calculateBoxInertia(Real mass, Real width, Real height, Vec2 centerOfMass)
    {
        constexpr Real div = 1.0 / 12.0;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return mass * (div * (width * width + height * height) + deltaSquared);
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

        // Spawn a thread pool.
        auto& threadPool = getGlobalThreadPool();
        (void)threadPool;
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_N("Simulation update");

        // Delta time check.
        if (deltaTime <= 0) return;

        // Cap how much real time this call is allowed to consume.
        const Real cappedDeltaTime = std::fmin(deltaTime, simulationSettings.maxDeltaTimePerUpdateCall);

        // Advance counter.
        updateTimeAccumulator += cappedDeltaTime;

        // Physics steps.
        uint32_t stepCount = std::floor(updateTimeAccumulator / simulationSettings.updateInterval);
        updateTimeAccumulator -= stepCount * simulationSettings.updateInterval;

        const Real fixedDeltaTime = simulationSettings.updateInterval * simulationSettings.timeScale;
        if (stepCount > 0)
        {
            for (uint32_t i = 0; i < stepCount; i++)
            {
                physicsStep(fixedDeltaTime);
            }
            postUpdate();
        }

        // Debug data.
        {
            constexpr Real DEBUG_DATA_SWITCH_INTERVAL = 0.25;

            debugDataResetTimeAccumulator += deltaTime;
            if (debugDataResetTimeAccumulator > DEBUG_DATA_SWITCH_INTERVAL)
            {
                debugDataSnaphot = runtimeDebugData;

                debugDataResetTimeAccumulator = 0.0;

                runtimeDebugData.updatesHappened = 0;

                collectMemoryUsage(debugDataSnaphot);
            }

            runtimeDebugData.updatesHappened += stepCount / DEBUG_DATA_SWITCH_INTERVAL;
            runtimeDebugData.updatesSupposedToHappen = std::floor(Real(1.0) / simulationSettings.updateInterval);

            runtimeDebugData.maxCollisionSolvingIterations = simulationSettings.collisionSolvingIterations;
        }
    }

    void Simulation::createCircle(
        Vec2 position,
        Vec2 velocity,
        Real rotation,
        Real angularVelocity,
        Real mass,
        Vec2 centerOfMass,
        MaterialIndex materialIndex,
        Real radius,
        BodyTextureId textureId
    )
    {
        mass = std::fmax(Real(0.0), mass);
        radius = std::fmax(Real(0.0), radius);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = circles.getCount();

        const Real inertia = calculateCircleInertia(mass, radius, centerOfMass);

        bodies.append(
            position,
            velocity,
            rotation,
            angularVelocity,
            mass, mass == 0.0 ? 0.0 : 1.0 / mass,
            inertia, inertia == 0.0 ? 0.0 : 1.0 / inertia,
			centerOfMass,
            materialIndex < materials.size() ? materialIndex : 0,
            BodyType::Circle,
            newShapeIndex,
            textureId
		);

        circles.append(
            newBodyIndex,
            radius
		);
    }

    void Simulation::createBox(
        Vec2 position,
        Vec2 velocity,
        Real rotation,
        Real angularVelocity,
        Real mass,
        Vec2 centerOfMass,
        MaterialIndex materialIndex,
        Vec2 size,
        BodyTextureId textureId
    )
    {
        mass = std::fmax(Real(0.0), mass);
        const Real width = std::fmax(Real(0.0), size.x);
        const Real height = std::fmax(Real(0.0), size.y);

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = boxes.getCount();

        const Real inertia = calculateBoxInertia(mass, width, height, centerOfMass);

        bodies.append(
            position,
            velocity,
            rotation,
            angularVelocity,
            mass, mass == 0.0 ? 0.0 : 1.0 / mass,
            inertia, inertia == 0.0 ? 0.0 : 1.0 / inertia,
            centerOfMass,
            materialIndex < materials.size() ? materialIndex : 0,
            BodyType::Box,
            newShapeIndex,
            textureId
        );

        const Real halfWidth = width * Real(0.5);
        const Real halfHeight = height * Real(0.5);

        boxes.append(
            newBodyIndex,
            halfWidth,
            halfHeight
        );
    }

    void Simulation::destroyBody(BodyIndex bodyIndex)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyIndex >= bodyCount) return;

        const BodyType type = bodies.bodyType[bodyIndex];
        const BodyIndex shapeIdx = bodies.shapeIndex[bodyIndex];

        // Remove shape entry from the appropriate SoA.
        if (type == BodyType::Circle)
        {
            const size_t circleCount = circles.getCount();
            if (shapeIdx < circleCount)
            {
                // Swap with last element if not already last.
                if (shapeIdx != circleCount - 1)
                {
                    // Swap body indices in circles.
                    std::swap(circles.bodyIndices[shapeIdx], circles.bodyIndices.back());
                    std::swap(circles.radius[shapeIdx], circles.radius.back());

                    // Update the body that now occupies shapeIdx to point to the new shape index.
                    const BodyIndex swappedBody = circles.bodyIndices[shapeIdx];
                    bodies.shapeIndex[swappedBody] = shapeIdx;
                }
                circles.bodyIndices.pop_back();
                circles.radius.pop_back();
            }
        }
        else if (type == BodyType::Box)
        {
            const size_t boxCount = boxes.getCount();
            if (shapeIdx < boxCount)
            {
                if (shapeIdx != boxCount - 1)
                {
                    std::swap(boxes.bodyIndices[shapeIdx], boxes.bodyIndices.back());
                    std::swap(boxes.halfWidth[shapeIdx], boxes.halfWidth.back());
                    std::swap(boxes.halfHeight[shapeIdx], boxes.halfHeight.back());

                    const BodyIndex swappedBody = boxes.bodyIndices[shapeIdx];
                    bodies.shapeIndex[swappedBody] = shapeIdx;
                }
                boxes.bodyIndices.pop_back();
                boxes.halfWidth.pop_back();
                boxes.halfHeight.pop_back();
            }
        }

        // Remove body entry from BodySoA.
        if (bodyIndex != bodyCount - 1)
        {
            // Swap all vectors in BodySoA.
            std::swap(bodies.positionX[bodyIndex], bodies.positionX.back());
            std::swap(bodies.positionY[bodyIndex], bodies.positionY.back());
            std::swap(bodies.localCenterOfMassX[bodyIndex], bodies.localCenterOfMassX.back());
            std::swap(bodies.localCenterOfMassY[bodyIndex], bodies.localCenterOfMassY.back());
            std::swap(bodies.truePositionX[bodyIndex], bodies.truePositionX.back());
            std::swap(bodies.truePositionY[bodyIndex], bodies.truePositionY.back());
            std::swap(bodies.velocityX[bodyIndex], bodies.velocityX.back());
            std::swap(bodies.velocityY[bodyIndex], bodies.velocityY.back());
            std::swap(bodies.rotation[bodyIndex], bodies.rotation.back());
            std::swap(bodies.angularVelocity[bodyIndex], bodies.angularVelocity.back());
            std::swap(bodies.mass[bodyIndex], bodies.mass.back());
            std::swap(bodies.invMass[bodyIndex], bodies.invMass.back());
            std::swap(bodies.inertia[bodyIndex], bodies.inertia.back());
            std::swap(bodies.invInertia[bodyIndex], bodies.invInertia.back());
            std::swap(bodies.rotationCos[bodyIndex], bodies.rotationCos.back());
            std::swap(bodies.rotationSin[bodyIndex], bodies.rotationSin.back());
            std::swap(bodies.materialIndex[bodyIndex], bodies.materialIndex.back());
            std::swap(bodies.aabb.minX[bodyIndex], bodies.aabb.minX.back());
            std::swap(bodies.aabb.minY[bodyIndex], bodies.aabb.minY.back());
            std::swap(bodies.aabb.maxX[bodyIndex], bodies.aabb.maxX.back());
            std::swap(bodies.aabb.maxY[bodyIndex], bodies.aabb.maxY.back());
            std::swap(bodies.bodyType[bodyIndex], bodies.bodyType.back());
            std::swap(bodies.shapeIndex[bodyIndex], bodies.shapeIndex.back());
            std::swap(bodies.textureId[bodyIndex], bodies.textureId.back());

            // Update the shape entry that refers to the swapped body (if any).
            //const BodyIndex swappedBodyIndex = bodyCount - 1;
            const BodyType swappedType = bodies.bodyType[bodyIndex];
            const BodyIndex swappedShapeIdx = bodies.shapeIndex[bodyIndex];
            if (swappedType == BodyType::Circle)
            {
                if (swappedShapeIdx < circles.getCount())
                    circles.bodyIndices[swappedShapeIdx] = bodyIndex;
            }
            else if (swappedType == BodyType::Box)
            {
                if (swappedShapeIdx < boxes.getCount())
                    boxes.bodyIndices[swappedShapeIdx] = bodyIndex;
            }
        }

        // Pop back all BodySoA vectors.
        bodies.positionX.pop_back();
        bodies.positionY.pop_back();
        bodies.localCenterOfMassX.pop_back();
        bodies.localCenterOfMassY.pop_back();
        bodies.truePositionX.pop_back();
        bodies.truePositionY.pop_back();
        bodies.velocityX.pop_back();
        bodies.velocityY.pop_back();
        bodies.rotation.pop_back();
        bodies.angularVelocity.pop_back();
        bodies.mass.pop_back();
        bodies.invMass.pop_back();
        bodies.inertia.pop_back();
        bodies.invInertia.pop_back();
        bodies.rotationCos.pop_back();
        bodies.rotationSin.pop_back();
        bodies.materialIndex.pop_back();
        bodies.aabb.minX.pop_back();
        bodies.aabb.minY.pop_back();
        bodies.aabb.maxX.pop_back();
        bodies.aabb.maxY.pop_back();
        bodies.bodyType.pop_back();
        bodies.shapeIndex.pop_back();
        bodies.textureId.pop_back();
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::mainBodyHolderGrabAt(Vec2 grabPosition)
    {
        if (mainBodyHolder.heldBody.has_value()) return;

        constexpr Real MAX_GRAB_DISTANCE = 2.0;
        constexpr Real MAX_GRAB_DISTANCE_SQ = MAX_GRAB_DISTANCE * MAX_GRAB_DISTANCE;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.positionX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.positionY.data();
        const Real* ECSTASY_RESTRICT truePositionXPtr = bodies.truePositionX.data();
        const Real* ECSTASY_RESTRICT truePositionYPtr = bodies.truePositionY.data();
        const Real* ECSTASY_RESTRICT massPtr = bodies.mass.data();

        Real minSqDistance = FLT_MAX;
        BodyIndex closestBody;
        Vec2 closestBodyDelta;

        const uint32_t bodyCount = bodies.getCount();
        for (uint32_t i = 0; i < bodyCount; i++)
        {
            if (massPtr[i] == 0) continue;

            const Vec2 bodyTruePosition = { truePositionXPtr[i], truePositionYPtr[i] };

            const Vec2 delta = bodyTruePosition - grabPosition;

            const Real sqDistance = glm::dot(delta, delta);

            if (sqDistance > MAX_GRAB_DISTANCE_SQ) continue;
            else if (sqDistance < minSqDistance)
            {
                const Vec2 bodyPosition = { positionXPtr[i], positionYPtr[i] };

                minSqDistance = sqDistance;
                closestBody = i;
                closestBodyDelta = bodyPosition - grabPosition;
            }
        }

        // Grab.
        if (minSqDistance < FLT_MAX)
        {
            mainBodyHolder.heldBody = closestBody;
            mainBodyHolder.bodyOffset = closestBodyDelta;
        }
    }

    void Simulation::mainBodyHolderRelease()
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        const Vec2 newBodyVelocity = mainBodyHolder.getVelocity();

        bodies.velocityX[bodyIndex] = newBodyVelocity.x;
        bodies.velocityY[bodyIndex] = newBodyVelocity.y;

        mainBodyHolder.heldBody = std::nullopt;
    }

    void Simulation::mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp)
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        bodies.angularVelocity[bodyIndex] += radiansSpeedUp;
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
            std::cerr << "[AGONY][Simulation]: Material count is zero, which should be impossible.\n";
            materials.emplace_back(); // Default material.
        }

        applyExternalForces(bodyCount, deltaTime);
        applyConstraints();
        integrate(bodyCount, deltaTime);
        wrapRotation();
        computeRotationCosSin();
        iterativeCollisionSolving(deltaTime);
    }

    void Simulation::postUpdate()
    {
        // That's for renderer to have actual information.
        computeTruePositions();
        buildBodyAABBs();
    }

    void Simulation::applyExternalForces(size_t bodyCount, Real deltaTime)
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Apply external forces");

        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

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
            Real* ECSTASY_RESTRICT positionXPtr = bodies.positionX.data();
            Real* ECSTASY_RESTRICT positionYPtr = bodies.positionY.data();
            const Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
            const Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();

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
            Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();
            const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

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

    void Simulation::iterativeCollisionSolving(Real deltaTime)
    {
        runtimeDebugData.collisionSolvingIterationsHappened = 0;

        const size_t bodyCount = bodies.getCount();

        // Early return.
        if (bodyCount < 2) return;

        // Set data viewers.
        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(bodies.aabb)
        );

        narrowPhaseCollisionDetector.setDataViewers(
            BodySoAViewer(bodies),
            CircleSoAViewer(circles),
            BoxSoAViewer(boxes)
        );

        // Solves until runs out of iterations or no collision is found.
        uint32_t i = 0;
        for (;i < simulationSettings.collisionSolvingIterations; i++)
        {
            // Compute true position for all bodies.
            computeTruePositions();

            // Rebuild AABBs.
            buildBodyAABBs();

            // Broad phase.
            const std::vector<BodyPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions(i == 0);
            if (broadCollisionData.empty()) break;

            // Narrow phase.
            const std::vector<BodyCollisionData>& narrowCollisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisionData);
            if (narrowCollisionData.empty()) break;

            // Collision resolution.
            resolveCollisions(narrowCollisionData);
        }
        runtimeDebugData.collisionSolvingIterationsHappened = i;
    }

    void Simulation::buildBodyAABBs()
    {
        TRACY_SCOPE_N("Build body AABBs");
        buildCircleAABBs();
        buildBoxAABBs();
    }

    void Simulation::buildCircleAABBs()
    {
        TRACY_SCOPE_N("Build circle AABBs");

        const size_t count = circles.getCount();
        if (count == 0) return;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.truePositionX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.truePositionY.data();

        const BodyIndex* ECSTASY_RESTRICT bodyIndexPtr = circles.bodyIndices.data();
        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

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

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.truePositionX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.truePositionY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const BodyIndex* ECSTASY_RESTRICT bodyIndexPtr = boxes.bodyIndices.data();
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth.data();
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real widthHalf = halfWidthPtr[i];
            const Real heightHalf = halfHeightPtr[i];

            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];
            const Real cos = rotationCosPtr[bodyIndex];
            const Real sin = rotationSinPtr[bodyIndex];

            const Real absCos = std::fabs(cos);
            const Real absSin = std::fabs(sin);

            const Real ex = absCos * widthHalf + absSin * heightHalf;
            const Real ey = absSin * widthHalf + absCos * heightHalf;

            aabbMinXPtr[bodyIndex] = x - ex;
            aabbMinYPtr[bodyIndex] = y - ey;
            aabbMaxXPtr[bodyIndex] = x + ex;
            aabbMaxYPtr[bodyIndex] = y + ey;
        }
    }

    void Simulation::wrapRotation()
    {
        TRACY_SCOPE_N("Wrap rotation");

        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

        const size_t bodyCount = bodies.getCount();
        for (size_t i = 0; i < bodyCount; i++)
        {
            rotationPtr[i] = std::fmod(rotationPtr[i], Constants::TWO_PI);
        }
    }

    void Simulation::computeRotationCosSin()
    {
        TRACY_SCOPE_N("Compute rotation cos/sin");

        const Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();
        Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const size_t bodyCount = bodies.getCount();
        for (size_t i = 0; i < bodyCount; i++)
        {
            const Real rot = rotationPtr[i];
            rotationCosPtr[i] = std::cos(rot);
            rotationSinPtr[i] = std::sin(rot);
		}
    }

    void Simulation::computeTruePositions()
    {
        using RealSimd = Simd<Real>;

        TRACY_SCOPE_N("Compute true positions");

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.positionX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.positionY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies.localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        Real* ECSTASY_RESTRICT truePositionXPtr = bodies.truePositionX.data();
        Real* ECSTASY_RESTRICT truePositionYPtr = bodies.truePositionY.data();

        const size_t bodyCount = bodies.getCount();

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            const RealSimd positionX = RealSimd::load(positionXPtr + i);
            const RealSimd positionY = RealSimd::load(positionYPtr + i);

            const RealSimd localCenterOfMassX = RealSimd::load(localCenterOfMassXPtr + i);
            const RealSimd localCenterOfMassY = RealSimd::load(localCenterOfMassYPtr + i);

            const RealSimd cosRot = RealSimd::load(rotationCosPtr + i);
            const RealSimd sinRot = RealSimd::load(rotationSinPtr + i);

            const RealSimd truePositionX = RealSimd::neg_mul_add(localCenterOfMassX, cosRot,     RealSimd::mul_add(localCenterOfMassY, sinRot, positionX + localCenterOfMassX));
            const RealSimd truePositionY = RealSimd::neg_mul_add(localCenterOfMassX, sinRot, RealSimd::neg_mul_add(localCenterOfMassY, cosRot, positionY + localCenterOfMassY));
        
            truePositionX.store(truePositionXPtr + i);
            truePositionY.store(truePositionYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real positionX = positionXPtr[i];
            const Real positionY = positionYPtr[i];
            const Real localCenterOfMassX = localCenterOfMassXPtr[i];
            const Real localCenterOfMassY = localCenterOfMassYPtr[i];
            const Real cosRot = rotationCosPtr[i];
            const Real sinRot = rotationSinPtr[i];
            truePositionXPtr[i] = (positionX + localCenterOfMassX) - (localCenterOfMassX * cosRot - localCenterOfMassY * sinRot);
            truePositionYPtr[i] = (positionY + localCenterOfMassY) - (localCenterOfMassX * sinRot + localCenterOfMassY * cosRot);
        }
    }

    void Simulation::resolveCollisions(const std::vector<BodyCollisionData>& narrowPhaseCollisions)
    {
        TRACY_SCOPE_N("Resolve collisions");

        constexpr Real frictionEpsilonSq = Real(1e-3 * 1e-3);

        // Get pointers.
        Real* ECSTASY_RESTRICT positionXPtr = bodies.positionX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies.positionY.data();

        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies.localCenterOfMassY.data();

        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
		Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();
		const Real* ECSTASY_RESTRICT invInertiaPtr = bodies.invInertia.data();

        const MaterialIndex* ECSTASY_RESTRICT materialIndexPtr = bodies.materialIndex.data();
        const Material* ECSTASY_RESTRICT materialPtr = materials.data();

        // Lambdas.
        auto getLinearVelocity = [&](const BodyIndex& bodyIndex) -> Vec2
        {
            const Real velocityX = velocityXPtr[bodyIndex];
            const Real velocityY = velocityYPtr[bodyIndex];
            return { velocityX, velocityY };
			};

        auto getCenterOfMass = [&](BodyIndex bodyIndex) -> Vec2
        {
			const Real positionX = positionXPtr[bodyIndex];
			const Real positionY = positionYPtr[bodyIndex];

            const Real localCOMX = localCenterOfMassXPtr[bodyIndex];
            const Real localCOMY = localCenterOfMassYPtr[bodyIndex];

            return { positionX + localCOMX, positionY + localCOMY };
			};

        // Main loop.
        // Note: Storing velocities on stack, updating them, then storing back with pointers was slower than how its right now. Why?!
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

            const Real staticFriction = std::sqrt(std::fmax(Real(0), materialA->staticFriction * materialB->staticFriction));
            const Real dynamicFriction = std::sqrt(std::fmax(Real(0), materialA->dynamicFriction * materialB->dynamicFriction));

			// Compute world centers of mass.
			const Vec2 centerOfMassA = getCenterOfMass(bodyIndexA);
			const Vec2 centerOfMassB = getCenterOfMass(bodyIndexB);

            //
			const Real invInertiaA = invInertiaPtr[bodyIndexA];
			const Real invInertiaB = invInertiaPtr[bodyIndexB];

			const Vec2 normal = data.normal;
            const Real depth = data.depth;

            // Calculate collision impulses.
            Vec2 impulseArray[2] = { Vec2(),  Vec2()  };
            Vec2 rAPerpArray[2] =  { Vec2(),  Vec2()  };
            Vec2 rBPerpArray[2] =  { Vec2(),  Vec2()  };
            Real jnArray[2] =      { Real(0), Real(0) };
            const uint32_t contactCount = data.contactCount;//  std::min(data.contactCount, 2u);

            const Real impulseScale = Real(1.0) / Real(data.contactCount);
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];

                bool noContacts = true;

                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 contactPoint = data.contacts[i];

                    const Vec2 rA = contactPoint - centerOfMassA;
                    const Vec2 rB = contactPoint - centerOfMassB;

                    const Vec2 rAPerp = { -rA.y, rA.x };
                    const Vec2 rBPerp = { -rB.y, rB.x };

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

                    const Vec2 relativeVelocity =
                        (linearVelocityB + angularLinearVelB) -
                        (linearVelocityA + angularLinearVelA);

                    const Real velocityAlongNormal = glm::dot(relativeVelocity, normal);

                    if (velocityAlongNormal > Real(0)) continue;

                    const Real rAPerpDotN = glm::dot(rAPerp, normal);
                    const Real rBPerpDotN = glm::dot(rBPerp, normal);

                    const Real inertiaTermA = rAPerpDotN * rAPerpDotN * invInertiaA;
                    const Real inertiaTermB = rBPerpDotN * rBPerpDotN * invInertiaB;

                    const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
                    const Real jn = -elasticityPlusOne * velocityAlongNormal / denom * impulseScale;

                    impulseArray[i] = jn * normal;
                    rAPerpArray[i] = rAPerp;
                    rBPerpArray[i] = rBPerp;
                    jnArray[i] = jn;

                    noContacts = false;
                }

                // Check if there is at least one valid contact.
                if (noContacts) continue;
            }

            // Apply collision impulses.
            {
                const Vec2 impulseSum = impulseArray[0] + impulseArray[1];
                {
                    const Vec2 linearVelocityChangeA = impulseSum * invMassA;
                    velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
                    velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;

                    const Real angularVelocityChangeA = (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;
                    angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
                }

                {
                    const Vec2 linearVelocityChangeB = impulseSum * invMassB;
                    velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
                    velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;

                    const Real angularVelocityChangeB = (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                    angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
                }
            }

			// Calculate friction impulses.
            {
                const Vec2 linearVelocityA = getLinearVelocity(bodyIndexA);
                const Vec2 linearVelocityB = getLinearVelocity(bodyIndexB);
                const Real angularVelA = angularVelocityPtr[bodyIndexA];
                const Real angularVelB = angularVelocityPtr[bodyIndexB];
                for (uint32_t i = 0; i < contactCount; i++)
                {
                    const Vec2 rAPerp = rAPerpArray[i];
                    const Vec2 rBPerp = rBPerpArray[i];

                    const Vec2 angularLinearVelA = rAPerp * angularVelA;
                    const Vec2 angularLinearVelB = rBPerp * angularVelB;

                    const Vec2 relativeVelocity =
                        (linearVelocityB + angularLinearVelB) -
                        (linearVelocityA + angularLinearVelA);

					Vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
					const Real tangentLengthSq = glm::dot(tangent, tangent);
                    if (tangentLengthSq < frictionEpsilonSq)
                    {
						impulseArray[i] = Vec2(0.0, 0.0);
                        continue;
                    }

					tangent /= std::sqrt(tangentLengthSq);

					const Real rAPerpDotT = glm::dot(rAPerp, tangent);
					const Real rBPerpDotT = glm::dot(rBPerp, tangent);

					const Real inertiaTermA = rAPerpDotT * rAPerpDotT * invInertiaA;
					const Real inertiaTermB = rBPerpDotT * rBPerpDotT * invInertiaB;

                    const Real denom = totalInvMass + inertiaTermA + inertiaTermB;
					const Real jt = glm::dot(relativeVelocity, tangent) / denom * impulseScale;

					const Real jn = jnArray[i];
                    if (std::fabs(jt) <= jn * staticFriction)
                    {
                        impulseArray[i] = -jt * tangent; // Static friction.
                    }
                    else
                    {
                        const Real maxDynamic = jn * dynamicFriction;
                        const Real f = -std::clamp(jt, -maxDynamic, maxDynamic);
                        impulseArray[i] = f * tangent; // Dynamic friction.
					}
                }
            }

			// Apply friction impulses.
            {
                const Vec2 impulseSum = impulseArray[0] + impulseArray[1];
                {
                    const Vec2 linearVelocityChangeA = impulseSum * invMassA;
                    velocityXPtr[bodyIndexA] -= linearVelocityChangeA.x;
                    velocityYPtr[bodyIndexA] -= linearVelocityChangeA.y;

                    const Real angularVelocityChangeA = (
                        glm::dot(rAPerpArray[0], impulseArray[0]) +
                        glm::dot(rAPerpArray[1], impulseArray[1])
                        ) * invInertiaA;
                    angularVelocityPtr[bodyIndexA] -= angularVelocityChangeA;
                }

                {
                    const Vec2 linearVelocityChangeB = impulseSum * invMassB;
                    velocityXPtr[bodyIndexB] += linearVelocityChangeB.x;
                    velocityYPtr[bodyIndexB] += linearVelocityChangeB.y;

                    const Real angularVelocityChangeB = (
                        glm::dot(rBPerpArray[0], impulseArray[0]) +
                        glm::dot(rBPerpArray[1], impulseArray[1])
                        ) * invInertiaB;
                    angularVelocityPtr[bodyIndexB] += angularVelocityChangeB;
                }
            }

            // Position and velocity correction.
            const Real invTotalInvMass_x_Depth = depth / totalInvMass;
            const Real correctionStrengthA = invMassA * invTotalInvMass_x_Depth;
            const Real correctionStrengthB = invMassB * invTotalInvMass_x_Depth;
            {
                const Real correctionA = correctionStrengthA * simulationSettings.positionCorrectionPercent;
                const Real correctionB = correctionStrengthB * simulationSettings.positionCorrectionPercent;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                positionXPtr[bodyIndexA] -= correctionAVec.x;
                positionYPtr[bodyIndexA] -= correctionAVec.y;
                positionXPtr[bodyIndexB] += correctionBVec.x;
                positionYPtr[bodyIndexB] += correctionBVec.y;
            }
            if (ENABLE_VELOCITY_CORRECTION)
            {
                const Real correctionA = correctionStrengthA * simulationSettings.velocityCorrectionStrength;
                const Real correctionB = correctionStrengthB * simulationSettings.velocityCorrectionStrength;

                const Vec2 correctionAVec = normal * correctionA;
                const Vec2 correctionBVec = normal * correctionB;

                velocityXPtr[bodyIndexA] -= correctionAVec.x;
                velocityYPtr[bodyIndexA] -= correctionAVec.y;
                velocityXPtr[bodyIndexB] += correctionBVec.x;
                velocityYPtr[bodyIndexB] += correctionBVec.y;
            }
        }
    }

    void Simulation::applyConstraints()
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        const Vec2 newBodyPosition = mainBodyHolder.getPosition() + mainBodyHolder.bodyOffset;
        const Vec2 newBodyVelocity = mainBodyHolder.getVelocity();

        Real* ECSTASY_RESTRICT positionXPtr = bodies.positionX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies.positionY.data();
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();

        positionXPtr[bodyIndex] = newBodyPosition.x;
        positionYPtr[bodyIndex] = newBodyPosition.y;

        velocityXPtr[bodyIndex] = newBodyVelocity.x;
        velocityYPtr[bodyIndex] = newBodyVelocity.y;
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