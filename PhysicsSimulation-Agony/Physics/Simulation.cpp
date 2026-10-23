#include "Simulation.h"
#include "FastCosSin.h"

#include "SimulationImpl/ColliderAABBBuilder.h"
#include "SimulationImpl/LeftOutFunctions.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>
#include <cmath>

// TODO: Deferred command queue. Handlers must not destroy bodies/colliders mid-drain (destroyBody swaps with back,
// so pending events would hold stale indices). Queue create/destroy calls issued from handlers, run them after the
// drain (creates first, then destroys), and report success/failure through callbacks. Maybe stable global ids later.

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;


    Simulation::Simulation() :
        objectManager(materialManager.getMaterials())
    {
        const auto registerSystem = [this](IConstraintSystem& system) { constraintSystems[size_t(system.getType())] = &system; };
        registerSystem(springConstraintSystem);
        registerSystem(jointConstraintSystem);
        registerSystem(rodConstraintSystem);
        registerSystem(prismaticJointConstraintSystem);
        registerSystem(sliderJointConstraintSystem);
        registerSystem(motorConstraintSystem);
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_NC("Simulation update", PS_AGONY::Profiling::Color::Wheat);

        // Delta time check.
        if (deltaTime <= 0) return;

        // Paused: only keep the spatial structures valid for newly created/destroyed colliders.
        if (paused)
        {
            refreshSpatialData();
            return;
        }

        // Physics steps.
        clock.setSettings(simulationSettings.clockSettings);
        const uint32_t stepCount = clock.advance(deltaTime);
        if (stepCount > 0)
        {
            const Real fixedDeltaTime = clock.getFixedDeltaTime();

            preUpdate();
            for (uint32_t i = 0; i < stepCount; ++i)
            {
                physicsStep(fixedDeltaTime);
            }
            postUpdate();
        }
        else
        {
            refreshSpatialData();
        }

        renderAlpha = clock.getRenderAlpha();

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
            runtimeDebugData.updatesSupposedToHappen = std::floor(Real(1.0) / simulationSettings.clockSettings.updateInterval);

            // Track body penetration.
            {
                Vec2 errors{ 0 };
                if (simulationSettings.trackBodyCollisionSolverConstraintErrors)
                {
                    errors = bodyCollisionSolver.computeConstraintErrors();
                }
                runtimeDebugData.bodyCollisionSolverVelocityError = errors.x;
                runtimeDebugData.bodyCollisionSolverPositionError = errors.y;
            }

            // Track body kinetic energy.
            if (simulationSettings.trackBodyKineticEnergySum)
            {
                runtimeDebugData.bodyKineticEnergySum = computeBodyTotalKineticEnergy();
            }
            else
            {
                runtimeDebugData.bodyKineticEnergySum = 0;
            }
        }
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        return materialManager.create(material);
    }

    bool Simulation::destroyMaterial(MaterialIndex materialIndex)
    {
        if (!materialManager.destroy(materialIndex))
            return false;

        MaterialIndex* const colliderMaterials = objectManager.colliders.materialIndex.data();
        const size_t colliderCount = objectManager.colliders.getCount();
        for (size_t i = 0; i < colliderCount; i++)
        {
            if (colliderMaterials[i] == materialIndex)
                colliderMaterials[i] = 0;
        }
        return true;
    }

    std::optional<CollisionCallbackIndex> Simulation::createCollisionCallback(CollisionCallback function, void* userData)
    {
        return collisionCallbackManager.create(function, userData);
    }

    bool Simulation::destroyCollisionCallback(CollisionCallbackIndex callbackIndex)
    {
        if (!collisionCallbackManager.destroy(callbackIndex))
            return false;

        ColliderSoA& colliders = objectManager.colliders;
        const size_t colliderCount = colliders.getCount();
        for (size_t i = 0; i < colliderCount; i++)
        {
            if (colliders.callbackId[i] == callbackIndex)
            {
                colliders.callbackId[i] = 0;
                colliders.eventThreshold[i] = NO_COLLISION_EVENT_THRESHOLD;
            }
        }
        return true;
    }

    bool Simulation::setColliderCollisionCallback(ColliderIndex colliderIndex, CollisionCallbackIndex callbackIndex, Real impulseThreshold)
    {
        ColliderSoA& colliders = objectManager.colliders;
        if (static_cast<size_t>(colliderIndex) >= colliders.getCount())
            return false;

        if (callbackIndex == 0)
        {
            colliders.callbackId[colliderIndex] = 0;
            colliders.eventThreshold[colliderIndex] = NO_COLLISION_EVENT_THRESHOLD;
            return true;
        }

        if (!collisionCallbackManager.isAlive(callbackIndex) || !(impulseThreshold > Real(0)))
            return false;

        colliders.callbackId[colliderIndex] = callbackIndex;
        colliders.eventThreshold[colliderIndex] = impulseThreshold;
        return true;
    }

    void Simulation::dispatchCollisionEvents()
    {
        TRACY_SCOPE_N("Dispatch collision events");

        std::span<const CollisionEvent> events = bodyCollisionSolver.getCollisionEvents();
        collisionCallbackManager.invoke(events);
    }

    std::span<const ColliderIndex> Simulation::getBodyColliders(ObjectIndex bodyIndex)
    {
        if (bodyIndex >= objectManager.bodies.getCount())
        {
            return {};
        }

        std::span<const ColliderIndex> indicesSpan = objectManager.bodies.colliderIndices[bodyIndex];
        return indicesSpan;
    }

    void Simulation::mainBodyHolderGrabAt(Vec2 grabPosition)
    {
        if (mainBodyHolder.heldBody.has_value()) return;

        TRACY_SCOPE_N("Try grab body");

        constexpr Real MAX_GRAB_DISTANCE = 1.0;

        const Real* ECSTASY_RESTRICT worldCenterXPtr = objectManager.bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT worldCenterYPtr = objectManager.bodies.worldCenterY.data();

        // Broad-phase.
        broadPhaseColliders.clear();
        broadPhaseColliders.reserve(128);
        broadPhaseCollisionDetector.queryCollidersInShape(
            BroadPhaseInternal::CircleQuery(grabPosition, MAX_GRAB_DISTANCE),
            BroadPhaseInternal::BvhTreeTypeFlag::Dynamic,
            broadPhaseColliders);
        if (broadPhaseColliders.empty()) return;

        // Narrow phase.
        narrowPhaseColliders.clear();
        narrowPhaseColliders.reserve(broadPhaseColliders.size());
        narrowPhaseCollisionDetector.findCollisionsInCircle(broadPhaseColliders, grabPosition, MAX_GRAB_DISTANCE, narrowPhaseColliders);
        if (narrowPhaseColliders.empty()) return;

        // Find closest body.
        ObjectIndex closestCollider;
        ObjectIndex closestBody;
        Real minDistance = std::numeric_limits<Real>::max();
        for (const auto [colliderIndex, distance] : narrowPhaseColliders)
        {
            if (distance < minDistance)
            {
                minDistance = distance;
                closestCollider = colliderIndex;
            }
        }
        closestBody = objectManager.colliders.bodyIndex[closestCollider];

        // Grab.
        mainBodyHolder.heldBody = closestBody;

        // Grab vector in world space.
        const Vec2 worldGrabVec = grabPosition - Vec2(worldCenterXPtr[closestBody], worldCenterYPtr[closestBody]);

        // Rotate grab vector into the body's local space.
        const Real cosRot = objectManager.bodies.rotationCos[closestBody];
        const Real sinRot = objectManager.bodies.rotationSin[closestBody];

        // Inverse rotation.
        mainBodyHolder.localBodyOffset = Vec2(
            worldGrabVec.x * cosRot + worldGrabVec.y * sinRot,
            -worldGrabVec.x * sinRot + worldGrabVec.y * cosRot
        );
    }

    void Simulation::mainBodyHolderRelease()
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const ObjectIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= objectManager.bodies.getCount()) return;

        mainBodyHolder.heldBody = std::nullopt;
    }

    void Simulation::mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp)
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const ObjectIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= objectManager.bodies.getCount()) return;

        objectManager.bodies.angularVelocity[bodyIndex] += radiansSpeedUp;
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_NC("Physics step", PS_AGONY::Profiling::Color::Orange);

        // Clear data. Unnecessary, but usually it won't get cleared if narrow phase cd will get get reached.
        narrowPhaseCollisionDetector.clearData();

        // Update timer.
        simulationRunTimer += deltaTime;

        // Reset stuff.
        bodyCollisionSolver.reportNoCollisions();
        bodyCollisionSolver.setCollisionEventRecording(
            simulationSettings.collisionEventsEnabled && collisionCallbackManager.hasCallbacks()
        );

        // Check if any body exist.
        const size_t bodyCount = objectManager.bodies.getCount();
        if (bodyCount == 0)
        {
            broadPhaseCollisionDetector.clearData();
            return;
        }

        // Set data viewers.
        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(objectManager.colliders),
            objectManager.colliders.bodyIndex.data()
        );

        narrowPhaseCollisionDetector.setDataViewers(
            BodySoAViewer(objectManager.bodies),
            ColliderSoAViewer(objectManager.colliders),
            CircleSoAViewer(objectManager.circles),
            BoxSoAViewer(objectManager.boxes),
            PolygonSoAViewer(objectManager.polygons),
            CapsuleSoAViewer(objectManager.capsules)
        );

        bodyCollisionSolver.setDataViewers(
            objectManager.bodies,
            ColliderSoAViewer(objectManager.colliders),
            materialManager.getMaterials().data()
        );

        // Remap warm-starting data if a collider was deleted. Persistent contact data is
        // now keyed by collider pairs, not body pairs, so this must use deletedColliders.
        narrowPhaseCollisionDetector.remapPersistentContactData(objectManager.deletedColliders);
        objectManager.deletedColliders.clear();
        objectManager.deletedBodies.clear(); // Nothing currently consumes body-deletion remaps; clear to avoid unbounded growth.

        // Semi-implicit integration + constraint solving + more computing.
        {
            // Integration first phase.
            Integrator::integrateVelocities_SemiImplicit(objectManager.bodies, deltaTime, simulationSettings.integratorSettings);
            Integrator::dampAngularVelocities(objectManager.bodies, deltaTime, simulationSettings.integratorSettings);

            // Solve constraints.
            applyBodyHolderConstraint(deltaTime);
            constraintScheduler.solve(constraintSystems, deltaTime, objectManager.bodies);

            // Integration second phase.
            Integrator::integrateKinematics_SemiImplicit(objectManager.bodies, deltaTime, simulationSettings.integratorSettings);
        }

        // Velocity-verlet integration + constraint solving + more computing.
        //{
        //    const Real halfDT = deltaTime * Real(0.5);
        //
        //    // Integration first phase.
        //    Integrator::integratePhase1_VelocityVerlet(objectManager.bodies, deltaTime, simulationSettings.integratorSettings);
        //    Integrator::dampAngularVelocities(objectManager.bodies, halfDT, simulationSettings.integratorSettings);
        //
        //    // Solve constraints.
        //    applyBodyHolderConstraint(deltaTime);
        //    constraintScheduler.solve(constraintSystems, deltaTime, objectManager.bodies);
        //
        //    // Integration second phase.
        //    Integrator::integratePhase2_VelocityVerlet(objectManager.bodies, deltaTime, simulationSettings.integratorSettings);
        //    Integrator::dampAngularVelocities(objectManager.bodies, halfDT, simulationSettings.integratorSettings);
        //}

        // Computing important data.
        wrapBodyRotations(objectManager.bodies);
        Integrator::computeRotationCosSin(objectManager.bodies);

        // Compute transforms, AABBs and (re)build trees.
        rebuildSpatialData();

        if (objectManager.colliders.getCount() > 1)
        {
            // Broad phase.
            const std::span<const ObjectPair> broad = broadPhaseCollisionDetector.findCollisions(
                ExecutionPolicy::ForceSingleThreaded
            );
            if (broad.empty()) return;

            // Narrow phase.
            const std::span<BodyCollisionData> narrow = narrowPhaseCollisionDetector.findCollisions(
                broad
            );
            if (narrow.empty())
            {
                // findCollisions already emptied allCollisionData.
                narrowPhaseCollisionDetector.updatePersistentContactData();
                return;
            }

            // Collision resolution.
            bodyCollisionSolver.solveCollisions(
                narrow,
                simulationSettings.collisionVelocitySolvingIterations,
                simulationSettings.collisionPositionSolvingIterations
            );

            // Updating persistent contact data.
            narrowPhaseCollisionDetector.updatePersistentContactData();

            // Collision events (only exist if collisions were solved).
            dispatchCollisionEvents();
        }
    }

    void Simulation::rebuildSpatialData()
    {
        computeBodyWorldCenters();
        computeColliderWorldTransforms();
        buildColliderAABBs();

        // Always (re)build/fit the tree, even with 0 or 1 colliders, so rendering queries
        // (fetchAABBs/fetchCollidersInCircle/fetchCollidersInAABB) never see a stale scene.
        broadPhaseCollisionDetector.buildTrees(
            objectManager.getDynamicColliderIDs(),
            objectManager.getStaticColliderIDs(),
            StaticTreeUpdate::Refit);

        objectManager.spatialDirty = false;
    }

    void Simulation::refreshSpatialData()
    {
        if (!objectManager.spatialDirty) return;

        if (objectManager.bodies.getCount() == 0)
        {
            broadPhaseCollisionDetector.clearData();
            objectManager.spatialDirty = false;
            return;
        }

        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(objectManager.colliders),
            objectManager.colliders.bodyIndex.data()
        );

        Integrator::computeRotationCosSin(objectManager.bodies);
        rebuildSpatialData();
    }

    void Simulation::preUpdate()
    {
        // Set old positions.
        {
            TRACY_SCOPE_NC("Set old position/rotation/wrap-count", PS_AGONY::Profiling::Color::Black);

            std::copy(objectManager.bodies.offsetX.begin(), objectManager.bodies.offsetX.end(), objectManager.bodies.renderOldOffsetX.begin());
            std::copy(objectManager.bodies.offsetY.begin(), objectManager.bodies.offsetY.end(), objectManager.bodies.renderOldOffsetY.begin());
            std::copy(objectManager.bodies.rotation.begin(), objectManager.bodies.rotation.end(), objectManager.bodies.renderOldRotation.begin());

            std::fill(objectManager.bodies.renderRotationWrapCount.begin(), objectManager.bodies.renderRotationWrapCount.end(), Real(0));
        }
    }

    void Simulation::postUpdate()
    {
        // That's for renderer to have actual information.
        computeBodyWorldCenters();
        computeColliderWorldTransforms();
        buildColliderAABBs();
    }

    void Simulation::buildColliderAABBs()
    {
        TRACY_SCOPE_NC("Build collider AABBs", PS_AGONY::Profiling::Color::Green);
        buildCircleAABBs(objectManager.colliders, objectManager.circles);
        buildBoxAABBs(objectManager.colliders, objectManager.boxes);
        buildPolygonAABBs(objectManager.colliders, objectManager.polygons);
        buildCapsuleAABBs(objectManager.colliders, objectManager.capsules);
    }

    void Simulation::computeBodyWorldCenters()
    {
        TRACY_SCOPE_NC("Compute true positions", PS_AGONY::Profiling::Color::Magenta);

        const Real* ECSTASY_RESTRICT positionXPtr = objectManager.bodies.offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = objectManager.bodies.offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = objectManager.bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = objectManager.bodies.localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = objectManager.bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = objectManager.bodies.rotationSin.data();

        Real* ECSTASY_RESTRICT worldCenterXPtr = objectManager.bodies.worldCenterX.data();
        Real* ECSTASY_RESTRICT worldCenterYPtr = objectManager.bodies.worldCenterY.data();

        const size_t bodyCount = objectManager.bodies.getCount();

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            const RealSimd positionX = RealSimd::load(positionXPtr + i);
            const RealSimd positionY = RealSimd::load(positionYPtr + i);

            const RealSimd localCenterOfMassX = RealSimd::load(localCenterOfMassXPtr + i);
            const RealSimd localCenterOfMassY = RealSimd::load(localCenterOfMassYPtr + i);

            const RealSimd cosRot = RealSimd::load(rotationCosPtr + i);
            const RealSimd sinRot = RealSimd::load(rotationSinPtr + i);

            const RealSimd truePositionX = RealSimd::negMulAdd(localCenterOfMassX, cosRot, RealSimd::mulAdd(localCenterOfMassY, sinRot, positionX + localCenterOfMassX));
            const RealSimd truePositionY = RealSimd::negMulAdd(localCenterOfMassX, sinRot, RealSimd::negMulAdd(localCenterOfMassY, cosRot, positionY + localCenterOfMassY));

            truePositionX.store(worldCenterXPtr + i);
            truePositionY.store(worldCenterYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real positionX = positionXPtr[i];
            const Real positionY = positionYPtr[i];
            const Real localCenterOfMassX = localCenterOfMassXPtr[i];
            const Real localCenterOfMassY = localCenterOfMassYPtr[i];
            const Real cosRot = rotationCosPtr[i];
            const Real sinRot = rotationSinPtr[i];
            worldCenterXPtr[i] = (positionX + localCenterOfMassX) - (localCenterOfMassX * cosRot - localCenterOfMassY * sinRot);
            worldCenterYPtr[i] = (positionY + localCenterOfMassY) - (localCenterOfMassX * sinRot + localCenterOfMassY * cosRot);
        }
    }

    void Simulation::computeColliderWorldTransforms()
    {
        TRACY_SCOPE_NC("Compute collider world transforms", PS_AGONY::Profiling::Color::Magenta);

        computeColliderWorldPositions();
        computeColliderWorldRotations();
    }

    void Simulation::computeColliderWorldPositions()
    {
        TRACY_SCOPE_N("Compute collider world positions");

        const Real* ECSTASY_RESTRICT bodyWorldXPtr = objectManager.bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT bodyWorldYPtr = objectManager.bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT bodyCosPtr = objectManager.bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT bodySinPtr = objectManager.bodies.rotationSin.data();

        const ObjectIndex* ECSTASY_RESTRICT ownerPtr = objectManager.colliders.bodyIndex.data();
        const Real* ECSTASY_RESTRICT localOffXPtr = objectManager.colliders.localOffsetX.data();
        const Real* ECSTASY_RESTRICT localOffYPtr = objectManager.colliders.localOffsetY.data();

        Real* ECSTASY_RESTRICT worldPosXPtr = objectManager.colliders.worldPosX.data();
        Real* ECSTASY_RESTRICT worldPosYPtr = objectManager.colliders.worldPosY.data();

        // Position uses the BODY's rotation only (an offset defined in body-local
        // space); the collider's own local rotation affects its orientation, not
        // where its origin sits relative to the body.
        const size_t colliderCount = objectManager.colliders.getCount();
        for (size_t i = 0; i < colliderCount; i++)
        {
            const ObjectIndex body = ownerPtr[i];
            const Real offX = localOffXPtr[i];
            const Real offY = localOffYPtr[i];

            const Real bCos = bodyCosPtr[body];
            const Real bSin = bodySinPtr[body];
            const Real bodyWorldX = bodyWorldXPtr[body];
            const Real bodyWorldY = bodyWorldYPtr[body];

            worldPosXPtr[i] = bodyWorldX + (offX * bCos - offY * bSin);
            worldPosYPtr[i] = bodyWorldY + (offX * bSin + offY * bCos);
        }
    }

    void Simulation::computeColliderWorldRotations()
    {
        TRACY_SCOPE_N("Compute collider world rotations");

        const size_t colliderCount = objectManager.colliders.getCount();
        if (colliderCount == 0) return;

        // worldRotation = bodyRotation + localRotation.
        {
            const Real* ECSTASY_RESTRICT bodyRotationPtr = objectManager.bodies.rotation.data();
            const ObjectIndex* ECSTASY_RESTRICT ownerPtr = objectManager.colliders.bodyIndex.data();
            const Real* ECSTASY_RESTRICT localRotationPtr = objectManager.colliders.localRotation.data();
            Real* ECSTASY_RESTRICT worldRotationPtr = objectManager.colliders.worldRotation.data();

            for (size_t i = 0; i < colliderCount; i++)
            {
                const ObjectIndex body = ownerPtr[i];
                worldRotationPtr[i] = bodyRotationPtr[body] + localRotationPtr[i];
            }
        }

        // Wrap rotations to [0, 2pi) range.
        wrapColliderRotations(objectManager.colliders);

        // Compute cos/sin.
        FastCosSin::order5Array(
            objectManager.colliders.worldRotation.data(),
            objectManager.colliders.worldRotationCos.data(),
            objectManager.colliders.worldRotationSin.data(),
            colliderCount
        );
    }

    void Simulation::applyBodyHolderConstraint(Real deltaTime)
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const ObjectIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= objectManager.bodies.getCount()) return;

        const Real mass = objectManager.bodies.mass[bodyIndex];
        if (mass == 0.0) return; // Static objects can't be dragged.

        const Real invMass = objectManager.bodies.invMass[bodyIndex];
        const Real invInertia = objectManager.bodies.invInertia[bodyIndex];

        const Real cosRot = objectManager.bodies.rotationCos[bodyIndex];
        const Real sinRot = objectManager.bodies.rotationSin[bodyIndex];

        // Calculate current world position of the grab point.
        const Vec2 worldCenter = { objectManager.bodies.worldCenterX[bodyIndex], objectManager.bodies.worldCenterY[bodyIndex] };
        const Vec2 localOffset = mainBodyHolder.localBodyOffset;

        const Vec2 rotatedOffset = Vec2(
            localOffset.x * cosRot - localOffset.y * sinRot,
            localOffset.x * sinRot + localOffset.y * cosRot
        );
        const Vec2 worldGrabPoint = worldCenter + rotatedOffset;

        // Calculate vector (worldR) from the World COM to the world Grab Point.
        const Vec2 localCenterOfMass = { objectManager.bodies.localCenterOfMassX[bodyIndex], objectManager.bodies.localCenterOfMassY[bodyIndex] };
        const Vec2 localR = localOffset - localCenterOfMass;

        const Vec2 worldR = Vec2(
            localR.x * cosRot - localR.y * sinRot,
            localR.x * sinRot + localR.y * cosRot
        );

        // Target position and velocity.
        const Vec2 targetPosition = mainBodyHolder.getPosition();
        const Vec2 targetVelocity = mainBodyHolder.getVelocity();

        // Calculate grab point velocity on the rotating body.
        const Vec2 linearVelocity = { objectManager.bodies.velocityX[bodyIndex], objectManager.bodies.velocityY[bodyIndex] };
        const Real angularVelocity = objectManager.bodies.angularVelocity[bodyIndex];
        const Vec2 grabPointVelocity = Vec2(
            linearVelocity.x - angularVelocity * worldR.y,
            linearVelocity.y + angularVelocity * worldR.x
        );

        // PD Controller for target acceleration at the grab point.
        // Scale frequency down dynamically to maintain explicit Euler stability (freq < 0.5 / deltaTime).
        constexpr Real baseFrequency = 30.0;
        const Real maxStableFrequency = Real(0.5) / deltaTime;
        const Real frequency = std::fmin(baseFrequency, maxStableFrequency);

        const Real stiffness = frequency * frequency;
        const Real damping = Real(2.0) * frequency;

        const Vec2 positionError = targetPosition - worldGrabPoint;
        const Vec2 velocityError = targetVelocity - grabPointVelocity;
        const Vec2 desiredAccel = (stiffness * positionError) + (damping * velocityError);

        // Compute K matrix.
        const Real k00 = invMass + (worldR.y * worldR.y) * invInertia;
        const Real k11 = invMass + (worldR.x * worldR.x) * invInertia;
        const Real k01 = -worldR.x * worldR.y * invInertia;

        const Real det = k00 * k11 - k01 * k01;
        Vec2 force(0.0);
        if (det > 0.0)
        {
            const Real invDet = 1.0 / det;
            const Real mEff00 = k11 * invDet;
            const Real mEff11 = k00 * invDet;
            const Real mEff01 = -k01 * invDet;

            force.x = mEff00 * desiredAccel.x + mEff01 * desiredAccel.y;
            force.y = mEff01 * desiredAccel.x + mEff11 * desiredAccel.y;
        }

        // Apply linear force.
        objectManager.bodies.velocityX[bodyIndex] += force.x * invMass * deltaTime;
        objectManager.bodies.velocityY[bodyIndex] += force.y * invMass * deltaTime;

        // Apply torque.
        if (invInertia > 0.0)
        {
            const Real torque = worldR.x * force.y - worldR.y * force.x;
            objectManager.bodies.angularVelocity[bodyIndex] += (torque * invInertia) * deltaTime;
        }

        // Apply strong angular velocity damping.
        objectManager.bodies.angularVelocity[bodyIndex] *= std::pow(Real(0.1), deltaTime);
    }

    Real Simulation::computeBodyTotalKineticEnergy()
    {
        TRACY_SCOPE_N("Compute body total kinetic energy");

        const Real* ECSTASY_RESTRICT velocityXPtr = objectManager.bodies.velocityX.data();
        const Real* ECSTASY_RESTRICT velocityYPtr = objectManager.bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT angularVelocityPtr = objectManager.bodies.angularVelocity.data();

        const Real* ECSTASY_RESTRICT massPtr = objectManager.bodies.mass.data();
        const Real* ECSTASY_RESTRICT inertiaPtr = objectManager.bodies.inertia.data();

        const size_t bodyCount = objectManager.bodies.getCount();

        Real totalKineticEnergy = 0;

        size_t i = 0;
        if (bodyCount >= RealSimd::lanes)
        {
            const RealSimd half{ 0.5 };
            RealSimd totalKineticEnergyV{ 0 };
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd velocityX = RealSimd::load(velocityXPtr + i);
                const RealSimd velocityY = RealSimd::load(velocityYPtr + i);
                const RealSimd angularVelocity = RealSimd::load(angularVelocityPtr + i);

                const RealSimd mass = RealSimd::load(massPtr + i);
                const RealSimd inertia = RealSimd::load(inertiaPtr + i);

                const RealSimd linearVelocitySquared = RealSimd::mulAdd(velocityX, velocityX, velocityY * velocityY);
                const RealSimd angularVelocitySquared = angularVelocity * angularVelocity;

                const RealSimd kineticEnergy = RealSimd::mulAdd(linearVelocitySquared, mass, angularVelocitySquared * inertia) * half;
                totalKineticEnergyV += kineticEnergy;
            }
            totalKineticEnergy += RealSimd::horizontalAdd(totalKineticEnergyV);
        }
        for (; i < bodyCount; i++)
        {
            const Real velocityX = velocityXPtr[i];
            const Real velocityY = velocityYPtr[i];
            const Real angularVelocity = angularVelocityPtr[i];

            const Real mass = massPtr[i];
            const Real inertia = inertiaPtr[i];

            const Real linearVelocitySquared = std::fma(velocityX, velocityX, velocityY * velocityY);
            const Real angularVelocitySquared = angularVelocity * angularVelocity;

            const Real kineticEnergy = std::fma(linearVelocitySquared, mass, angularVelocitySquared * inertia) * Real(0.5);
            totalKineticEnergy += kineticEnergy;
        }
        return totalKineticEnergy;
    }

    void Simulation::collectMemoryUsage(DebugData& data) const
    {
        data.bodyDataMemoryUsage = objectManager.bodies.getMemoryUsage();

        data.colliderDataMemoryUsage = objectManager.colliders.getMemoryUsage();
        data.circleDataMemoryUsage = objectManager.circles.getMemoryUsage();
        data.boxDataMemoryUsage = objectManager.boxes.getMemoryUsage();
        data.polygonDataMemoryUsage = objectManager.polygons.getMemoryUsage();
        data.capsuleDataMemoryUsage = objectManager.capsules.getMemoryUsage();

        data.materialDataMemoryUsage = materialManager.getMemoryUsage();
        data.collisionCallbackMemoryUsage = collisionCallbackManager.getMemoryUsage();
        data.broadPhaseDetectorMemoryUsage = broadPhaseCollisionDetector.getMemoryUsage();
        data.narrowPhaseDetectorMemoryUsage = narrowPhaseCollisionDetector.getMemoryUsage();

        data.bodyCollisionSolverMemoryUsage = bodyCollisionSolver.getMemoryUsage();

        for (size_t i = 0; i < constraintSystems.size(); ++i)
        {
            data.constraintDataMemoryUsage[i] = constraintSystems[i]->getMemoryUsage();
        }

        data.solvingPlannerMemoryUsage = SolvingPlanner::threadLocal().getMemoryUsage() + constraintScheduler.getMemoryUsage();

        // TODO: Add:
        // std::vector<ColliderIndex> broadPhaseColliders;
        // std::vector<std::pair<ColliderIndex, Real>> narrowPhaseColliders;
    }
}