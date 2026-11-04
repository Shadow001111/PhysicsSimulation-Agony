#include "Simulation.h"
#include "FastCosSin.h"

#include "SimulationImpl/ColliderAABBBuilder.h"
#include "SimulationImpl/LeftOutFunctions.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

//#include <iostream>
#include <cmath>

namespace PS_AGONY2D
{
    using RealSimd = Ecstasy::Core::Simd<Real>;


    Simulation::Simulation() :
        objectManager(materialManager.getMaterials(), constraintSystems)
    {
        const auto registerSystem = [this](IConstraintSystem& system) { constraintSystems[size_t(system.getType())] = &system; };
        registerSystem(springConstraintSystem);
        registerSystem(jointConstraintSystem);
        registerSystem(prismaticJointConstraintSystem);
        registerSystem(sliderJointConstraintSystem);
        registerSystem(motorConstraintSystem);
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_NC("Simulation update", PS_AGONY2D::Profiling::Color::Wheat);

        if (deltaTime <= 0) return;

        // Apply queued modifications before anything reads the data.
        flushCommands();

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
            for (uint32_t i = 0; i < stepCount; i++)
            {
                physicsStep(fixedDeltaTime);
                updateSleeping();
                flushCommands();
            }
            postUpdate();
        }
        else
        {
            refreshSpatialData();
        }

        renderAlpha = clock.getRenderAlpha();

        updateDebugData(deltaTime, stepCount);
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
            {
                colliderMaterials[i] = 0;
                objectManager.wakeBody(objectManager.colliders.bodyIndex[i]);
            }
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

    bool Simulation::setColliderCollisionCallback(ObjectDenseIndex colliderIndex, CollisionCallbackIndex callbackIndex, Real impulseThreshold)
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

        if (!collisionCallbackManager.isAlive(callbackIndex) || !(impulseThreshold >= Real(0)))
            return false;

        colliders.callbackId[colliderIndex] = callbackIndex;
        colliders.eventThreshold[colliderIndex] = impulseThreshold;
        return true;
    }

    void Simulation::dispatchCollisionEvents()
    {
        TRACY_SCOPE_N("Dispatch collision events");

        std::span<const CollisionEvent> events = bodyCollisionSolver.getCollisionEvents();
        collisionCallbackManager.invoke(events, *this);
    }

    void Simulation::mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp)
    {
        for (const Interactivity::BodyHolder::HeldBody& held : mainBodyHolder.heldBodies)
        {
            ObjectDenseIndex bodyIndex = objectManager.bodies.indexOfUnchecked(held.body);
            if (bodyIndex >= objectManager.bodies.getCount()) continue;

            bodyIndex = objectManager.wakeBody(bodyIndex);
            objectManager.bodies.angularVelocity[bodyIndex] += radiansSpeedUp;
        }
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_NC("Physics step", PS_AGONY2D::Profiling::Color::Orange);

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
        if (objectManager.bodies.getCount() == 0)
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

        wakeHeldBodies();

        const IndexRange dynamicRange = objectManager.bodies.range(BodyGroup::Dynamic);

        if (simulationSettings.integratorSettings.integratorType == Integrator::IntegratorType::SemiImplicit)
        {
            // Integration first phase.
            Integrator::integrateVelocities_SemiImplicit(objectManager.bodies, dynamicRange, deltaTime, simulationSettings.integratorSettings);
            Integrator::dampAngularVelocities(objectManager.bodies, dynamicRange, deltaTime, simulationSettings.integratorSettings);

            // Solve constraints.
            applyBodyHolderConstraint(deltaTime, mainBodyHolder);
            constraintScheduler.solve(constraintSystems, deltaTime, objectManager.bodies);

            // Integration second phase.
            Integrator::integrateKinematics_SemiImplicit(objectManager.bodies, dynamicRange, deltaTime, simulationSettings.integratorSettings);
        }
        else if (simulationSettings.integratorSettings.integratorType == Integrator::IntegratorType::VelocityVerlet)
        {
            const Real halfDT = deltaTime * Real(0.5);

            // Integration first phase.
            Integrator::integratePhase1_VelocityVerlet(objectManager.bodies, dynamicRange, deltaTime, simulationSettings.integratorSettings);
            Integrator::dampAngularVelocities(objectManager.bodies, dynamicRange, halfDT, simulationSettings.integratorSettings);

            // Solve constraints.
            applyBodyHolderConstraint(deltaTime, mainBodyHolder);
            constraintScheduler.solve(constraintSystems, deltaTime, objectManager.bodies);

            // Integration second phase.
            Integrator::integratePhase2_VelocityVerlet(objectManager.bodies, dynamicRange, deltaTime, simulationSettings.integratorSettings);
            Integrator::dampAngularVelocities(objectManager.bodies, dynamicRange, halfDT, simulationSettings.integratorSettings);
        }

        // Computing important data.
        wrapBodyRotations(objectManager.bodies, dynamicRange);
        Integrator::computeRotationCosSin(objectManager.bodies, dynamicRange);

        // Full pass only after structural changes (create/destroy/sleep/commands); otherwise dynamic bodies only.
        if (objectManager.spatialDirty) rebuildSpatialData();
        else updateDynamicSpatialData();

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
            wakeCollidingBodies(narrow);
        }
    }

    void Simulation::flushCommands()
    {
        objectManager.flushCommands();
        for (auto* constraintSystem : constraintSystems)
        {
            constraintSystem->flushCommands(objectManager.bodies);
        }
    }

    void Simulation::updateDebugData(Real deltaTime, uint32_t stepCount)
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
    }

    void Simulation::rebuildSpatialData()
    {
        computeBodyWorldCenters(objectManager.bodies.liveRange());
        computeColliderWorldPositions();
        computeColliderWorldRotations();
        buildColliderAABBs();

        // Rebuild/refit trees.

        TreesUpdateParams params{};

        params.types[size_t(BroadPhaseInternal::BvhTreeType::Dynamic)] = TreeUpdate::Rebuild;
        params.types[size_t(BroadPhaseInternal::BvhTreeType::Static)] = TreeUpdate::Refit;
        params.types[size_t(BroadPhaseInternal::BvhTreeType::Sleeping)] = TreeUpdate::Rebuild;

        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Dynamic)] = objectManager.getDynamicColliderIDs();
        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Static)] = objectManager.getStaticColliderIDs();
        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Sleeping)] = objectManager.getSleepingColliderIDs();

        broadPhaseCollisionDetector.updateTrees(params);

        objectManager.spatialDirty = false;
    }

    void Simulation::updateDynamicSpatialData()
    {
        computeBodyWorldCenters(objectManager.bodies.range(BodyGroup::Dynamic));
        computeColliderWorldPositions();
        computeColliderWorldRotations();
        buildColliderAABBs();

        TreesUpdateParams params{};

        params.types[size_t(BroadPhaseInternal::BvhTreeType::Dynamic)] = TreeUpdate::Rebuild;
        params.types[size_t(BroadPhaseInternal::BvhTreeType::Static)] = TreeUpdate::None;
        params.types[size_t(BroadPhaseInternal::BvhTreeType::Sleeping)] = TreeUpdate::None;

        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Dynamic)] = objectManager.getDynamicColliderIDs();
        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Static)] = objectManager.getStaticColliderIDs();
        params.ids[size_t(BroadPhaseInternal::BvhTreeType::Sleeping)] = objectManager.getSleepingColliderIDs();

        broadPhaseCollisionDetector.updateTrees(params);
    }

    void Simulation::wakeHeldBodies()
    {
        for (const Interactivity::BodyHolder::HeldBody& held : mainBodyHolder.heldBodies)
        {
            const ObjectDenseIndex bodyIndex = objectManager.bodies.indexOfUnchecked(held.body);
            if (bodyIndex >= objectManager.bodies.getCount()) continue;

            objectManager.wakeBody(bodyIndex);
        }
    }

    // TEST ONLY, NOT A GOOD SOLUTION: one-step wake latency, one island layer per step,
    // no island/graph handling, wake() invalidates body indices in previousCollisionData
    // (bodyA/bodyB become stale after this call), and sleepers are treated as immovable during the solve.
    void Simulation::wakeCollidingBodies(std::span<const BodyCollisionData> narrow)
    {
        TRACY_SCOPE_N("Wake colliding bodies");

        const BodySoA& bodies = objectManager.bodies;
        const IndexRange sleeping = bodies.range(BodyGroup::Sleeping);
        if (sleeping.empty()) return;
        const IndexRange dynamic = bodies.range(BodyGroup::Dynamic);

        wakeCandidates.clear();
        for (const BodyCollisionData& c : narrow)
        {
            const bool aSleep = bodies.groupOf(c.bodyA) == BodyGroup::Sleeping;
            wakeCandidates.push_back(aSleep ? c.bodyA : c.bodyB);
        }
        if (wakeCandidates.empty()) return;

        std::sort(wakeCandidates.begin(), wakeCandidates.end());
        wakeCandidates.erase(std::unique(wakeCandidates.begin(), wakeCandidates.end()), wakeCandidates.end());

        // Descending: a move only swaps with slots at or above the body's index.
        for (size_t k = wakeCandidates.size(); k-- > 0;)
            objectManager.wakeBody(wakeCandidates[k]);
    }

    void Simulation::updateSleeping()
    {
        TRACY_SCOPE_N("Update sleeping");

        BodySoA& bodies = objectManager.bodies;
        const IndexRange range = bodies.range(BodyGroup::Dynamic);
        if (range.empty()) return;

        const Real linearSq = simulationSettings.linearSpeedSleepThreshold * simulationSettings.linearSpeedSleepThreshold;
        const Real angular = simulationSettings.angularSpeedSleepThreshold;

        const Real* ECSTASY_RESTRICT velX = bodies.velocityX.data();
        const Real* ECSTASY_RESTRICT velY = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT angVel = bodies.angularVelocity.data();

        sleepCandidates.clear();
        for (ObjectDenseIndex i = range.begin; i < range.end; i++)
        {
            const Real linearExcess = velX[i] * velX[i] + velY[i] * velY[i] - linearSq;
            const Real angularExcess = std::fabs(angVel[i]) - angular;
            if (std::fmax(linearExcess, angularExcess) <= Real(0))
            {
                sleepCandidates.push_back(i);
            }
        }
        if (sleepCandidates.empty()) return;

        // Held bodies stay awake.
        const auto isHeld = [&](ObjectDenseIndex index)
            {
                for (const Interactivity::BodyHolder::HeldBody& held : mainBodyHolder.heldBodies)
                {
                    if (bodies.indexOfUnchecked(held.body) == index) return true;
                }
                return false;
            };

        // Descending: a move only swaps with slots at or above the body's index, so lower candidates stay valid.
        for (size_t k = sleepCandidates.size(); k-- > 0;)
        {
            const ObjectDenseIndex index = sleepCandidates[k];
            if (isHeld(index)) continue;
            objectManager.sleepBody(index);
        }
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

        Integrator::computeRotationCosSin(objectManager.bodies, objectManager.bodies.liveRange());
        rebuildSpatialData();
    }

    void Simulation::preUpdate()
    {
        // Set old positions.
        {
            TRACY_SCOPE_NC("Set old position/rotation/wrap-count", PS_AGONY2D::Profiling::Color::Black);

            // Only dynamic bodies move. Static/sleeping ones keep render-old == current.
            BodySoA& bodies = objectManager.bodies;
            const size_t n = bodies.range(BodyGroup::Dynamic).end;

            std::copy_n(bodies.offsetX.data(), n, bodies.renderOldOffsetX.data());
            std::copy_n(bodies.offsetY.data(), n, bodies.renderOldOffsetY.data());
            std::copy_n(bodies.rotation.data(), n, bodies.renderOldRotation.data());

            std::fill_n(bodies.renderRotationWrapCount.data(), n, Real(0));
        }
    }

    void Simulation::postUpdate()
    {
        // Commands flushed after the last step may have changed collider lists/shapes: full rebuild, not a dynamic refit.
        if (objectManager.spatialDirty)
        {
            rebuildSpatialData();
            return;
        }

        // That's for renderer to have actual information.
        computeBodyWorldCenters(objectManager.bodies.range(BodyGroup::Dynamic));
        computeColliderWorldPositions();
        computeColliderWorldRotations();
        buildColliderAABBs();

        broadPhaseCollisionDetector.refitTrees(BroadPhaseInternal::BvhTreeTypeFlag::Dynamic);
    }

    void Simulation::buildColliderAABBs()
    {
        TRACY_SCOPE_NC("Build collider AABBs", PS_AGONY2D::Profiling::Color::Green);
        buildCircleAABBs(objectManager.colliders, objectManager.circles);
        buildBoxAABBs(objectManager.colliders, objectManager.boxes);
        buildPolygonAABBs(objectManager.colliders, objectManager.polygons);
        buildCapsuleAABBs(objectManager.colliders, objectManager.capsules);
    }

    void Simulation::computeBodyWorldCenters(IndexRange range)
    {
        TRACY_SCOPE_NC("Compute true positions", PS_AGONY2D::Profiling::Color::Magenta);

        const Real* ECSTASY_RESTRICT positionXPtr = objectManager.bodies.offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = objectManager.bodies.offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = objectManager.bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = objectManager.bodies.localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = objectManager.bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = objectManager.bodies.rotationSin.data();

        Real* ECSTASY_RESTRICT worldCenterXPtr = objectManager.bodies.worldCenterX.data();
        Real* ECSTASY_RESTRICT worldCenterYPtr = objectManager.bodies.worldCenterY.data();

        const size_t bodyCount = range.end;

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

    void Simulation::computeColliderWorldPositions()
    {
        TRACY_SCOPE_N("Compute collider world positions");

        const Real* ECSTASY_RESTRICT bodyWorldXPtr = objectManager.bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT bodyWorldYPtr = objectManager.bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT bodyCosPtr = objectManager.bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT bodySinPtr = objectManager.bodies.rotationSin.data();

        const ObjectDenseIndex* ECSTASY_RESTRICT ownerPtr = objectManager.colliders.bodyIndex.data();
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
            const ObjectDenseIndex body = ownerPtr[i];
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
            const ObjectDenseIndex* ECSTASY_RESTRICT ownerPtr = objectManager.colliders.bodyIndex.data();
            const Real* ECSTASY_RESTRICT localRotationPtr = objectManager.colliders.localRotation.data();
            Real* ECSTASY_RESTRICT worldRotationPtr = objectManager.colliders.worldRotation.data();

            for (size_t i = 0; i < colliderCount; i++)
            {
                const ObjectDenseIndex body = ownerPtr[i];
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

    void Simulation::applyBodyHolderConstraint(Real deltaTime, const Interactivity::BodyHolder& holder)
    {
        if (!holder.isHolding()) return;

        // Shared by all held bodies.
        const Vec2 holderPosition = holder.getPosition();
        const Vec2 targetVelocity = holder.getVelocity();

        // PD Controller for target acceleration at the grab point.
        // Scale frequency down dynamically to maintain explicit Euler stability (freq < 0.5 / deltaTime).
        constexpr Real baseFrequency = 30.0;
        const Real maxStableFrequency = Real(0.5) / deltaTime;
        const Real frequency = std::fmin(baseFrequency, maxStableFrequency);

        const Real stiffness = frequency * frequency;
        const Real damping = Real(2.0) * frequency;
        const Real angularDamping = std::pow(Real(0.1), deltaTime);

        for (const Interactivity::BodyHolder::HeldBody& held : holder.heldBodies)
        {
            const ObjectDenseIndex bodyIndex = objectManager.bodies.indexOfUnchecked(held.body);
            if (bodyIndex >= objectManager.bodies.getCount()) continue;

            const Real invMass = objectManager.bodies.invMass[bodyIndex];
            if (invMass == 0.0) continue; // Static objects can't be dragged.

            const Real invInertia = objectManager.bodies.invInertia[bodyIndex];

            const Real cosRot = objectManager.bodies.rotationCos[bodyIndex];
            const Real sinRot = objectManager.bodies.rotationSin[bodyIndex];

            // Calculate current world position of the grab point.
            const Vec2 worldCenter = { objectManager.bodies.worldCenterX[bodyIndex], objectManager.bodies.worldCenterY[bodyIndex] };
            const Vec2 localOffset = held.localOffset;

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

            // Target position of this body's anchor.
            const Vec2 targetPosition = holderPosition + held.targetOffset;

            // Calculate grab point velocity on the rotating body.
            const Vec2 linearVelocity = { objectManager.bodies.velocityX[bodyIndex], objectManager.bodies.velocityY[bodyIndex] };
            const Real angularVelocity = objectManager.bodies.angularVelocity[bodyIndex];
            const Vec2 grabPointVelocity = Vec2(
                linearVelocity.x - angularVelocity * worldR.y,
                linearVelocity.y + angularVelocity * worldR.x
            );

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
                force.x = (k11 * desiredAccel.x - k01 * desiredAccel.y) * invDet;
                force.y = (k00 * desiredAccel.y - k01 * desiredAccel.x) * invDet;
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
            objectManager.bodies.angularVelocity[bodyIndex] *= angularDamping;
        }
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