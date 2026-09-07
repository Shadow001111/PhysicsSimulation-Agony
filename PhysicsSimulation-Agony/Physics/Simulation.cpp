#include "Simulation.h"
#include "Threading.h"
#include "FastCosSin.h"
#include "Constants.h"

#include "SimulationImpl/PhysicsGeometry.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>
#include <cmath>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;


    Simulation::Simulation() :
        objectManager(materials)
    {
        materials.reserve(16);
        materials.emplace_back(); // Default material.

        constraintSystems[static_cast<size_t>(ConstraintType::Spring)] = &springConstraintSystem;

        // Spawn a thread pool.
        auto& threadPool = Threading::getGlobalThreadPool();
        (void)threadPool;
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_NC("Simulation update", Ecstasy::Core::Color::Wheat);

        // Delta time check.
        if (deltaTime <= 0) return;

        // Cap how much real time this call is allowed to consume.
        const Real cappedDeltaTime = std::fmin(deltaTime, simulationSettings.maxDeltaTimePerUpdateCall);

        // Advance counter.
        updateTimeAccumulator += cappedDeltaTime;

        // Physics steps.
        uint32_t stepCount = std::floor(updateTimeAccumulator / simulationSettings.updateInterval);
        updateTimeAccumulator -= stepCount * simulationSettings.updateInterval;

        const Real fixedDeltaTime = simulationSettings.updateInterval * simulationSettings.integration.timeScale;
        if (stepCount > 0)
        {
            lastStepCount = stepCount;

            preUpdate();
            for (uint32_t i = 0; i < stepCount; i++)
            {
                physicsStep(fixedDeltaTime);
            }
            postUpdate();
        }

        // Render alpha.
        const Real rawAlpha = updateTimeAccumulator / simulationSettings.updateInterval;
        renderAlpha = (static_cast<Real>(lastStepCount - 1) + rawAlpha) / static_cast<Real>(lastStepCount);

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

    void Simulation::createSpring(const SpringCreateParams& params)
    {
        springConstraintSystem.createSpring(
            params.bodyIndexA,
            params.bodyIndexB,
            params.localAnchorA,
            params.localAnchorB,
            params.restLength,
            params.stiffness,
            params.damping,
            objectManager.bodies
        );

        springsWereChanged = true;
    }

    void Simulation::destroySpring(uint32_t springIndex)
    {
        springConstraintSystem.removeConstraint(springIndex, objectManager.bodies);
        springsWereChanged = true;
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

        TRACY_SCOPE_N("Try grab body");

        constexpr Real MAX_GRAB_DISTANCE = 1.0;

        const Real* ECSTASY_RESTRICT worldCenterXPtr = objectManager.bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT worldCenterYPtr = objectManager.bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT massPtr = objectManager.bodies.mass.data();

        // Broad-phase.
        std::vector<ColliderIndex> broadPhaseColliders; // TODO: Get rid of allocation.
        broadPhaseColliders.reserve(128);
        broadPhaseCollisionDetector.fetchCollidersInCircle(grabPosition, MAX_GRAB_DISTANCE, broadPhaseColliders);
        if (broadPhaseColliders.empty()) return;

        // Narrow phase.
        std::vector<std::pair<ColliderIndex, Real>> narrowPhaseColliders; // TODO: Get rid of allocation.
        narrowPhaseColliders.reserve(broadPhaseColliders.size());
        narrowPhaseCollisionDetector.findCollisionsInCircle(broadPhaseColliders, grabPosition, MAX_GRAB_DISTANCE, narrowPhaseColliders);
        if (narrowPhaseColliders.empty()) return;

        // Sort bodies by distance to the surface.
        std::sort(
            narrowPhaseColliders.begin(),
            narrowPhaseColliders.end(),
            [](const auto& a, const auto& b) -> bool
            {
                return a.second < b.second;
            }
        );

        // Get closest body.
        ObjectIndex closestBody;
        bool foundAnyBody = false;
        for (const auto [colliderIndex, distance] : narrowPhaseColliders)
        {
            ObjectIndex bodyIndex = objectManager.colliders.bodyIndex[colliderIndex];
            if (massPtr[bodyIndex] > 0) // Fix for body!
            {
                closestBody = bodyIndex;
                foundAnyBody = true;
                break;
            }
        }
        if (!foundAnyBody) return;

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
        TRACY_SCOPE_NC("Physics step", Ecstasy::Core::Color::Orange);

        // Clear data. Unnecessary, but usually it won't get cleared if narrow phase cd will get get reached.
        narrowPhaseCollisionDetector.clearData();

        // Update timer.
        simulationRunTimer += deltaTime;

        // Reset stuff.
        bodyCollisionSolver.reportNoCollisions();

        // Check if any body exist.
        const size_t bodyCount = objectManager.bodies.getCount();
        if (bodyCount == 0)
        {
            broadPhaseCollisionDetector.clearData();
            return;
        }

        // Check if any material exist.
        if (materials.empty()) [[unlikely]]
        {
            std::cerr << "[AGONY][Simulation]: Material count is zero, which should be impossible.\n";
            materials.emplace_back(); // Default material.
        }

        // Set data viewers.
        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(objectManager.colliders.aabb),
            objectManager.colliders.bodyIndex.data()
        );

        narrowPhaseCollisionDetector.setDataViewers(
            BodySoAViewer(objectManager.bodies),
            ColliderSoAViewer(objectManager.colliders),
            CircleSoAViewer(objectManager.circles),
            BoxSoAViewer(objectManager.boxes),
            PolygonSoAViewer(objectManager.polygons)
        );

        bodyCollisionSolver.setDataViewers(
            objectManager.bodies,
            ColliderSoAViewer(objectManager.colliders),
            materials,
            bodyCollisionPlanner
        );

        springSolver.setDataViewers(
            objectManager.bodies,
            SpringSoAViewer(springs),
            springPlanner
        );

        // Remap warm-starting data if a collider was deleted. Persistent contact data is
        // now keyed by collider pairs, not body pairs, so this must use deletedColliders.
        narrowPhaseCollisionDetector.remapPersistentContactData(objectManager.deletedColliders);
        objectManager.deletedColliders.clear();
        objectManager.deletedBodies.clear(); // Nothing currently consumes body-deletion remaps; clear to avoid unbounded growth.

        // Main stuff.
        Integrator::integrateVelocities(objectManager.bodies, deltaTime, simulationSettings.integration);
        applyBodyHolderConstraint(deltaTime);
        Integrator::integrateKinematics(objectManager.bodies, deltaTime, simulationSettings.integration);
        Integrator::wrapRotation(objectManager.bodies);
        Integrator::computeRotationCosSin(objectManager.bodies);

        // Compute true position for all bodies.
        computeBodyWorldCenters();

        //
        computeColliderWorldTransforms();

        // Rebuild AABBs.
        buildColliderAABBs();

        // Springs.
        springSolver.solveSprings(
            deltaTime,
            simulationSettings.springSolvingIterations,
            springsWereChanged
        );
        springsWereChanged = false;

        // Always (re)build/fit the tree, even with 0 or 1 colliders, so rendering queries
        // (fetchAABBs/fetchCollidersInCircle/fetchCollidersInAABB) never see a stale scene.
        broadPhaseCollisionDetector.buildTree(true);

        if (objectManager.colliders.getCount() > 1)
        {
            // Broad phase.
            const std::vector<ObjectPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions();
            if (broadCollisionData.empty()) return;

            // Narrow phase.
            const std::vector<BodyCollisionData>& narrowCollisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisionData);
            if (narrowCollisionData.empty()) return;

            // Collision resolution.
            bodyCollisionSolver.solveCollisions(
                narrowCollisionData,
                simulationSettings.collisionVelocitySolvingIterations,
                simulationSettings.collisionPositionSolvingIterations
            );

            // Updating persistent contact data.
            narrowPhaseCollisionDetector.updatePersistentContactData();
        }
    }

    void Simulation::preUpdate()
    {
        // Set old positions.
        {
            TRACY_SCOPE_NC("Set old position/rotation/wrap-count", Ecstasy::Core::Color::Black);

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
        TRACY_SCOPE_NC("Build collider AABBs", Ecstasy::Core::Color::Green);
        buildCircleAABBs();
        buildBoxAABBs();
        buildPolygonAABBs();
    }

    void Simulation::buildCircleAABBs()
    {
        const size_t count = objectManager.circles.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build circle AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = objectManager.colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = objectManager.colliders.worldPosY.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = objectManager.circles.colliderIndices.data();
        const Real* ECSTASY_RESTRICT radiusPtr = objectManager.circles.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = objectManager.colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = objectManager.colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = objectManager.colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = objectManager.colliders.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];
            const Real radius = radiusPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];

            aabbMinXPtr[colliderIndex] = x - radius;
            aabbMinYPtr[colliderIndex] = y - radius;
            aabbMaxXPtr[colliderIndex] = x + radius;
            aabbMaxYPtr[colliderIndex] = y + radius;
        }
    }

    void Simulation::buildBoxAABBs()
    {
        const size_t count = objectManager.boxes.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build box AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = objectManager.colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = objectManager.colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = objectManager.colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = objectManager.colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = objectManager.boxes.colliderIndices.data();
        const Real* ECSTASY_RESTRICT halfWidthPtr = objectManager.boxes.halfWidth.data();
        const Real* ECSTASY_RESTRICT halfHeightPtr = objectManager.boxes.halfHeight.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = objectManager.colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = objectManager.colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = objectManager.colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = objectManager.colliders.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];
            const Real halfWidth = halfWidthPtr[i];
            const Real halfHeight = halfHeightPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];
            const Real cos = worldRotationCosPtr[colliderIndex];
            const Real sin = worldRotationSinPtr[colliderIndex];

            const Real absCos = std::fabs(cos);
            const Real absSin = std::fabs(sin);

            const Real ex = absCos * halfWidth + absSin * halfHeight;
            const Real ey = absSin * halfWidth + absCos * halfHeight;

            aabbMinXPtr[colliderIndex] = x - ex;
            aabbMinYPtr[colliderIndex] = y - ey;
            aabbMaxXPtr[colliderIndex] = x + ex;
            aabbMaxYPtr[colliderIndex] = y + ey;
        }
    }

    void Simulation::buildPolygonAABBs()
    {
        const size_t count = objectManager.polygons.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build polygon AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = objectManager.colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = objectManager.colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = objectManager.colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = objectManager.colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = objectManager.polygons.colliderIndices.data();
        const VerticesContainer* ECSTASY_RESTRICT localVertsPtr = objectManager.polygons.localVertices.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = objectManager.colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = objectManager.colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = objectManager.colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = objectManager.colliders.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const ColliderIndex colliderIndex = colliderIndexPtr[i];

            const Real x = worldPosXPtr[colliderIndex];
            const Real y = worldPosYPtr[colliderIndex];
            const Real cos = worldRotationCosPtr[colliderIndex];
            const Real sin = worldRotationSinPtr[colliderIndex];

            const Vec2* verticesPtr = localVertsPtr[i].data();
            const size_t vertexCount = localVertsPtr[i].size();

            Real minX = std::numeric_limits<Real>::max();
            Real minY = std::numeric_limits<Real>::max();
            Real maxX = -std::numeric_limits<Real>::max();
            Real maxY = -std::numeric_limits<Real>::max();

            for (size_t v = 0; v < vertexCount; v++)
            {
                const Vec2 vertex = verticesPtr[v];
                const Real wx = cos * vertex.x - sin * vertex.y;
                const Real wy = sin * vertex.x + cos * vertex.y;
                minX = std::fmin(minX, wx);
                maxX = std::fmax(maxX, wx);
                minY = std::fmin(minY, wy);
                maxY = std::fmax(maxY, wy);
            }

            aabbMinXPtr[colliderIndex] = x + minX;
            aabbMinYPtr[colliderIndex] = y + minY;
            aabbMaxXPtr[colliderIndex] = x + maxX;
            aabbMaxYPtr[colliderIndex] = y + maxY;
        }
    }

    void Simulation::computeBodyWorldCenters()
    {
        TRACY_SCOPE_NC("Compute true positions", Ecstasy::Core::Color::Magenta);

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
        TRACY_SCOPE_NC("Compute collider world transforms", Ecstasy::Core::Color::Magenta);

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
            const Real bCos = bodyCosPtr[body];
            const Real bSin = bodySinPtr[body];

            const Real offX = localOffXPtr[i];
            const Real offY = localOffYPtr[i];

            worldPosXPtr[i] = bodyWorldXPtr[body] + (offX * bCos - offY * bSin);
            worldPosYPtr[i] = bodyWorldYPtr[body] + (offX * bSin + offY * bCos);
        }
    }

    void Simulation::computeColliderWorldRotations()
    {
        TRACY_SCOPE_N("Compute collider world rotations");

        const size_t colliderCount = objectManager.colliders.getCount();
        if (colliderCount == 0) return;

        // 1) worldRotation = body's rotation + collider's fixed local rotation.
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

        // 2) FastCosSin::order4CosSin requires angles in [0, 2pi); body.rotation is
        //    already wrapped, but adding localRotation can push the sum back out
        //    of range, so wrap again before the batch trig call.
        wrapColliderRotations();

        // 3) Batch cos/sin, same approximation/approach bodies use.
        FastCosSin::order4Array(
            objectManager.colliders.worldRotation.data(),
            objectManager.colliders.worldRotationCos.data(),
            objectManager.colliders.worldRotationSin.data(),
            colliderCount
        );
    }

    void Simulation::wrapColliderRotations()
    {
        TRACY_SCOPE_NC("Wrap collider rotations", Ecstasy::Core::Color::Cyan);

        // Unlike bodies.wrapRotation(), no wrap-count bookkeeping is needed here:
        // worldRotation is fully recomputed from scratch every step (body.rotation
        // + fixed localRotation), never integrated/accumulated on its own, so
        // there's nothing to unwrap later for render interpolation.

        constexpr size_t LANES = RealSimd::lanes;

        Real* ECSTASY_RESTRICT worldRotationPtr = objectManager.colliders.worldRotation.data();
        const size_t colliderCount = objectManager.colliders.getCount();

        const RealSimd twoPIV(Constants::TWO_PI);
        const RealSimd invTwoPIV(Real(1) / Constants::TWO_PI);

        size_t i = 0;
        for (; i + LANES <= colliderCount; i += LANES)
        {
            RealSimd rot = RealSimd::load(worldRotationPtr + i);

            RealSimd wrapCount = RealSimd::roundTowardsZero(rot * invTwoPIV);
            rot = rot - wrapCount * twoPIV;

            RealSimd isRotNegativeMask = rot < RealSimd(0);
            rot += isRotNegativeMask & twoPIV;

            rot.store(worldRotationPtr + i);
        }
        for (; i < colliderCount; i++)
        {
            Real rot = worldRotationPtr[i];
            Real wrapCount = std::trunc(rot * Constants::INV_TWO_PI);
            rot -= wrapCount * Constants::TWO_PI;

            if (rot < Real(0)) rot += Constants::TWO_PI;

            worldRotationPtr[i] = rot;
        }
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
        // Memory
        data.bodyDataMemoryUsage = sizeof(BodySoA) + objectManager.bodies.getMemoryUsage();
        data.colliderDataMemoryUsage = sizeof(ColliderSoA) + objectManager.colliders.getMemoryUsage();
        data.circleDataMemoryUsage = sizeof(CircleSoA) + objectManager.circles.getMemoryUsage();
        data.boxDataMemoryUsage = sizeof(BoxSoA) + objectManager.boxes.getMemoryUsage();
        data.polygonDataMemoryUsage = sizeof(PolygonSoA) + objectManager.polygons.getMemoryUsage();

        data.springDataMemoryUsage = sizeof(SpringSoA) + springs.getMemoryUsage();

        data.materialDataMemoryUsage = materials.capacity() * sizeof(materials[0]);
        data.broadPhaseDetectorMemoryUsage = broadPhaseCollisionDetector.getMemoryUsage();
        data.narrowPhaseDetectorMemoryUsage = narrowPhaseCollisionDetector.getMemoryUsage();

        data.bodyCollisionSolverMemoryUsage = bodyCollisionSolver.getMemoryUsage();
        data.bodyCollisionPlannerMemoryUsage = bodyCollisionPlanner.getMemoryUsage();

        data.springSolverMemoryUsage = springSolver.getMemoryUsage();
        data.springPlannerMemoryUsage = springPlanner.getMemoryUsage();
    }
}