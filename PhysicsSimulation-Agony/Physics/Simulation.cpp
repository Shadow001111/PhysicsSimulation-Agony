#include "Simulation.h"
#include "Threading.h"
#include "FastCosSin.h"
#include "Constants.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <iostream>
#include <cmath>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Core::Simd<Real>;


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

    static std::pair<Real, Vec2> calculatePolygonInertia(
        Real mass,
        VerticesContainer& verticesContainer,
        std::optional<Vec2> centerOfMass = std::nullopt
    )
    {
        const size_t verticesCount = verticesContainer.size();
        if (verticesCount < 3)
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        Real signedArea = Real(0);
        Real cx = Real(0);
        Real cy = Real(0);
        Real xx = Real(0);
        Real yy = Real(0);

        const bool computeCOM = !centerOfMass.has_value();
        const Vec2* verticesPtr = verticesContainer.data();

        for (size_t i = 0; i < verticesCount; i++)
        {
            const Vec2& p0 = verticesPtr[i];
            const Vec2& p1 = verticesPtr[(i + 1) % verticesCount];

            Real cross = p0.x * p1.y - p1.x * p0.y;
            signedArea += cross;

            if (computeCOM)
            {
                cx += (p0.x + p1.x) * cross;
                cy += (p0.y + p1.y) * cross;
            }

            // Area moments about origin.
            xx += (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) * cross;
            yy += (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) * cross;
        }

        // If winding order is clockwise, reverse the container to make it counter-clockwise.
        // Since all accumulated values are linear with respect to 'cross', we can just negate them.
        if (signedArea < Real(0))
        {
            std::reverse(verticesContainer.begin(), verticesContainer.end());
            signedArea = -signedArea;
            xx = -xx;
            yy = -yy;
            if (computeCOM)
            {
                cx = -cx;
                cy = -cy;
            }
        }

        signedArea *= Real(0.5);
        const Real absoluteArea = signedArea;
        if (absoluteArea < std::numeric_limits<Real>::epsilon())
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        // Determine final Center of Mass.
        Vec2 finalCOM;
        if (computeCOM)
        {
            finalCOM = Vec2(
                cx / (Real(6) * signedArea),
                cy / (Real(6) * signedArea)
            );
        }
        else
        {
            finalCOM = centerOfMass.value();
        }

        xx /= Real(12);
        yy /= Real(12);

        // Local/World polar moment of area scaled to mass moment
        Real inertia = (mass / absoluteArea) * (xx + yy);

        // If COM was explicitly provided, treat vertices as local space 
        // and shift to world origin via the parallel axis theorem.
        if (!computeCOM)
        {
            Real deltaSquared = glm::dot(finalCOM, finalCOM);
            inertia += mass * deltaSquared;
        }

        return { inertia, finalCOM };
    }

    Simulation::Simulation()
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

        const Real fixedDeltaTime = simulationSettings.updateInterval * simulationSettings.timeScale;
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

    std::optional<ObjectIndex> Simulation::createCircle(const CircleCreateParams& params)
    {
        const ObjectIndex newBodyIndex = bodies.getCount();
        const ObjectIndex newShapeIndex = circles.getCount();

        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real radius = std::fmax(Real(0), params.radius);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        const Real inertia = calculateCircleInertia(mass, radius, centerOfMass);

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position, params.base.velocity, params.base.rotation, params.base.angularVelocity,
            mass, invMass, inertia, invInertia, centerOfMass
        );

        const ColliderIndex newCollider = createColliderInternal(
            newBodyIndex, Vec2(0), Real(0), materialIndex, BodyType::Circle, newShapeIndex
        );
        circles.append(newCollider, radius);

        return newBodyIndex;
    }

    std::optional<ObjectIndex> Simulation::createBox(const BoxCreateParams& params)
    {
        const ObjectIndex newBodyIndex = bodies.getCount();
        const ObjectIndex newShapeIndex = boxes.getCount();

        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        const Real inertia = calculateBoxInertia(mass, width, height, centerOfMass);

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass, invMass,
            inertia, invInertia,
            centerOfMass
        );

        const ColliderIndex newCollider = createColliderInternal(
            newBodyIndex, Vec2(0), Real(0), materialIndex, BodyType::Box, newShapeIndex
        );

        boxes.append(
            newCollider,
            width * Real(0.5),
            height * Real(0.5)
        );

        return newBodyIndex;
    }

    std::optional<ObjectIndex> Simulation::createPolygon(const PolygonCreateParams& params)
    {
        if (params.localVertices == nullptr)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices container is nullptr.\n";
            return std::nullopt;
        }
        if (params.verticesCount < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices count is less than three.\n";
            return std::nullopt;
        }
        //if (params.base.centerOfMass.has_value())
        //{
        //    // I just don't know how to make it work with my 'true positions' and other stuff.
        //    std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Custom center of mass is not supported.\n";
        //    return;
        //}

        // Filter out consecutive duplicate vertices.
        std::vector<Vec2> uniqueVertices;
        uniqueVertices.reserve(params.verticesCount);
        for (size_t i = 0; i < params.verticesCount; ++i)
        {
            const Vec2& current = params.localVertices[i];

            // Check the current vertex against all vertices we've already accepted.
            bool isDuplicate = false;
            for (const Vec2& existing : uniqueVertices)
            {
                if (existing.x == current.x && existing.y == current.y)
                {
                    isDuplicate = true;
                    break;
                }
            }

            // Only add it if it hasn't been seen anywhere else yet.
            if (!isDuplicate)
            {
                uniqueVertices.push_back(current);
            }
        }

        if (uniqueVertices.size() < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Unique vertices count is less than three.\n";
            return std::nullopt;
        }

        const ObjectIndex newBodyIndex = bodies.getCount();
        const ObjectIndex newShapeIndex = polygons.getCount();

        const Real mass = std::fmax(Real(0), params.base.mass);
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        VerticesContainer vertices{ uniqueVertices.data(), uniqueVertices.size() };

        auto iCOM = calculatePolygonInertia(mass, vertices, std::nullopt);
        const Real inertia = iCOM.first;
        Vec2 trueCenterOfMass = iCOM.second;
        Vec2 neededCenterOfMass = params.base.centerOfMass.value_or(trueCenterOfMass);

        // This makes: position == true position == world COM.
        // TODO: This probably invalidates inertia. Need second pass.
        for (Vec2& v : vertices)
        {
            v -= trueCenterOfMass;
        }
        neededCenterOfMass -= trueCenterOfMass;

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position + trueCenterOfMass,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass, invMass,
            inertia, invInertia,
            neededCenterOfMass
        );

        const ColliderIndex newCollider = createColliderInternal(
            newBodyIndex, Vec2(0), Real(0), materialIndex, BodyType::Polygon, newShapeIndex
        );

        polygons.append(
            newCollider,
            std::move(vertices)
        );

        return newBodyIndex;
    }

    void Simulation::destroyBody(ObjectIndex bodyIndex)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyIndex >= bodyCount) return;

        if (mainBodyHolder.heldBody.has_value() && mainBodyHolder.heldBody.value() == bodyIndex)
            mainBodyHolderRelease();

        // Cascade-delete constraints (springs, unchanged).
        while (!bodies.attachments[bodyIndex].empty())
        {
            const BodyAttachment attachment = bodies.attachments[bodyIndex].back();
            constraintSystems[static_cast<size_t>(attachment.type)]->removeConstraint(attachment.objectIndex, bodies);
        }

        // Cascade-delete every collider owned by this body.
        while (!bodies.colliderIndices[bodyIndex].empty())
        {
            const ColliderIndex colliderIdx = bodies.colliderIndices[bodyIndex].back();
            const BodyType shapeType = colliders.shapeType[colliderIdx];
            const ObjectIndex shapeIdx = colliders.shapeIndex[colliderIdx];

            // Remove from the underlying shape SoA (swap-remove), fixing up the collider
            // that now occupies shapeIdx (if any) to point at its new slot.
            if (shapeType == BodyType::Circle)
            {
                const size_t last = circles.getCount() - 1;
                if (shapeIdx != last)
                {
                    std::swap(circles.colliderIndices[shapeIdx], circles.colliderIndices[last]);
                    std::swap(circles.radius[shapeIdx], circles.radius[last]);
                    colliders.shapeIndex[circles.colliderIndices[shapeIdx]] = static_cast<ObjectIndex>(shapeIdx);
                }
                circles.colliderIndices.pop_back();
                circles.radius.pop_back();
            }
            else if (shapeType == BodyType::Box)
            {
                const size_t last = boxes.getCount() - 1;
                if (shapeIdx != last)
                {
                    std::swap(boxes.colliderIndices[shapeIdx], boxes.colliderIndices[last]);
                    std::swap(boxes.halfWidth[shapeIdx], boxes.halfWidth[last]);
                    std::swap(boxes.halfHeight[shapeIdx], boxes.halfHeight[last]);
                    colliders.shapeIndex[boxes.colliderIndices[shapeIdx]] = static_cast<ObjectIndex>(shapeIdx);
                }
                boxes.colliderIndices.pop_back();
                boxes.halfWidth.pop_back();
                boxes.halfHeight.pop_back();
            }
            else if (shapeType == BodyType::Polygon)
            {
                const size_t last = polygons.getCount() - 1;
                if (shapeIdx != last)
                {
                    std::swap(polygons.colliderIndices[shapeIdx], polygons.colliderIndices[last]);
                    std::swap(polygons.localVertices[shapeIdx], polygons.localVertices[last]);
                    colliders.shapeIndex[polygons.colliderIndices[shapeIdx]] = static_cast<ObjectIndex>(shapeIdx);
                }
                polygons.colliderIndices.pop_back();
                polygons.localVertices.pop_back();
            }

            // Remove from ColliderSoA itself.
            bodies.removeCollider(bodyIndex, colliderIdx);
            const size_t oldBackCollider = colliders.swapRemove(colliderIdx);
            if (oldBackCollider != colliderIdx)
            {
                // The collider that was swapped into colliderIdx needs its owning body's
                // back-reference (and, if that's the same body, our own worklist) updated.
                const ObjectIndex swappedOwner = colliders.bodyIndex[colliderIdx];
                bodies.removeCollider(swappedOwner, static_cast<ColliderIndex>(oldBackCollider));
                bodies.addCollider(swappedOwner, colliderIdx);

                deletedColliders.emplace_back(
                    static_cast<ObjectIndex>(colliderIdx), static_cast<ObjectIndex>(oldBackCollider)
                );
            }
            else
            {
                deletedColliders.emplace_back(static_cast<ObjectIndex>(colliderIdx), static_cast<ObjectIndex>(colliderIdx));
            }
        }

        // --- Body removal (unchanged below this point, minus bodyType/shapeIndex shape-swap block) ---
        if (bodyIndex != bodyCount - 1)
        {
            const ObjectIndex oldBackIndex = static_cast<ObjectIndex>(bodyCount - 1);

            bodies.swapWithBack(bodyIndex);

            // Fix up colliders owned by the body that got swapped into bodyIndex.
            for (ColliderIndex ci : bodies.colliderIndices[bodyIndex])
                colliders.bodyIndex[ci] = bodyIndex;

            if (mainBodyHolder.heldBody.has_value() && mainBodyHolder.heldBody.value() == oldBackIndex)
                mainBodyHolder.heldBody = bodyIndex;

            for (const BodyAttachment& attachment : bodies.attachments[bodyIndex])
                constraintSystems[static_cast<size_t>(attachment.type)]->remapBodyIndex(attachment.objectIndex, oldBackIndex, bodyIndex);

            deletedBodies.emplace_back(bodyIndex, static_cast<ObjectIndex>(bodyCount - 1));
        }
        else
        {
            deletedBodies.emplace_back(bodyIndex, bodyIndex);
        }

        bodies.popBack();
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
            bodies
        );

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

        const Real* ECSTASY_RESTRICT worldCenterXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT worldCenterYPtr = bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT massPtr = bodies.mass.data();

        // Broad-phase.
        std::vector<ObjectIndex> broadPhaseBodies; // TODO: Get rid of allocation.
        broadPhaseBodies.reserve(128);
        broadPhaseCollisionDetector.fetchBodiesInCircle(grabPosition, MAX_GRAB_DISTANCE, broadPhaseBodies);
        if (broadPhaseBodies.empty()) return;

        // Narrow phase.
        std::vector<std::pair<ObjectIndex, Real>> narrowPhaseBodies; // TODO: Get rid of allocation.
        narrowPhaseBodies.reserve(broadPhaseBodies.size());
        narrowPhaseCollisionDetector.findCollisionsInCircle(broadPhaseBodies, grabPosition, MAX_GRAB_DISTANCE, narrowPhaseBodies);
        if (narrowPhaseBodies.empty()) return;

        // Sort bodies by distance to the surface.
        std::sort(
            narrowPhaseBodies.begin(),
            narrowPhaseBodies.end(),
            [](const auto& a, const auto& b) -> bool
            {
                return a.second < b.second;
            }
        );

        // Get closest body.
        ObjectIndex closestBody;
        bool foundAnyBody = false;
        for (const auto [bodyIndex, distance] : narrowPhaseBodies)
        {
            if (massPtr[bodyIndex] > 0)
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
        const Real cosRot = bodies.rotationCos[closestBody];
        const Real sinRot = bodies.rotationSin[closestBody];

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
        if (bodyIndex >= bodies.getCount()) return;

        mainBodyHolder.heldBody = std::nullopt;
    }

    void Simulation::mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp)
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const ObjectIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        bodies.angularVelocity[bodyIndex] += radiansSpeedUp;
    }

    void Simulation::getBroadPhaseAABBs(std::vector<AABB>& outAABBs) const
    {
        broadPhaseCollisionDetector.fetchAABBs(outAABBs);
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_NC("Physics step", Ecstasy::Core::Color::Orange);

        // Update timer.
        simulationRunTimer += deltaTime;

        // Reset stuff.
        bodyCollisionSolver.reportNoCollisions();

        // Check if any body exist.
        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        // Check if any material exist.
        if (materials.empty()) [[unlikely]]
        {
            std::cerr << "[AGONY][Simulation]: Material count is zero, which should be impossible.\n";
            materials.emplace_back(); // Default material.
        }

        // Set data viewers.
        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(colliders.aabb),
            colliders.bodyIndex.data()
        );

        narrowPhaseCollisionDetector.setDataViewers(
            BodySoAViewer(bodies),
            ColliderSoAViewer(colliders),
            CircleSoAViewer(circles),
            BoxSoAViewer(boxes),
            PolygonSoAViewer(polygons)
        );

        bodyCollisionSolver.setDataViewers(
            bodies,
            //ColliderSoAViewer(colliders),
            materials,
            bodyCollisionPlanner
        );

        springSolver.setDataViewers(
            bodies,
            SpringSoAViewer(springs),
            springPlanner
        );

        // Remap data if body was deleted.
        narrowPhaseCollisionDetector.remapPersistentContactData(deletedBodies);
        deletedBodies.clear();

        // Main stuff.
        integrateVelocities(bodyCount, deltaTime);
        applyBodyHolderConstraint(deltaTime);
        integratePositions(bodyCount, deltaTime);
        wrapRotation();
        computeRotationCosSin();

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

        if (bodyCount > 1)
        {
            // Broad phase.
            const std::vector<ObjectPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions(true);
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
        else
        {
            // TODO: Call manual reset for data that can be displayed.
        }
    }

    void Simulation::preUpdate()
    {
        // Set old positions.
        {
            TRACY_SCOPE_NC("Set old position/rotation/wrap-count", Ecstasy::Core::Color::Black);

            std::copy(bodies.offsetX.begin(), bodies.offsetX.end(), bodies.renderOldOffsetX.begin());
            std::copy(bodies.offsetY.begin(), bodies.offsetY.end(), bodies.renderOldOffsetY.begin());
            std::copy(bodies.rotation.begin(), bodies.rotation.end(), bodies.renderOldRotation.begin());

            std::fill(bodies.renderRotationWrapCount.begin(), bodies.renderRotationWrapCount.end(), Real(0));
        }
    }

    void Simulation::postUpdate()
    {
        // That's for renderer to have actual information.
        computeBodyWorldCenters();
        computeColliderWorldTransforms();
        buildColliderAABBs();
    }

    void Simulation::integrateVelocities(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_NC("Integrate velocities", Ecstasy::Core::Color::Red);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY.data();
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };

        const RealSimd zeros = RealSimd(Real(0));

        size_t i = 0;
        if constexpr (true)
        {
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
        else
        {
            constexpr Real PLANET_RADIUS = 10;
            constexpr Real PLANET_RADIUS_SQUARED = PLANET_RADIUS * PLANET_RADIUS;
            const Real G = 1000;

            const RealSimd gV(G);
            const RealSimd planetRadiusV(PLANET_RADIUS);
            const RealSimd planetRadiusSquaredV(PLANET_RADIUS_SQUARED);

            size_t i = 0;
            //for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            //{
            //    RealSimd velX = RealSimd::load(velocityXPtr + i);
            //    RealSimd velY = RealSimd::load(velocityYPtr + i);
            //    const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            //    const auto movableMask = invMassV != zeros;
            //
            //    RealSimd posX = RealSimd::load(positionXPtr + i);
            //    RealSimd posY = RealSimd::load(positionYPtr + i);
            //
            //    RealSimd r2 = posX * posX + posY * posY + softSqV;
            //
            //    RealSimd accX = -Gv * posX / r2;
            //    RealSimd accY = -Gv * posY / r2;
            //
            //    RealSimd deltaVX = accX * RealSimd(deltaTime);
            //    RealSimd deltaVY = accY * RealSimd(deltaTime);
            //
            //    RealSimd newVelX = velX + deltaVX;
            //    RealSimd newVelY = velY + deltaVY;
            //    velX = RealSimd::blendv(velX, newVelX, movableMask);
            //    velY = RealSimd::blendv(velY, newVelY, movableMask);
            //
            //    velX.store(velocityXPtr + i);
            //    velY.store(velocityYPtr + i);
            //}
            for (; i < bodyCount; i++)
            {
                const Real invMass = invMassPtr[i];
                if (invMass == Real(0.0)) continue;

                const Real posX = positionXPtr[i];
                const Real posY = positionYPtr[i];
                const Real distanceSquared = posX * posX + posY * posY;

                Real accX, accY;
                if (distanceSquared < PLANET_RADIUS_SQUARED)
                {
                    accX = posX / PLANET_RADIUS;
                    accY = posY / PLANET_RADIUS;
                }
                else
                {
                    const Real distance = std::sqrt(distanceSquared);

                    const Real normalX = posX / distance;
                    const Real normalY = posY / distance;

                    const Real radiusRatioSquared = PLANET_RADIUS_SQUARED / distanceSquared;

                    accX = normalX * radiusRatioSquared;
                    accY = normalY * radiusRatioSquared;
                }
                accX *= -G;
                accY *= -G;

                velocityXPtr[i] += accX * deltaTime;
                velocityYPtr[i] += accY * deltaTime;
            }
        }

        // Damp angular velocity.
        if (simulationSettings.angularVelocityDamping >= 0 && simulationSettings.angularVelocityDamping < 1)
        {
            Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();
            Real* ECSTASY_RESTRICT invInertiaPtr = bodies.invInertia.data();

            const Real damping = std::pow(simulationSettings.angularVelocityDamping, deltaTime);
            const RealSimd dampingV(damping);

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                RealSimd angVel = RealSimd::load(angularVelocityPtr + i);

                const RealSimd invInertiaV = RealSimd::load(invInertiaPtr + i);
                const auto movableMask = invInertiaV != zeros;

                RealSimd newAngVel = angVel * dampingV;

                newAngVel = RealSimd::blendv(angVel, newAngVel, movableMask);

                newAngVel.store(angularVelocityPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                const Real invInertia = invInertiaPtr[i];
                const Real movableMask = invInertia != Real(0.0);

                const Real angVel = angularVelocityPtr[i];

                Real newAngVel = angVel * damping;

                newAngVel = movableMask * newAngVel + (Real(1) - movableMask) * angVel;

                angularVelocityPtr[i] = newAngVel;
            }
        }
    }

    void Simulation::integratePositions(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_NC("Intergrate positions", Ecstasy::Core::Color::Blue);

        const RealSimd deltaTimeV{ deltaTime };

        // Position and rotatiob.
        {
            Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
            Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
            Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

            const Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
            const Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
            const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd velX = RealSimd::load(velocityXPtr + i);
                const RealSimd velY = RealSimd::load(velocityYPtr + i);
                const RealSimd angVel = RealSimd::load(angularVelocityPtr + i);

                RealSimd posX = RealSimd::load(positionXPtr + i);
                RealSimd posY = RealSimd::load(positionYPtr + i);
                RealSimd rot = RealSimd::load(rotationPtr + i);

                posX = RealSimd::mulAdd(velX, deltaTimeV, posX);
                posY = RealSimd::mulAdd(velY, deltaTimeV, posY);
                rot = RealSimd::mulAdd(angVel, deltaTimeV, rot);

                posX.store(positionXPtr + i);
                posY.store(positionYPtr + i);
                rot.store(rotationPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                positionXPtr[i] += velocityXPtr[i] * deltaTime;
                positionYPtr[i] += velocityYPtr[i] * deltaTime;
                rotationPtr[i] += angularVelocityPtr[i] * deltaTime;
            }
        }
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
        const size_t count = circles.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build circle AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = circles.colliderIndices.data();
        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabb.maxY.data();

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
        const size_t count = boxes.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build box AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = boxes.colliderIndices.data();
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth.data();
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabb.maxY.data();

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
        const size_t count = polygons.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build polygon AABBs", Ecstasy::Core::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
        const Real* ECSTASY_RESTRICT worldRotationCosPtr = colliders.worldRotationCos.data();
        const Real* ECSTASY_RESTRICT worldRotationSinPtr = colliders.worldRotationSin.data();

        const ColliderIndex* ECSTASY_RESTRICT colliderIndexPtr = polygons.colliderIndices.data();
        const VerticesContainer* ECSTASY_RESTRICT localVertsPtr = polygons.localVertices.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabb.maxY.data();

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

    void Simulation::wrapRotation()
    {
        constexpr size_t LANES = RealSimd::lanes;

        TRACY_SCOPE_NC("Wrap rotation", Ecstasy::Core::Color::Cyan);

        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();
        Real* ECSTASY_RESTRICT rotationWrapCountPtr = bodies.renderRotationWrapCount.data();

        const size_t bodyCount = bodies.getCount();

        const RealSimd oneV(1);
        const RealSimd twoPIV(Constants::TWO_PI);
        const RealSimd invTwoPIV(Real(1) / Constants::TWO_PI);

        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            RealSimd rot = RealSimd::load(rotationPtr + i);
            RealSimd oldWrapCount = RealSimd::load(rotationWrapCountPtr + i);

            RealSimd wrapCount = RealSimd::roundTowardsZero(rot * invTwoPIV);
            rot = rot - wrapCount * twoPIV;

            RealSimd isRotNegativeMask = rot < RealSimd(0);

            rot += isRotNegativeMask & twoPIV;
            wrapCount -= isRotNegativeMask & oneV;

            rot.store(rotationPtr + i);
            (oldWrapCount + wrapCount).store(rotationWrapCountPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            Real rot = rotationPtr[i];
            Real wrapCount = std::trunc(rot * Constants::INV_TWO_PI);
            rot -= wrapCount * Constants::TWO_PI;

            Real isRotNegativeMask = rot < 0;

            rot += isRotNegativeMask * Constants::TWO_PI;
            wrapCount -= isRotNegativeMask; // * Real(1);

            rotationPtr[i] = rot;
            rotationWrapCountPtr[i] += wrapCount;
        }
    }

    void Simulation::computeRotationCosSin()
    {
        TRACY_SCOPE_NC("Compute rotation cos/sin", Ecstasy::Core::Color::Teal);

        FastCosSin::order4CosSin(
            bodies.rotation.data(),
            bodies.rotationCos.data(),
            bodies.rotationSin.data(),
            bodies.getCount()
        );
    }

    void Simulation::computeBodyWorldCenters()
    {
        TRACY_SCOPE_NC("Compute true positions", Ecstasy::Core::Color::Magenta);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies.localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        Real* ECSTASY_RESTRICT worldCenterXPtr = bodies.worldCenterX.data();
        Real* ECSTASY_RESTRICT worldCenterYPtr = bodies.worldCenterY.data();

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

        const Real* ECSTASY_RESTRICT bodyWorldXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT bodyWorldYPtr = bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT bodyCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT bodySinPtr = bodies.rotationSin.data();

        const ObjectIndex* ECSTASY_RESTRICT ownerPtr = colliders.bodyIndex.data();
        const Real* ECSTASY_RESTRICT localOffXPtr = colliders.localOffsetX.data();
        const Real* ECSTASY_RESTRICT localOffYPtr = colliders.localOffsetY.data();

        Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
        Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();

        // Position uses the BODY's rotation only (an offset defined in body-local
        // space); the collider's own local rotation affects its orientation, not
        // where its origin sits relative to the body.
        const size_t colliderCount = colliders.getCount();
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

        const size_t colliderCount = colliders.getCount();
        if (colliderCount == 0) return;

        // 1) worldRotation = body's rotation + collider's fixed local rotation.
        {
            const Real* ECSTASY_RESTRICT bodyRotationPtr = bodies.rotation.data();
            const ObjectIndex* ECSTASY_RESTRICT ownerPtr = colliders.bodyIndex.data();
            const Real* ECSTASY_RESTRICT localRotationPtr = colliders.localRotation.data();
            Real* ECSTASY_RESTRICT worldRotationPtr = colliders.worldRotation.data();

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
        FastCosSin::order4CosSin(
            colliders.worldRotation.data(),
            colliders.worldRotationCos.data(),
            colliders.worldRotationSin.data(),
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

        Real* ECSTASY_RESTRICT worldRotationPtr = colliders.worldRotation.data();
        const size_t colliderCount = colliders.getCount();

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
        if (bodyIndex >= bodies.getCount()) return;

        const Real mass = bodies.mass[bodyIndex];
        if (mass == 0.0) return; // Static objects can't be dragged.

        const Real invMass = bodies.invMass[bodyIndex];
        const Real invInertia = bodies.invInertia[bodyIndex];

        const Real cosRot = bodies.rotationCos[bodyIndex];
        const Real sinRot = bodies.rotationSin[bodyIndex];

        // Calculate current world position of the grab point.
        const Vec2 worldCenter = { bodies.worldCenterX[bodyIndex], bodies.worldCenterY[bodyIndex] };
        const Vec2 localOffset = mainBodyHolder.localBodyOffset;

        const Vec2 rotatedOffset = Vec2(
            localOffset.x * cosRot - localOffset.y * sinRot,
            localOffset.x * sinRot + localOffset.y * cosRot
        );
        const Vec2 worldGrabPoint = worldCenter + rotatedOffset;

        // Calculate vector (worldR) from the World COM to the world Grab Point.
        const Vec2 localCenterOfMass = { bodies.localCenterOfMassX[bodyIndex], bodies.localCenterOfMassY[bodyIndex] };
        const Vec2 localR = localOffset - localCenterOfMass;

        const Vec2 worldR = Vec2(
            localR.x * cosRot - localR.y * sinRot,
            localR.x * sinRot + localR.y * cosRot
        );

        // Target position and velocity.
        const Vec2 targetPosition = mainBodyHolder.getPosition();
        const Vec2 targetVelocity = mainBodyHolder.getVelocity();

        // Calculate grab point velocity on the rotating body.
        const Vec2 linearVelocity = { bodies.velocityX[bodyIndex], bodies.velocityY[bodyIndex] };
        const Real angularVelocity = bodies.angularVelocity[bodyIndex];
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
        bodies.velocityX[bodyIndex] += force.x * invMass * deltaTime;
        bodies.velocityY[bodyIndex] += force.y * invMass * deltaTime;

        // Apply torque.
        if (invInertia > 0.0)
        {
            const Real torque = worldR.x * force.y - worldR.y * force.x;
            bodies.angularVelocity[bodyIndex] += (torque * invInertia) * deltaTime;
        }

        // Apply strong angular velocity damping.
        bodies.angularVelocity[bodyIndex] *= std::pow(Real(0.1), deltaTime);
    }

    Real Simulation::computeBodyTotalKineticEnergy()
    {
        TRACY_SCOPE_N("Compute body total kinetic energy");

        const Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        const Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

        const Real* ECSTASY_RESTRICT massPtr = bodies.mass.data();
        const Real* ECSTASY_RESTRICT inertiaPtr = bodies.inertia.data();

        const size_t bodyCount = bodies.getCount();

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
        data.bodyDataMemoryUsage = sizeof(BodySoA) + bodies.getMemoryUsage();
        data.circleDataMemoryUsage = sizeof(CircleSoA) + circles.getMemoryUsage();
        data.boxDataMemoryUsage = sizeof(BoxSoA) + boxes.getMemoryUsage();
        data.polygonDataMemoryUsage = sizeof(PolygonSoA) + polygons.getMemoryUsage();

        data.springDataMemoryUsage = sizeof(SpringSoA) + springs.getMemoryUsage();

        data.materialDataMemoryUsage = materials.capacity() * sizeof(materials[0]);
        data.broadPhaseDetectorMemoryUsage = broadPhaseCollisionDetector.getMemoryUsage();
        data.narrowPhaseDetectorMemoryUsage = narrowPhaseCollisionDetector.getMemoryUsage();

        data.bodyCollisionSolverMemoryUsage = bodyCollisionSolver.getMemoryUsage();
        data.bodyCollisionPlannerMemoryUsage = bodyCollisionPlanner.getMemoryUsage();

        data.springSolverMemoryUsage = springSolver.getMemoryUsage();
        data.springPlannerMemoryUsage = springPlanner.getMemoryUsage();
    }

    ColliderIndex Simulation::createColliderInternal(ObjectIndex bodyIndex, Vec2 localOffset, Real localRotation, MaterialIndex materialIndex, BodyType shapeType, ObjectIndex shapeIndex)
    {
        const ColliderIndex newCollider = colliders.append(
            bodyIndex, localOffset, localRotation, materialIndex, shapeType, shapeIndex
        );
        bodies.addCollider(bodyIndex, newCollider);
        return newCollider;
    }
}