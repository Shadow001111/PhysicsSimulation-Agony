#include "ObjectManager.h"
#include "PhysicsGeometry.h"
#include "PolygonBuild.h"

#include "PS_AGONY2D/Limits.h"
#include "PS_AGONY2D/Constants.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace PS_AGONY2D
{
    ObjectManager::ObjectManager(
        const std::vector<Material>& materials,
        ConstraintSystemArray& constraintSystems
    ) :
        materials(materials),
        constraintSystems(constraintSystems)
    {
    }

    std::optional<ObjectDenseIndex> ObjectManager::createBody(const BodyCreateParams& params)
    {
        const Vec2 centerOfMass = params.centerOfMass.value_or(Vec2(0));

        const Real mass = std::fmax(Real(0), params.mass);
        const Real inertia = std::fmax(Real(0), params.inertia);

        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        // No shape attached yet, so there's nothing to derive inertia from: it starts
        // at zero (infinite resistance to rotation change is NOT implied - it just
        // means no angular response until a collider/inertia is set some other way).
        const ObjectDenseIndex newBodyIndex = appendBody(
            params.position, params.velocity, params.rotation, params.angularVelocity,
            mass, invMass, inertia, invInertia, centerOfMass
        );

        return newBodyIndex;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createCircleCollider(const CircleColliderCreateParams& params)
    {
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createCircleCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        const ObjectDenseIndex bodyIndex = wakeBody(params.bodyIndex);

        const Real radius = std::fmax(Real(0), params.radius);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(circles.getCount());

        const ObjectDenseIndex newCollider = createColliderInternal(
            bodyIndex, params.localOffset, params.localRotation, materialIndex, ColliderShapeType::Circle, newShapeIndex
        );
        circles.append(newCollider, radius);

        return newCollider;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createBoxCollider(const BoxColliderCreateParams& params)
    {
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createBoxCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        const ObjectDenseIndex bodyIndex = wakeBody(params.bodyIndex);

        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(boxes.getCount());

        const ObjectDenseIndex newCollider = createColliderInternal(
            bodyIndex, params.localOffset, params.localRotation, materialIndex, ColliderShapeType::Box, newShapeIndex
        );
        boxes.append(newCollider, width * Real(0.5), height * Real(0.5));

        return newCollider;
    }

    std::optional<PolygonColliderCreateResult> ObjectManager::createPolygonCollider(const PolygonColliderCreateParams& params)
    {
        if (params.localVertices == nullptr)
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: vertices pointer is nullptr.\n";
            return std::nullopt;
        }
        if (params.verticesCount < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: vertices count is less than three.\n";
            return std::nullopt;
        }
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        std::optional<std::vector<Vec2>> outline = PhysicsGeometry::preparePolygonOutline({ params.localVertices, params.verticesCount });
        if (!outline)
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: outline is degenerate or self-intersecting.\n";
            return std::nullopt;
        }
        std::vector<Vec2>& uniqueVertices = *outline; // Deduped, no collinear points, CCW, simple.

        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;

        // Concave outlines are split into multiple convex pieces here; an already-convex
        // outline comes back as a single (CCW-normalized) piece at no extra cost.
        std::vector<std::vector<Vec2>> convexPieces = PhysicsGeometry::decomposeConcave(uniqueVertices, params.concaveDecompositionMethod);
        PhysicsGeometry::limitConvexPieceVertexCount(
            convexPieces, MAX_POLYGON_VERTEX_COUNT, params.convexDecompositionMethod
        );

        if (convexPieces.empty())
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: decomposition produced no pieces.\n";
            return std::nullopt;
        }

        const ObjectDenseIndex bodyIndex = wakeBody(params.bodyIndex);
        const ObjectDenseIndex firstCollider = createConvexPolygonColliderInternal(
            bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[0])
        );
        for (size_t i = 1; i < convexPieces.size(); ++i)
        {
            createConvexPolygonColliderInternal(
                bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[i])
            );
        }

        return PolygonColliderCreateResult{ firstCollider, convexPieces.size() };
    }

    std::optional<ObjectDenseIndex> ObjectManager::createCapsuleCollider(const CapsuleColliderCreateParams& params)
    {
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createCapsuleCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        const ObjectDenseIndex bodyIndex = wakeBody(params.bodyIndex);

        Real length = std::fmax(Real(0), params.length);
        Vec2 localOffset = params.localOffset;
        Real localRotation = params.localRotation;
        if (params.localStart && params.localEnd)
        {
            // Endpoints dominate length/localOffset/localRotation. Axis is local +X at rotation 0.
            const Vec2 d = *params.localEnd - *params.localStart;
            length = std::sqrt(d.x * d.x + d.y * d.y);
            localOffset = (*params.localStart + *params.localEnd) * Real(0.5);
            localRotation = std::atan2(d.y, d.x); // 0 when degenerate.
        }

        const Real radius = std::fmax(Real(0), params.radius);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(capsules.getCount());

        const ObjectDenseIndex newCollider = createColliderInternal(
            bodyIndex, localOffset, localRotation, materialIndex, ColliderShapeType::Capsule, newShapeIndex
        );
        capsules.append(newCollider, length * Real(0.5), radius);

        return newCollider;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createCircle(const CircleCreateParams& params)
    {
        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real radius = std::fmax(Real(0), params.radius);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));

        const Real inertia = PhysicsGeometry::calculateCircleInertia(mass, radius, centerOfMass);
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectDenseIndex newBodyIndex = appendBody(
            params.base.position, params.base.velocity, params.base.rotation, params.base.angularVelocity,
            mass, invMass, inertia, invInertia, centerOfMass
        );

        createCircleCollider({
            .bodyIndex = newBodyIndex,
            .localOffset = Vec2(0),
            .localRotation = Real(0),
            .materialIndex = params.base.materialIndex,
            .radius = radius
            });

        return newBodyIndex;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createBox(const BoxCreateParams& params)
    {
        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));

        const Real inertia = PhysicsGeometry::calculateBoxInertia(mass, width, height, centerOfMass);
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectDenseIndex newBodyIndex = appendBody(
            params.base.position, params.base.velocity, params.base.rotation, params.base.angularVelocity,
            mass, invMass, inertia, invInertia, centerOfMass
        );

        createBoxCollider({
            .bodyIndex = newBodyIndex,
            .localOffset = Vec2(0),
            .localRotation = Real(0),
            .materialIndex = params.base.materialIndex,
            .size = Vec2(width, height)
            });

        return newBodyIndex;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createPolygon(const PolygonCreateParams& params)
    {
        if (params.localVertices.data() == nullptr)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices container is nullptr.\n";
            return std::nullopt;
        }
        if (params.localVertices.size() < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices count is less than three.\n";
            return std::nullopt;
        }

        const Real mass = std::fmax(Real(0), params.base.mass);

        // Decompose BEFORE touching storage so a failure leaves no orphan body.
        PhysicsGeometry::PolygonBuild build = PhysicsGeometry::buildPolygon(
            params.localVertices,
            params.concaveDecompositionMethod, mass, params.base.centerOfMass, params.convexDecompositionMethod
        );
        if (!build.ok())
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: "
                << PhysicsGeometry::polygonBuildErrorMessage(build.error) << "\n";
            return std::nullopt;
        }

        const Real inertia = build.inertia;
        const Vec2 trueCenterOfMass = build.trueCenterOfMass;
        const Vec2 neededCenterOfMass = build.neededCenterOfMass;
        std::vector<std::vector<Vec2>>& convexPieces = build.pieces;

        // Body origin == world COM == position + R(rotation) * trueCOM.
        const Real cosR = std::cos(params.base.rotation);
        const Real sinR = std::sin(params.base.rotation);
        const Vec2 worldCOM = params.base.position + Vec2(
            cosR * trueCenterOfMass.x - sinR * trueCenterOfMass.y,
            sinR * trueCenterOfMass.x + cosR * trueCenterOfMass.y
        );

        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectDenseIndex newBodyIndex = appendBody(
            worldCOM,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass, invMass,
            inertia, invInertia,
            neededCenterOfMass
        );

        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        // Split the already de-duplicated/re-centered outline into convex colliders (one
        // piece if it was already convex) via the same helper createPolygonCollider() uses.
        for (std::vector<Vec2>& piece : convexPieces)
        {
            createConvexPolygonColliderInternal(
                newBodyIndex, Vec2(0), Real(0), materialIndex, std::move(piece)
            );
        }

        return newBodyIndex;
    }

    std::optional<ObjectDenseIndex> ObjectManager::createCapsule(const CapsuleCreateParams& params)
    {
        Vec2 position = params.base.position;
        Real rotation = params.base.rotation;
        Real length = std::fmax(Real(0), params.length);
        if (params.start && params.end)
        {
            // World-space endpoints dominate length/position/rotation. Axis is local +X at rotation 0.
            const Vec2 d = *params.end - *params.start;
            length = std::sqrt(d.x * d.x + d.y * d.y);
            position = (*params.start + *params.end) * Real(0.5);
            rotation = std::atan2(d.y, d.x); // 0 when degenerate.
        }

        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real radius = std::fmax(Real(0), params.radius);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));

        const Real inertia = PhysicsGeometry::calculateCapsuleInertia(mass, length, radius, centerOfMass);
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectDenseIndex newBodyIndex = appendBody(
            position, params.base.velocity, rotation, params.base.angularVelocity,
            mass, invMass, inertia, invInertia, centerOfMass
        );

        createCapsuleCollider({
            .bodyIndex = newBodyIndex,
            .localOffset = Vec2(0),
            .localRotation = Real(0),
            .materialIndex = params.base.materialIndex,
            .length = length,
            .radius = radius,
            });

        return newBodyIndex;
    }

    void ObjectManager::destroyBodyImmediate(ObjectDenseIndex bodyIndex)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyIndex >= bodyCount) return;

        // Removing a body from a sleeping island invalidates its rest state: wake the island.
        bodyIndex = wakeBody(bodyIndex);

        // Cascade-delete constraints. Waking the other bodies moves indices (possibly ours), so track by handle.
        const Handle bodyHandle = bodies.handleAt(bodyIndex);
        for (;;)
        {
            bodyIndex = bodies.indexOfUnchecked(bodyHandle);
            const auto& list = bodies.attachments[bodyIndex];
            if (list.empty()) break;
            const BodyAttachment attachment = list.back();
            destroyConstraintImmediate(attachment.type, attachment.objectIndex);
        }
        bodyIndex = bodies.indexOfUnchecked(bodyHandle);

        // Cascade-delete every collider owned by this body.
        while (!bodies.colliderIndices[bodyIndex].empty())
        {
            const ObjectDenseIndex colliderIdx = bodies.colliderIndices[bodyIndex].back();
            destroyColliderInternal(bodyIndex, colliderIdx);
        }

        // Body removal: move to the unused tail (keeps group order), then pop.
        moveBodyToGroup(bodyIndex, BodyGroup::Unused);
        bodies.popBack();
    }

    void ObjectManager::destroyColliderImmediate(ObjectDenseIndex colliderIndex)
    {
        if (colliderIndex >= colliders.getCount()) return;

        const ObjectDenseIndex bodyIndex = wakeBody(colliders.bodyIndex[colliderIndex]);
        destroyColliderInternal(bodyIndex, colliderIndex);
    }

    bool ObjectManager::wakeAndResolvePair(Handle hA, Handle hB, ObjectDenseIndex& a, ObjectDenseIndex& b)
    {
        if (bodies.indexOf(hA) == Handle::NULL_DENSE_INDEX ||
            bodies.indexOf(hB) == Handle::NULL_DENSE_INDEX) return false;

        wakeBody(bodies.indexOfUnchecked(hA));
        wakeBody(bodies.indexOfUnchecked(hB));
        a = bodies.indexOfUnchecked(hA);
        b = bodies.indexOfUnchecked(hB);
        return true;
    }

    template<class System, class Params>
    std::optional<Handle> ObjectManager::createPairConstraint(ConstraintType type, const Params& p)
    {
        ObjectDenseIndex a, b;
        if (!wakeAndResolvePair(p.bodyHandleA, p.bodyHandleB, a, b)) return std::nullopt;

        auto& system = static_cast<System&>(*constraintSystems[size_t(type)]);
        const BodySoA& constBodies = bodies;
        ObjectDenseIndex ci;
        if constexpr (requires { system.createAt(a, b, p, constBodies); }) ci = system.createAt(a, b, p, constBodies);
        else ci = system.createAt(a, b, p);

        bodies.addAttachment(a, type, ci);
        bodies.addAttachment(b, type, ci);
        return system.handleAt(ci);
    }

    template<class System>
    void ObjectManager::destroyConstraintByHandle(ConstraintType type, Handle h)
    {
        const auto& system = static_cast<const System&>(*constraintSystems[size_t(type)]);
        const ObjectDenseIndex i = system.indexOf(h);
        if (i != Handle::NULL_DENSE_INDEX) destroyConstraintImmediate(type, i);
    }

    std::optional<Handle> ObjectManager::createSpring(const SpringConstraintSystem::CreateParams& p)
    {
        return createPairConstraint<SpringConstraintSystem>(ConstraintType::Spring, p);
    }

    std::optional<Handle> ObjectManager::createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& p)
    {
        return createPairConstraint<RevoluteJointConstraintSystem>(ConstraintType::RevoluteJoint, p);
    }

    std::optional<Handle> ObjectManager::createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& p)
    {
        return createPairConstraint<PrismaticJointConstraintSystem>(ConstraintType::PrismaticJoint, p);
    }

    std::optional<Handle> ObjectManager::createSliderJoint(const SliderJointConstraintSystem::CreateParams& p)
    {
        return createPairConstraint<SliderJointConstraintSystem>(ConstraintType::SliderJoint, p);
    }

    std::optional<Handle> ObjectManager::createMotor(const MotorConstraintSystem::CreateParams& p)
    {
        if (bodies.indexOf(p.bodyHandle) == Handle::NULL_DENSE_INDEX) return std::nullopt;

        const ObjectDenseIndex bodyIndex = wakeBody(bodies.indexOfUnchecked(p.bodyHandle));
        auto& motors = static_cast<MotorConstraintSystem&>(*constraintSystems[size_t(ConstraintType::Motor)]);
        const ObjectDenseIndex ci = motors.createAt(bodyIndex, p);
        bodies.addAttachment(bodyIndex, ConstraintType::Motor, ci);
        return motors.handleAt(ci);
    }

    void ObjectManager::destroySpringImmediate(Handle h) { destroyConstraintByHandle<SpringConstraintSystem>(ConstraintType::Spring, h); }
    void ObjectManager::destroyRevoluteJointImmediate(Handle h) { destroyConstraintByHandle<RevoluteJointConstraintSystem>(ConstraintType::RevoluteJoint, h); }
    void ObjectManager::destroyPrismaticJointImmediate(Handle h) { destroyConstraintByHandle<PrismaticJointConstraintSystem>(ConstraintType::PrismaticJoint, h); }
    void ObjectManager::destroySliderJointImmediate(Handle h) { destroyConstraintByHandle<SliderJointConstraintSystem>(ConstraintType::SliderJoint, h); }
    void ObjectManager::destroyMotorImmediate(Handle h) { destroyConstraintByHandle<MotorConstraintSystem>(ConstraintType::Motor, h); }

    void ObjectManager::destroyConstraintImmediate(ConstraintType type, ObjectDenseIndex index)
    {
        IConstraintSystem& system = *constraintSystems[size_t(type)];
        if (index >= system.getCount()) return;

        constexpr ObjectDenseIndex NONE = Handle::NULL_DENSE_INDEX;
        ObjectDenseIndex a, b;
        system.getBodyIndices(index, a, b);

        // Wake first (body indices move, constraint indices do not), then re-read the pair.
        const Handle hA = bodies.handleAt(a);
        const Handle hB = b != NONE ? bodies.handleAt(b) : Handle{};
        wakeBody(a);
        if (b != NONE) wakeBody(bodies.indexOfUnchecked(hB));
        system.getBodyIndices(index, a, b);

        bodies.removeAttachment(a, type, index);
        if (b != NONE) bodies.removeAttachment(b, type, index);

        const ObjectDenseIndex oldBack = system.swapRemove(index);
        if (oldBack == index) return;

        // The old back constraint now lives at 'index': relink on its bodies.
        system.getBodyIndices(index, a, b);
        bodies.removeAttachment(a, type, oldBack);
        bodies.addAttachment(a, type, index);
        if (b != NONE)
        {
            bodies.removeAttachment(b, type, oldBack);
            bodies.addAttachment(b, type, index);
        }
    }

    void ObjectManager::flushConstraintCommands()
    {
        constraintWakeScratch.clear();
        for (IConstraintSystem* system : constraintSystems)
            if (system) system->flushCommands(bodies, constraintWakeScratch);

        for (const Handle h : constraintWakeScratch)
        {
            const ObjectDenseIndex i = bodies.indexOf(h);
            if (i != Handle::NULL_DENSE_INDEX) wakeBody(i);
        }
    }

    ObjectDenseIndex ObjectManager::createColliderInternal(ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation, MaterialIndex materialIndex, ColliderShapeType shapeType, ObjectDenseIndex shapeIndex)
    {
        const ObjectDenseIndex newCollider = colliders.append(
            bodyIndex, localOffset, localRotation, materialIndex, shapeType, shapeIndex
        );
        bodies.addCollider(bodyIndex, newCollider);
        spatialDirty = true;

        std::vector<ObjectDenseIndex>& list = colliderListFor(bodies.groupOf(bodyIndex));
        colliderSlot.resize(colliders.getCount());
        colliderSlot[newCollider] = static_cast<uint32_t>(list.size());
        list.push_back(static_cast<ObjectDenseIndex>(newCollider));

        return newCollider;
    }

    ObjectDenseIndex ObjectManager::createConvexPolygonColliderInternal(
        ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation,
        MaterialIndex materialIndex, std::vector<Vec2> convexVertices
    )
    {
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(polygons.getCount());

        PolygonGeometry geometry{ convexVertices };

        const ObjectDenseIndex newCollider = createColliderInternal(
            bodyIndex, localOffset, localRotation, materialIndex, ColliderShapeType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(geometry));

        return newCollider;
    }

    void ObjectManager::destroyColliderInternal(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIdx)
    {
        spatialDirty = true;

        const ColliderShapeType shapeType = colliders.shapeType[colliderIdx];
        const ObjectDenseIndex shapeIdx = colliders.shapeIndex[colliderIdx];

        // Remove from the underlying shape SoA (swap-remove), fixing up the collider
        // that now occupies shapeIdx (if any) to point at its new slot.
        if (shapeType == ColliderShapeType::Circle)
        {
            const size_t last = circles.getCount() - 1;
            if (shapeIdx != last)
            {
                std::swap(circles.colliderIndices[shapeIdx], circles.colliderIndices[last]);
                std::swap(circles.radius[shapeIdx], circles.radius[last]);
                colliders.shapeIndex[circles.colliderIndices[shapeIdx]] = static_cast<ObjectDenseIndex>(shapeIdx);
            }
            circles.colliderIndices.pop_back();
            circles.radius.pop_back();
        }
        else if (shapeType == ColliderShapeType::Box)
        {
            const size_t last = boxes.getCount() - 1;
            if (shapeIdx != last)
            {
                std::swap(boxes.colliderIndices[shapeIdx], boxes.colliderIndices[last]);
                std::swap(boxes.halfWidth[shapeIdx], boxes.halfWidth[last]);
                std::swap(boxes.halfHeight[shapeIdx], boxes.halfHeight[last]);
                colliders.shapeIndex[boxes.colliderIndices[shapeIdx]] = static_cast<ObjectDenseIndex>(shapeIdx);
            }
            boxes.colliderIndices.pop_back();
            boxes.halfWidth.pop_back();
            boxes.halfHeight.pop_back();
        }
        else if (shapeType == ColliderShapeType::Polygon)
        {
            const size_t last = polygons.getCount() - 1;
            if (shapeIdx != last)
            {
                std::swap(polygons.colliderIndices[shapeIdx], polygons.colliderIndices[last]);
                std::swap(polygons.geometry[shapeIdx], polygons.geometry[last]);
                colliders.shapeIndex[polygons.colliderIndices[shapeIdx]] = static_cast<ObjectDenseIndex>(shapeIdx);
            }
            polygons.colliderIndices.pop_back();
            polygons.geometry.pop_back();
        }
        else if (shapeType == ColliderShapeType::Capsule)
        {
            const size_t last = capsules.getCount() - 1;
            if (shapeIdx != last)
            {
                std::swap(capsules.colliderIndices[shapeIdx], capsules.colliderIndices[last]);
                std::swap(capsules.halfLength[shapeIdx], capsules.halfLength[last]);
                std::swap(capsules.radius[shapeIdx], capsules.radius[last]);
                colliders.shapeIndex[capsules.colliderIndices[shapeIdx]] = static_cast<ObjectDenseIndex>(shapeIdx);
            }
            capsules.colliderIndices.pop_back();
            capsules.halfLength.pop_back();
            capsules.radius.pop_back();
        }

        // Remove from the persistent ID list (swap-remove, O(1)).
        {
            std::vector<ObjectDenseIndex>& list = colliderListFor(bodies.groupOf(bodyIndex));
            const uint32_t slot = colliderSlot[colliderIdx];
            const ObjectDenseIndex lastID = list.back();
            list[slot] = lastID;
            colliderSlot[lastID] = slot;
            list.pop_back();
        }

        // Remove from ColliderSoA itself.
        bodies.removeCollider(bodyIndex, colliderIdx);
        const ObjectDenseIndex oldBackCollider = colliders.swapRemove(colliderIdx);
        if (oldBackCollider != colliderIdx)
        {
            const ObjectDenseIndex swappedOwner = colliders.bodyIndex[colliderIdx];
            bodies.removeCollider(swappedOwner, static_cast<ObjectDenseIndex>(oldBackCollider));
            bodies.addCollider(swappedOwner, colliderIdx);

            // Moved collider: oldBackCollider -> colliderIdx in its ID list.
            std::vector<ObjectDenseIndex>& movedList = colliderListFor(bodies.groupOf(swappedOwner));
            const uint32_t movedSlot = colliderSlot[oldBackCollider];
            movedList[movedSlot] = static_cast<ObjectDenseIndex>(colliderIdx);
            colliderSlot[colliderIdx] = movedSlot;

            // Fix the shape SoA's back-reference: it still points at the collider's old index.
            const ColliderShapeType movedShapeType = colliders.shapeType[colliderIdx];
            const ObjectDenseIndex movedShapeIdx = colliders.shapeIndex[colliderIdx];
            if (movedShapeType == ColliderShapeType::Circle)
                circles.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == ColliderShapeType::Box)
                boxes.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == ColliderShapeType::Polygon)
                polygons.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == ColliderShapeType::Capsule)
                capsules.colliderIndices[movedShapeIdx] = colliderIdx;

            deletedColliders.emplace_back(
                static_cast<ObjectDenseIndex>(colliderIdx), static_cast<ObjectDenseIndex>(oldBackCollider)
            );
        }
        else
        {
            deletedColliders.emplace_back(static_cast<ObjectDenseIndex>(colliderIdx), static_cast<ObjectDenseIndex>(colliderIdx));
        }

        colliderSlot.pop_back();
    }

    namespace
    {
        Real colliderArea(const ObjectManager& om, ObjectDenseIndex c)
        {
            const ObjectDenseIndex s = om.colliders.shapeIndex[c];
            switch (om.colliders.shapeType[c])
            {
            case ColliderShapeType::Circle:
                return PhysicsGeometry::calculateCircleArea(om.circles.radius[s]);
            case ColliderShapeType::Box:
                return PhysicsGeometry::calculateBoxArea(om.boxes.halfWidth[s] * Real(2), om.boxes.halfHeight[s] * Real(2));
            case ColliderShapeType::Capsule:
                return PhysicsGeometry::calculateCapsuleArea(om.capsules.halfLength[s] * Real(2), om.capsules.radius[s]);
            case ColliderShapeType::Polygon:
            {
                const PolygonGeometry& g = om.polygons.geometry[s];
                return PhysicsGeometry::calculatePolygonArea({ g.vertices(), g.size() });
            }
            default:
                return Real(0);
            }
        }

        Real wrapAngle(Real angle)
        {
            Real wrapped = std::fmod(angle, Constants::TWO_PI);
            if (wrapped < Real(0)) wrapped += Constants::TWO_PI;
            return wrapped;
        }
    }

    bool ObjectManager::queueColliderPolygon(Handle collider, std::span<const Vec2> localVertices)
    {
        if (localVertices.size() < 3) return false;

        std::optional<std::vector<Vec2>> outline = PhysicsGeometry::preparePolygonOutline(localVertices);
        if (!outline) return false;
        if (outline->size() > MAX_POLYGON_VERTEX_COUNT) return false;
        if (!PhysicsGeometry::isConvexPolygon(*outline)) return false;

        const uint32_t slot = static_cast<uint32_t>(pendingPolygons.size());
        pendingPolygons.emplace_back(std::move(*outline));
        pushCommand(ObjectCommandType::SetPolygonGeometry, collider, 0, 0, slot);
        return true;
    }

    void ObjectManager::flushCommands()
    {
        if (commands.empty()) return;

        for (const ObjectCommand& command : commands)
        {
            applyCommand(command);
        }
        commands.clear();
        pendingPolygons.clear();
    }

    void ObjectManager::applyCommand(const ObjectCommand& c)
    {
        using T = ObjectCommandType;

        if (c.type < T::DestroyCollider)
        {
            ObjectDenseIndex bodyIndex = bodies.indexOf(c.handle);
            if (bodyIndex == Handle::NULL_DENSE_INDEX) return;

            if (c.type != T::DestroyBody)
                bodyIndex = wakeBody(bodyIndex);

            switch (c.type)
            {
            case T::DestroyBody:
                destroyBodyImmediate(bodyIndex);
                break;
            case T::SetBodyPosition:
                bodies.offsetX[bodyIndex] = c.a;
                bodies.offsetY[bodyIndex] = c.b;
                bodies.renderOldOffsetX[bodyIndex] = c.a; // Teleport: no render interpolation.
                bodies.renderOldOffsetY[bodyIndex] = c.b;
                spatialDirty = true;
                break;

            case T::SetBodyRotation:
            {
                const Real r = wrapAngle(c.a);
                bodies.rotation[bodyIndex] = r;
                bodies.rotationCos[bodyIndex] = std::cos(r);
                bodies.rotationSin[bodyIndex] = std::sin(r);
                bodies.renderOldRotation[bodyIndex] = r;
                bodies.renderRotationWrapCount[bodyIndex] = Real(0);
                spatialDirty = true;
                break;
            }

            case T::SetBodyVelocity:
                bodies.velocityX[bodyIndex] = c.a;
                bodies.velocityY[bodyIndex] = c.b;
                break;

            case T::SetBodyAngularVelocity:
                bodies.angularVelocity[bodyIndex] = c.a;
                break;

            case T::SetBodyMass:
                applyBodyMass(bodyIndex, c.a);
                break;

            case T::SetBodyInertia:
            {
                if (bodies.invMass[bodyIndex] == 0) break;
                const Real inertia = std::fmax(Real(0), c.a);
                bodies.inertia[bodyIndex] = inertia;
                bodies.invInertia[bodyIndex] = inertia > Real(0) ? Real(1) / inertia : Real(0);
                break;
            }

            case T::SetBodyLocalCenterOfMass:
                bodies.localCenterOfMassX[bodyIndex] = c.a;
                bodies.localCenterOfMassY[bodyIndex] = c.b;
                spatialDirty = true; // World center depends on it.
                break;

            case T::RecomputeBodyInertia:
                recomputeBodyInertia(bodyIndex);
                break;

            case T::SetBodyMassFromDensity:
                applyBodyMassFromDensity(bodyIndex, c.a);
                break;

            default:
                break;
            }
            return;
        }

        const ObjectDenseIndex colliderIndex = colliders.indexOf(c.handle);
        if (colliderIndex == Handle::NULL_DENSE_INDEX) return;
        wakeBody(colliders.bodyIndex[colliderIndex]);
        const ObjectDenseIndex shapeIndex = colliders.shapeIndex[colliderIndex];
        const ColliderShapeType shapeType = colliders.shapeType[colliderIndex];

        switch (c.type)
        {
        case T::DestroyCollider:
            destroyColliderImmediate(colliderIndex);
            break;
        case T::SetColliderLocalOffset:
            colliders.localOffsetX[colliderIndex] = c.a;
            colliders.localOffsetY[colliderIndex] = c.b;
            spatialDirty = true;
            break;

        case T::SetColliderLocalRotation:
            colliders.localRotation[colliderIndex] = wrapAngle(c.a);
            spatialDirty = true;
            break;

        case T::SetColliderMaterial:
            colliders.materialIndex[colliderIndex] = c.index < materials.size() ? static_cast<MaterialIndex>(c.index) : MaterialIndex(0);
            break;

        case T::SetCircleRadius:
            if (shapeType != ColliderShapeType::Circle) break;
            circles.radius[shapeIndex] = std::fmax(Real(0), c.a);
            spatialDirty = true;
            break;

        case T::SetBoxSize:
            if (shapeType != ColliderShapeType::Box) break;
            boxes.halfWidth[shapeIndex] = std::fmax(Real(0), c.a) * Real(0.5);
            boxes.halfHeight[shapeIndex] = std::fmax(Real(0), c.b) * Real(0.5);
            spatialDirty = true;
            break;

        case T::SetCapsuleShape:
            if (shapeType != ColliderShapeType::Capsule) break;
            capsules.halfLength[shapeIndex] = std::fmax(Real(0), c.a) * Real(0.5);
            capsules.radius[shapeIndex] = std::fmax(Real(0), c.b);
            spatialDirty = true;
            break;

        case T::SetPolygonGeometry:
            if (shapeType != ColliderShapeType::Polygon) break;
            polygons.geometry[shapeIndex] = std::move(pendingPolygons[c.index]);
            spatialDirty = true;
            break;

        default:
            break;
        }
    }

    void ObjectManager::applyBodyMass(ObjectDenseIndex b, Real newMass)
    {
        newMass = std::fmax(Real(0), newMass);
        const Real oldMass = bodies.mass[b];

        BodyGroup to;
        if (newMass > Real(0))
        {
            Real inertia = bodies.inertia[b];
            if (oldMass > Real(0)) inertia *= newMass / oldMass;

            bodies.mass[b] = newMass;
            bodies.invMass[b] = Real(1) / newMass;
            bodies.inertia[b] = inertia;
            bodies.invInertia[b] = inertia > Real(0) ? Real(1) / inertia : Real(0);

            to = BodyGroup::Dynamic;
        }
        else
        {
            bodies.mass[b] = Real(0);
            bodies.invMass[b] = Real(0);
            bodies.inertia[b] = Real(0);
            bodies.invInertia[b] = Real(0);
            bodies.velocityX[b] = Real(0);
            bodies.velocityY[b] = Real(0);
            bodies.angularVelocity[b] = Real(0);

            to = BodyGroup::Static;
        }

        const BodyGroup from = bodies.groupOf(b);
        if (from != to)
        {
            b = moveBodyToGroup(b, to);
            moveBodyColliders(b, from, to);
        }
    }

    void ObjectManager::applyBodyMassFromDensity(ObjectDenseIndex b, Real density)
    {
        Real totalArea = Real(0);
        for (const ObjectDenseIndex ci : bodies.colliderIndices[b])
            totalArea += colliderArea(*this, ci);
        if (totalArea <= Real(0)) return;

        applyBodyMass(b, std::fmax(Real(0), density) * totalArea);
    }

    void ObjectManager::recomputeBodyInertia(ObjectDenseIndex b)
    {
        const Real mass = bodies.mass[b];
        if (mass <= Real(0)) return;

        const std::vector<ObjectDenseIndex>& list = bodies.colliderIndices[b];

        Real totalArea = Real(0);
        for (const ObjectDenseIndex ci : list)
            totalArea += colliderArea(*this, ci);
        if (totalArea <= Real(0)) return;

        const Vec2 com(bodies.localCenterOfMassX[b], bodies.localCenterOfMassY[b]);
        const Real massPerArea = mass / totalArea;

        // Sum of each collider's inertia about its own centroid plus parallel-axis shift to the body COM.
        Real inertia = Real(0);
        for (const ObjectDenseIndex ci : list)
        {
            const Real area = colliderArea(*this, ci);
            if (area <= Real(0)) continue;

            const Real m = massPerArea * area;
            const ObjectDenseIndex s = colliders.shapeIndex[ci];
            Vec2 centroid(colliders.localOffsetX[ci], colliders.localOffsetY[ci]);
            Real own = Real(0);

            switch (colliders.shapeType[ci])
            {
            case ColliderShapeType::Circle:
                own = PhysicsGeometry::calculateCircleInertia(m, circles.radius[s], Vec2(0));
                break;
            case ColliderShapeType::Box:
                own = PhysicsGeometry::calculateBoxInertia(m, boxes.halfWidth[s] * Real(2), boxes.halfHeight[s] * Real(2), Vec2(0));
                break;
            case ColliderShapeType::Capsule:
                own = PhysicsGeometry::calculateCapsuleInertia(m, capsules.halfLength[s] * Real(2), capsules.radius[s], Vec2(0));
                break;
            case ColliderShapeType::Polygon:
            {
                const PolygonGeometry& g = polygons.geometry[s];
                const auto [polyInertia, polyCentroid] = PhysicsGeometry::calculatePolygonInertia(m, { g.vertices(), g.size() }, std::nullopt);
                own = polyInertia;

                // Polygon vertices live in the collider frame, rotated by the collider's local rotation.
                const Real cr = std::cos(colliders.localRotation[ci]);
                const Real sr = std::sin(colliders.localRotation[ci]);
                centroid += Vec2(cr * polyCentroid.x - sr * polyCentroid.y, sr * polyCentroid.x + cr * polyCentroid.y);
                break;
            }
            default:
                break;
            }

            const Vec2 d = centroid - com;
            inertia += own + m * (d.x * d.x + d.y * d.y);
        }

        bodies.inertia[b] = inertia;
        bodies.invInertia[b] = inertia > Real(0) ? Real(1) / inertia : Real(0);
    }

    std::vector<ObjectDenseIndex>& ObjectManager::colliderListFor(BodyGroup group) noexcept
    {
        switch (group)
        {
        case BodyGroup::Static:   return staticColliderIDs;
        case BodyGroup::Sleeping: return sleepingColliderIDs;
        default:                  return dynamicColliderIDs;
        }
    }

    void ObjectManager::moveBodyColliders(ObjectDenseIndex b, BodyGroup fromGroup, BodyGroup toGroup)
    {
        std::vector<ObjectDenseIndex>& from = colliderListFor(fromGroup);
        std::vector<ObjectDenseIndex>& to = colliderListFor(toGroup);

        for (const ObjectDenseIndex ci : bodies.colliderIndices[b])
        {
            // O(1) swap-remove from the old list.
            const uint32_t slot = colliderSlot[ci];
            const ObjectDenseIndex lastID = from.back();
            from[slot] = lastID;
            colliderSlot[lastID] = slot;
            from.pop_back();

            colliderSlot[ci] = static_cast<uint32_t>(to.size());
            to.push_back(static_cast<ObjectDenseIndex>(ci));
        }
        spatialDirty = true;
    }

    void ObjectManager::swapBodies(ObjectDenseIndex a, ObjectDenseIndex b)
    {
        if (a == b) return;

        bodies.swap(a, b);

        for (const ObjectDenseIndex ci : bodies.colliderIndices[a])
            colliders.bodyIndex[ci] = a;
        for (const ObjectDenseIndex ci : bodies.colliderIndices[b])
            colliders.bodyIndex[ci] = b;

        // Attachments swapped with the bodies: lists at b reference old index a, lists at a reference old index b.
        const auto& attA = bodies.attachments[b];
        const auto& attB = bodies.attachments[a];
        if (attA.empty() && attB.empty()) return;

        // Sentinel pass so constraints touching both bodies are not remapped twice.
        constexpr ObjectDenseIndex TMP = Handle::NULL_DENSE_INDEX;
        for (const auto& at : attA) constraintSystems[static_cast<size_t>(at.type)]->remapBodyIndex(at.objectIndex, a, TMP);
        for (const auto& at : attB) constraintSystems[static_cast<size_t>(at.type)]->remapBodyIndex(at.objectIndex, b, a);
        for (const auto& at : attA) constraintSystems[static_cast<size_t>(at.type)]->remapBodyIndex(at.objectIndex, TMP, b);
    }

    ObjectDenseIndex ObjectManager::moveBodyToGroup(ObjectDenseIndex index, BodyGroup target)
    {
        uint32_t* counts = bodies.groupCounts;
        uint32_t* ranges = bodies.groupStarts;

        uint32_t g = static_cast<uint32_t>(bodies.groupOf(index));
        const uint32_t to = static_cast<uint32_t>(target);

        while (g != to)
        {
            if (g < to)
            {
                // Up: swap with the last body of the group, then the boundary moves under it.
                const ObjectDenseIndex last = bodies.range(static_cast<BodyGroup>(g)).end - 1;
                swapBodies(index, last);
                index = last;
                counts[g]--;
                if (g + 1 < size_t(BodyGroup::Unused)) counts[g + 1]++;
                ranges[g + 1]--;
                g++;
            }
            else
            {
                // Down: swap with the first body of the group, then the boundary moves over it.
                const ObjectDenseIndex first = bodies.range(static_cast<BodyGroup>(g)).begin;
                swapBodies(index, first);
                index = first;
                if (g < size_t(BodyGroup::Unused)) counts[g]--;
                counts[g - 1]++;
                ranges[g]++;
                g--;
            }
        }

        return index;
    }

    ObjectDenseIndex ObjectManager::appendBody(
        Vec2 pos, Vec2 vel, Real rot, Real anglVel,
        Real mass, Real invMass, Real inertia, Real invInertia,
        Vec2 localCenterOfMass
    )
    {
        const ObjectDenseIndex index = bodies.append(
            pos, vel, rot, anglVel, mass, invMass, inertia, invInertia, localCenterOfMass
        );
        return moveBodyToGroup(index, invMass == Real(0) ? BodyGroup::Static : BodyGroup::Dynamic);
    }

    ObjectDenseIndex ObjectManager::sleepBody(ObjectDenseIndex b, uint32_t sleepIslandId)
    {
        if (bodies.groupOf(b) != BodyGroup::Dynamic) [[unlikely]] return b;

        // Callers sleeping a whole island pass its shared id (>= 2). 0 means a standalone body: fresh id.
        if (sleepIslandId == 0) sleepIslandId = newSleepIslandId();
        bodies.sleepIslandId[b] = sleepIslandId;

        // preUpdate no longer refreshes render-old values for this body.
        bodies.renderOldOffsetX[b] = bodies.offsetX[b];
        bodies.renderOldOffsetY[b] = bodies.offsetY[b];
        bodies.renderOldRotation[b] = bodies.rotation[b];
        bodies.renderRotationWrapCount[b] = Real(0);
        bodies.sleepingTimer[b] = 0.0f;

        b = moveBodyToGroup(b, BodyGroup::Sleeping);
        moveBodyColliders(b, BodyGroup::Dynamic, BodyGroup::Sleeping);
        sleptBodies.push_back(bodies.handleAt(b));
        return b;
    }

    // Wakes exactly one body. Callers go through wakeBody / wakeSleepIslands.
    ObjectDenseIndex ObjectManager::wakeOne(ObjectDenseIndex b)
    {
        if (bodies.groupOf(b) != BodyGroup::Sleeping) [[unlikely]] return b;

        bodies.sleepingTimer[b] = 0.0f;
        bodies.sleepIslandId[b] = 0;

        b = moveBodyToGroup(b, BodyGroup::Dynamic);
        moveBodyColliders(b, BodyGroup::Sleeping, BodyGroup::Dynamic);
        wokenBodies.push_back(bodies.handleAt(b));
        return b;
    }

    // Wakes every Sleeping body whose sleepIslandId is in ids (sorted, unique, no zeros).
    void ObjectManager::wakeSleepIslands(std::span<const uint32_t> ids)
    {
        const IndexRange sleeping = bodies.range(BodyGroup::Sleeping);
        if (ids.empty() || sleeping.empty()) return;

        const uint32_t* sleepId = bodies.sleepIslandId.data();

        wakeScratch.clear();
        if (ids.size() == 1)
        {
            const uint32_t id = ids[0];
            for (ObjectDenseIndex i = sleeping.begin; i < sleeping.end; ++i)
                if (sleepId[i] == id) wakeScratch.push_back(i);
        }
        else
        {
            for (ObjectDenseIndex i = sleeping.begin; i < sleeping.end; ++i)
                if (std::binary_search(ids.begin(), ids.end(), sleepId[i])) wakeScratch.push_back(i);
        }

        // Ascending: waking swaps the body with the first Sleeping slot (<= its index), which can never hold
        // a still-pending candidate, so the remaining indices stay valid. (Sleeping is the opposite: descending.)
        for (const ObjectDenseIndex i : wakeScratch)
            wakeOne(i);
    }

    // Wakes the body and its whole sleeping island. Returns the body's new index.
    ObjectDenseIndex ObjectManager::wakeBody(ObjectDenseIndex b)
    {
        if (bodies.groupOf(b) != BodyGroup::Sleeping) [[unlikely]] return b;

        const uint32_t id = bodies.sleepIslandId[b];
        if (id == 0) [[unlikely]] return wakeOne(b);

        // Other members move too, so track this body by handle.
        const Handle handle = bodies.handleAt(b);
        wakeSleepIslands(std::span<const uint32_t>(&id, 1));
        return bodies.indexOfUnchecked(handle);
    }

    std::optional<ColliderData> ObjectManager::getColliderData(ObjectDenseIndex i) const noexcept
    {
        if (i >= colliders.getCount()) return std::nullopt;

        ColliderData d;
        d.bodyHandle = bodies.handleAt(colliders.bodyIndex[i]);
        d.localOffset = Vec2(colliders.localOffsetX[i], colliders.localOffsetY[i]);
        d.localRotation = colliders.localRotation[i];
        d.worldPosition = Vec2(colliders.worldPosX[i], colliders.worldPosY[i]);
        d.worldRotation = colliders.worldRotation[i];
        d.materialIndex = colliders.materialIndex[i];
        d.shapeType = colliders.shapeType[i];
        d.callbackId = colliders.callbackId[i];
        d.eventThreshold = colliders.eventThreshold[i];

        const ObjectDenseIndex s = colliders.shapeIndex[i];
        switch (d.shapeType)
        {
        case ColliderShapeType::Circle:
            d.radius = circles.radius[s];
            break;
        case ColliderShapeType::Box:
            d.halfExtents = Vec2(boxes.halfWidth[s], boxes.halfHeight[s]);
            break;
        case ColliderShapeType::Capsule:
            d.halfLength = capsules.halfLength[s];
            d.radius = capsules.radius[s];
            break;
        case ColliderShapeType::Polygon:
            d.polygon = &polygons.geometry[s];
            break;
        default:
            break;
        }
        return d;
    }
}