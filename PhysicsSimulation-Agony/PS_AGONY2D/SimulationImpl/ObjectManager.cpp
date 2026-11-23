#include "ObjectManager.h"
#include "PhysicsGeometry.h"
#include "PolygonBuild.h"

#include "PS_AGONY2D/Limits.h"
#include "PS_AGONY2D/Constants.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ranges>

namespace PS_AGONY2D
{
    namespace
    {
        void refreshComOffset(const BodySoA& bodies, ColliderSoA& colliders, ObjectDenseIndex collider)
        {
            const ObjectDenseIndex body = colliders.bodyIndex[collider];
            colliders.comOffsetX[collider] = colliders.localOffsetX[collider] - bodies.localCenterOfMassX[body];
            colliders.comOffsetY[collider] = colliders.localOffsetY[collider] - bodies.localCenterOfMassY[body];
        }

        // 'ascendingIndices' must be sorted ascending. Visits the body nearest the Dynamic/Sleeping boundary first,
        // because each adjacent move swaps with the boundary-side body of the source group.
        template<bool TO_SLEEP, class Func>
        void forEachNearestBoundaryFirst(std::span<const ObjectDenseIndex> ascendingIndices, Func&& func)
        {
            constexpr bool DESCENDING = TO_SLEEP == SLEEPING_AFTER_DYNAMIC;
            if constexpr (DESCENDING)
            {
                for (size_t k = ascendingIndices.size(); k-- > 0;) func(ascendingIndices[k]);
            }
            else
            {
                for (const ObjectDenseIndex index : ascendingIndices) func(index);
            }
        }
    }

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
        colliders.minExtent[newCollider] = radius;
        colliders.maxExtent[newCollider] = radius;

        return placeNewCollider(newCollider);
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
        const Real halfWidth = width * Real(0.5);
        const Real halfHeight = height * Real(0.5);
        boxes.append(newCollider, halfWidth, halfHeight);
        colliders.minExtent[newCollider] = std::fmin(halfWidth, halfHeight);
        colliders.maxExtent[newCollider] = std::fmax(halfWidth, halfHeight);

        return placeNewCollider(newCollider);
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
        // Placing later pieces moves colliders between groups, so track the first piece by handle.
        const Handle firstHandle = colliders.handleAt(createConvexPolygonColliderInternal(
            bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[0])
        ));
        for (size_t i = 1; i < convexPieces.size(); ++i)
        {
            createConvexPolygonColliderInternal(
                bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[i])
            );
        }

        // NOTE: pieces are no longer guaranteed to have consecutive dense indices (grouping moves colliders).
        return PolygonColliderCreateResult{ colliders.indexOfUnchecked(firstHandle), convexPieces.size() };
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
        colliders.minExtent[newCollider] = radius;
        colliders.maxExtent[newCollider] = length * Real(0.5) + radius;

        return placeNewCollider(newCollider);
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

        // The outline is centered on the true COM, so the body origin sits there.
        const Real cosR = std::cos(params.base.rotation);
        const Real sinR = std::sin(params.base.rotation);
        const Vec2 origin = params.base.position + Vec2(
            cosR * trueCenterOfMass.x - sinR * trueCenterOfMass.y,
            sinR * trueCenterOfMass.x + cosR * trueCenterOfMass.y
        );

        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectDenseIndex newBodyIndex = appendBody(
            origin,
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
        const Handle handle = system.handleAt(ci);
        refreshConstraintGroup(type, ci);
        return handle;
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
        const Handle handle = motors.handleAt(ci);
        refreshConstraintGroup(ConstraintType::Motor, ci);
        return handle;
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

        // Remove attachments(Constraint from bodies' lists).
        bodies.removeAttachment(a, type, index);
        bodies.removeAttachment(b, type, index);

        // Delete constraint.
        moveConstraintToGroup(type, index, ConstraintGroup::Unused);
        system.popBack();

        // Wake body a. Then wake body b by handle.
        const Handle hB = b != NONE ? bodies.handleAt(b) : Handle{};
        wakeBody(a);
        if (b != NONE) wakeBody(bodies.indexOfUnchecked(hB));
    }

    ObjectDenseIndex ObjectManager::createColliderInternal(ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation, MaterialIndex materialIndex, ColliderShapeType shapeType, ObjectDenseIndex shapeIndex)
    {
        const ObjectDenseIndex newCollider = colliders.append(
            bodyIndex, localOffset, localRotation, materialIndex, shapeType, shapeIndex
        );
        refreshComOffset(bodies, colliders, newCollider);
        bodies.addCollider(bodyIndex, newCollider);
        spatialDirty = true;

        // The caller appends the shape, then calls placeNewCollider (moves collider and shape into the body's group).
        return newCollider;
    }

    ObjectDenseIndex ObjectManager::createConvexPolygonColliderInternal(
        ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation,
        MaterialIndex materialIndex, std::vector<Vec2> convexVertices
    )
    {
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(polygons.getCount());

        // Recenter on the piece centroid (always inside a convex piece) and move the shift into localOffset.
        // Guarantees origin-inside, which CCD relies on (minExtent = in-radius about the origin).
        // Collider frame -> body frame: localOffset + R(localRotation) * v.
        const Vec2 centroid = PhysicsGeometry::calculatePolygonInertia(Real(1), convexVertices, std::nullopt).second;
        for (Vec2& v : convexVertices) v -= centroid;
        const Real cosL = std::cos(localRotation);
        const Real sinL = std::sin(localRotation);
        localOffset += Vec2(cosL * centroid.x - sinL * centroid.y, sinL * centroid.x + cosL * centroid.y);

        PolygonGeometry geometry{ convexVertices };
        const Real minExtent = PhysicsGeometry::calculatePolygonInnerRadius(geometry);
        const Real maxExtent = PhysicsGeometry::calculatePolygonOuterRadius(geometry);

        const ObjectDenseIndex newCollider = createColliderInternal(
            bodyIndex, localOffset, localRotation, materialIndex, ColliderShapeType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(geometry));
        colliders.minExtent[newCollider] = minExtent;
        colliders.maxExtent[newCollider] = maxExtent;

        return placeNewCollider(newCollider);
    }

    void ObjectManager::markStaticColliderEdited(ObjectDenseIndex colliderIndex)
    {
        if (!bodies.inGroup(colliders.bodyIndex[colliderIndex], BodyGroup::Static)) return;
        editedStaticColliders.push_back(colliders.handles.handleAt(colliderIndex));
    }

    void ObjectManager::markStaticBodyEdited(ObjectDenseIndex bodyIndex)
    {
        if (!bodies.inGroup(bodyIndex, BodyGroup::Static)) return;
        for (const ObjectDenseIndex colliderIndex : bodies.colliderIndices[bodyIndex])
            editedStaticColliders.push_back(colliders.handles.handleAt(colliderIndex));
    }

    void ObjectManager::destroyColliderInternal(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIdx)
    {
        markStaticColliderEdited(colliderIdx);
        spatialDirty = true;

        // Move to the Unused tail (collider and shape both become the last element), then pop. No swap-remove fixups needed.
        const ColliderShapeType shapeType = colliders.shapeType[colliderIdx];
        colliderIdx = moveColliderToGroup(colliderIdx, BodyGroup::Unused);

        bodies.removeCollider(bodyIndex, colliderIdx);
        popBackShape(shapeType);
        colliders.popBack();
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
        // NOTE: on apply the geometry is recentered on its centroid and the shift is added to the collider's local offset.

        const uint32_t slot = static_cast<uint32_t>(pendingPolygons.size());
        pendingPolygons.emplace_back(std::move(*outline));
        pushCommand(ObjectCommandType::SetPolygonGeometry, collider, 0, 0, slot);
        return true;
    }

    void ObjectManager::flushCommands()
    {
        // Bodies/colliders.
        if (!commands.empty())
        {
            for (const ObjectCommand& command : commands)
            {
                applyCommand(command);
            }
            commands.clear();
            pendingPolygons.clear();
        }
        // Constraints.
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
    }

    void ObjectManager::applyCommand(const ObjectCommand& c)
    {
        using T = ObjectCommandType;

        if (c.type < T::DestroyCollider)
        {
            ObjectDenseIndex bodyIndex = bodies.indexOf(c.handle);
            if (bodyIndex == Handle::NULL_DENSE_INDEX) return;

            switch (c.type)
            {
            case T::SetBodyPosition:
            case T::SetBodyRotation:
            case T::SetBodyMass:
            case T::SetBodyLocalCenterOfMass:
            case T::SetBodyMassFromDensity:
                markStaticBodyEdited(bodyIndex);
                break;
            default:
                break;
            }

            if (c.type != T::DestroyBody)
                bodyIndex = wakeBody(bodyIndex);

            switch (c.type)
            {
            case T::DestroyBody:
                destroyBodyImmediate(bodyIndex);
                break;
            case T::SetBodyPosition:
            {
                const Real r = bodies.rotation[bodyIndex];
                const Real cosR = std::cos(r);
                const Real sinR = std::sin(r);
                const Real lx = bodies.localCenterOfMassX[bodyIndex];
                const Real ly = bodies.localCenterOfMassY[bodyIndex];
                const Real x = c.a + cosR * lx - sinR * ly;
                const Real y = c.b + sinR * lx + cosR * ly;
                bodies.comX[bodyIndex] = x;
                bodies.comY[bodyIndex] = y;
                bodies.renderOldComX[bodyIndex] = x; // Teleport: no render interpolation.
                bodies.renderOldComY[bodyIndex] = y;
                spatialDirty = true;
                break;
            }

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
            {
                // The body origin stays in place, so the world COM moves.
                const Real r = bodies.rotation[bodyIndex];
                const Real cosR = std::cos(r);
                const Real sinR = std::sin(r);
                const Real dx = c.a - bodies.localCenterOfMassX[bodyIndex];
                const Real dy = c.b - bodies.localCenterOfMassY[bodyIndex];
                bodies.comX[bodyIndex] += cosR * dx - sinR * dy;
                bodies.comY[bodyIndex] += sinR * dx + cosR * dy;
                bodies.renderOldComX[bodyIndex] = bodies.comX[bodyIndex];
                bodies.renderOldComY[bodyIndex] = bodies.comY[bodyIndex];
                bodies.localCenterOfMassX[bodyIndex] = c.a;
                bodies.localCenterOfMassY[bodyIndex] = c.b;
                for (const ObjectDenseIndex collider : bodies.colliderIndices[bodyIndex])
                    refreshComOffset(bodies, colliders, collider);
                spatialDirty = true;
                break;
            }

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
        markStaticColliderEdited(colliderIndex);
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
            refreshComOffset(bodies, colliders, colliderIndex);
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
            colliders.minExtent[colliderIndex] = circles.radius[shapeIndex];
            colliders.maxExtent[colliderIndex] = circles.radius[shapeIndex];
            spatialDirty = true;
            break;

        case T::SetBoxSize:
            if (shapeType != ColliderShapeType::Box) break;
            boxes.halfWidth[shapeIndex] = std::fmax(Real(0), c.a) * Real(0.5);
            boxes.halfHeight[shapeIndex] = std::fmax(Real(0), c.b) * Real(0.5);
            colliders.minExtent[colliderIndex] = std::fmin(boxes.halfWidth[shapeIndex], boxes.halfHeight[shapeIndex]);
            colliders.maxExtent[colliderIndex] = std::fmax(boxes.halfWidth[shapeIndex], boxes.halfHeight[shapeIndex]);
            spatialDirty = true;
            break;

        case T::SetCapsuleShape:
            if (shapeType != ColliderShapeType::Capsule) break;
            capsules.halfLength[shapeIndex] = std::fmax(Real(0), c.a) * Real(0.5);
            capsules.radius[shapeIndex] = std::fmax(Real(0), c.b);
            colliders.minExtent[colliderIndex] = capsules.radius[shapeIndex];
            colliders.maxExtent[colliderIndex] = capsules.halfLength[shapeIndex] + capsules.radius[shapeIndex];
            spatialDirty = true;
            break;

        case T::SetPolygonGeometry:
        {
            if (shapeType != ColliderShapeType::Polygon) break;
            // Recenter on the centroid (inside a convex polygon) so CCD always applies; the shift goes into the
            // collider's local offset (rotated by its current local rotation), so the visual placement is unchanged.
            auto& pending = pendingPolygons[c.index];
            const Vec2 centroid = PhysicsGeometry::calculatePolygonInertia(Real(1), pending, std::nullopt).second;
            for (Vec2& v : pending) v -= centroid;
            const Real cosL = std::cos(colliders.localRotation[colliderIndex]);
            const Real sinL = std::sin(colliders.localRotation[colliderIndex]);
            colliders.localOffsetX[colliderIndex] += cosL * centroid.x - sinL * centroid.y;
            colliders.localOffsetY[colliderIndex] += sinL * centroid.x + cosL * centroid.y;
            refreshComOffset(bodies, colliders, colliderIndex);
            polygons.geometry[shapeIndex] = std::move(pending);
            colliders.minExtent[colliderIndex] = PhysicsGeometry::calculatePolygonInnerRadius(polygons.geometry[shapeIndex]);
            colliders.maxExtent[colliderIndex] = PhysicsGeometry::calculatePolygonOuterRadius(polygons.geometry[shapeIndex]);
            spatialDirty = true;
            break;
        }

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
            moveBodyColliders(b, to);
            refreshBodyConstraintGroups(b);
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

        const Real massPerArea = mass / totalArea;

        // Sum of each collider's inertia about its own centroid plus parallel-axis shift to the body COM.
        Real inertia = Real(0);
        for (const ObjectDenseIndex ci : list)
        {
            const Real area = colliderArea(*this, ci);
            if (area <= Real(0)) continue;

            const Real m = massPerArea * area;
            const ObjectDenseIndex s = colliders.shapeIndex[ci];
            const Vec2 d(colliders.comOffsetX[ci], colliders.comOffsetY[ci]);
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
                // Polygons are centroid-centered, so the collider origin is their centroid.
                const PolygonGeometry& g = polygons.geometry[s];
                own = PhysicsGeometry::calculatePolygonInertia(m, { g.vertices(), g.size() }, std::nullopt).first;
                break;
            }
            default:
                break;
            }

            inertia += own + m * (d.x * d.x + d.y * d.y);
        }

        bodies.inertia[b] = inertia;
        bodies.invInertia[b] = inertia > Real(0) ? Real(1) / inertia : Real(0);
    }

    namespace
    {
        // Moves 'index' to group 'target' with boundary swaps. 'swapFn(a, b)' must swap two dense entries.
        // Only one element may live in Unused at a time (the one being created/destroyed).
        template<class SwapFn>
        ObjectDenseIndex moveInGroups(GroupRanges& ranges, ObjectDenseIndex index, BodyGroup target, SwapFn&& swapFn)
        {
            uint32_t* starts = ranges.starts;
            uint32_t g = static_cast<uint32_t>(ranges.groupOf(index));
            const uint32_t to = static_cast<uint32_t>(target);

            while (g != to)
            {
                if (g < to)
                {
                    const ObjectDenseIndex last = starts[g + 1] - 1;
                    swapFn(index, last);
                    index = last;
                    starts[g + 1]--;
                    g++;
                }
                else
                {
                    const ObjectDenseIndex first = starts[g];
                    swapFn(index, first);
                    index = first;
                    starts[g]++;
                    g--;
                }
            }
            return index;
        }
    }

    void ObjectManager::swapColliders(ObjectDenseIndex a, ObjectDenseIndex b)
    {
        if (a == b) return;

        colliders.swap(a, b);

        // Owner lists: collider data moved, so each owner's entry changes. Same owner: the set is unchanged.
        const ObjectDenseIndex ownerA = colliders.bodyIndex[a];
        const ObjectDenseIndex ownerB = colliders.bodyIndex[b];
        if (ownerA != ownerB)
        {
            // Collider now at a belongs to ownerA and was at b before.
            for (ObjectDenseIndex& ci : bodies.colliderIndices[ownerA]) if (ci == b) { ci = a; break; }
            for (ObjectDenseIndex& ci : bodies.colliderIndices[ownerB]) if (ci == a) { ci = b; break; }
        }

        // Shape back-references.
        const auto fixShape = [this](ObjectDenseIndex c)
            {
                const ObjectDenseIndex s = colliders.shapeIndex[c];
                switch (colliders.shapeType[c])
                {
                case ColliderShapeType::Circle:  circles.colliderIndices[s] = c; break;
                case ColliderShapeType::Box:     boxes.colliderIndices[s] = c; break;
                case ColliderShapeType::Polygon: polygons.colliderIndices[s] = c; break;
                case ColliderShapeType::Capsule: capsules.colliderIndices[s] = c; break;
                default: break;
                }
            };
        fixShape(a);
        fixShape(b);
    }

    void ObjectManager::swapShapes(ColliderShapeType type, ObjectDenseIndex a, ObjectDenseIndex b)
    {
        if (a == b) return;

        const auto swapEntries = [this, a, b](auto& soa)
            {
                soa.swap(a, b);
                colliders.shapeIndex[soa.colliderIndices[a]] = a;
                colliders.shapeIndex[soa.colliderIndices[b]] = b;
            };
        switch (type)
        {
        case ColliderShapeType::Circle:  swapEntries(circles); break;
        case ColliderShapeType::Box:     swapEntries(boxes); break;
        case ColliderShapeType::Polygon: swapEntries(polygons); break;
        case ColliderShapeType::Capsule: swapEntries(capsules); break;
        default: break;
        }
    }

    GroupRanges& ObjectManager::shapeGroups(ColliderShapeType type) noexcept
    {
        switch (type)
        {
        case ColliderShapeType::Circle:  return circles.groups;
        case ColliderShapeType::Box:     return boxes.groups;
        case ColliderShapeType::Polygon: return polygons.groups;
        default:                         return capsules.groups;
        }
    }

    size_t ObjectManager::shapeCount(ColliderShapeType type) const noexcept
    {
        switch (type)
        {
        case ColliderShapeType::Circle:  return circles.getCount();
        case ColliderShapeType::Box:     return boxes.getCount();
        case ColliderShapeType::Polygon: return polygons.getCount();
        default:                         return capsules.getCount();
        }
    }

    void ObjectManager::popBackShape(ColliderShapeType type)
    {
        switch (type)
        {
        case ColliderShapeType::Circle:  circles.popBack(); break;
        case ColliderShapeType::Box:     boxes.popBack(); break;
        case ColliderShapeType::Polygon: polygons.popBack(); break;
        case ColliderShapeType::Capsule: capsules.popBack(); break;
        default: break;
        }
    }

    ObjectDenseIndex ObjectManager::moveColliderToGroup(ObjectDenseIndex c, BodyGroup target)
    {
        c = moveInGroups(colliders.groups, c, target, [this](ObjectDenseIndex a, ObjectDenseIndex b) { swapColliders(a, b); });

        const ColliderShapeType type = colliders.shapeType[c];
        moveInGroups(shapeGroups(type), colliders.shapeIndex[c], target,
            [this, type](ObjectDenseIndex a, ObjectDenseIndex b) { swapShapes(type, a, b); });

        return c;
    }

    ObjectDenseIndex ObjectManager::placeNewCollider(ObjectDenseIndex c)
    {
        return moveColliderToGroup(c, bodies.groupOf(colliders.bodyIndex[c]));
    }

    void ObjectManager::moveBodyColliders(ObjectDenseIndex b, BodyGroup to)
    {
        spatialDirty = true;

        const std::vector<ObjectDenseIndex>& list = bodies.colliderIndices[b];
        if (list.size() == 1)
        {
            moveColliderToGroup(list[0], to);
            return;
        }

        // Moves swap colliders (possibly of this same body), so walk by handle.
        colliderMoveScratch.clear();
        for (const ObjectDenseIndex ci : list) colliderMoveScratch.push_back(colliders.handleAt(ci));
        for (const Handle h : colliderMoveScratch) moveColliderToGroup(colliders.indexOfUnchecked(h), to);
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

    void ObjectManager::swapConstraints(ConstraintType type, ObjectDenseIndex a, ObjectDenseIndex b)
    {
        if (a == b) return;

        IConstraintSystem& system = *constraintSystems[size_t(type)];
        constexpr ObjectDenseIndex NONE = Handle::NULL_DENSE_INDEX;

        ObjectDenseIndex a0, a1, b0, b1;
        system.getBodyIndices(a, a0, a1);
        system.getBodyIndices(b, b0, b1);
        system.swap(a, b);

        // Sentinel pass so constraints sharing a body are not relinked twice.
        const auto relink = [&](ObjectDenseIndex x, ObjectDenseIndex y, ObjectDenseIndex from, ObjectDenseIndex to)
            {
                bodies.replaceAttachment(x, type, from, to);
                if (y != NONE) bodies.replaceAttachment(y, type, from, to);
            };
        relink(a0, a1, a, NONE);
        relink(b0, b1, b, a);
        relink(a0, a1, NONE, b);
    }

    ObjectDenseIndex ObjectManager::moveConstraintToGroup(ConstraintType type, ObjectDenseIndex index, ConstraintGroup target)
    {
        IConstraintSystem& system = *constraintSystems[size_t(type)];
        uint32_t* starts = system.groupStarts;
        constexpr uint32_t ACTIVE = static_cast<uint32_t>(ConstraintGroup::Active);

        uint32_t g = static_cast<uint32_t>(system.groupOf(index));
        const uint32_t to = static_cast<uint32_t>(target);

        while (g != to)
        {
            const uint32_t from = g;
            if (g < to)
            {
                const ObjectDenseIndex last = system.range(static_cast<ConstraintGroup>(g)).end - 1;
                swapConstraints(type, index, last);
                index = last;
                starts[g + 1]--;
                g++;
            }
            else
            {
                const ObjectDenseIndex first = system.range(static_cast<ConstraintGroup>(g)).begin;
                swapConstraints(type, index, first);
                index = first;
                starts[g]++;
                g--;
            }
            if (from == ACTIVE || g == ACTIVE) system.wasSoAChanged = true;
        }

        return index;
    }

    ObjectDenseIndex ObjectManager::refreshConstraintGroup(ConstraintType type, ObjectDenseIndex index)
    {
        IConstraintSystem& system = *constraintSystems[size_t(type)];
        if (system.groupOf(index) == ConstraintGroup::Sleeping) return index;

        ObjectDenseIndex a, b;
        system.getBodyIndices(index, a, b);
        const bool inactive = bodies.invMass[a] == Real(0) &&
            (b == Handle::NULL_DENSE_INDEX || bodies.invMass[b] == Real(0));

        // Static set of a plan may have changed even without a move.
        if (!inactive) system.wasSoAChanged = true;
        return moveConstraintToGroup(type, index, inactive ? ConstraintGroup::Inactive : ConstraintGroup::Active);
    }

    void ObjectManager::refreshBodyConstraintGroups(ObjectDenseIndex b)
    {
        // Moves relink attachments in place, so list positions stay valid; entries are re-read each step.
        for (size_t i = 0; i < bodies.attachments[b].size(); i++)
        {
            const BodyAttachment attachment = bodies.attachments[b][i];
            refreshConstraintGroup(attachment.type, attachment.objectIndex);
        }
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
        const Real cosR = std::cos(rot);
        const Real sinR = std::sin(rot);
        const Vec2 centerOfMass = pos + Vec2(
            cosR * localCenterOfMass.x - sinR * localCenterOfMass.y,
            sinR * localCenterOfMass.x + cosR * localCenterOfMass.y
        );

        const ObjectDenseIndex index = bodies.append(
            centerOfMass, vel, rot, anglVel, mass, invMass, inertia, invInertia, localCenterOfMass
        );
        return moveBodyToGroup(index, invMass == Real(0) ? BodyGroup::Static : BodyGroup::Dynamic);
    }

    template<BodyGroup FROM, BodyGroup TO>
    ObjectDenseIndex ObjectManager::moveBodyToAdjacentGroup(ObjectDenseIndex bodyIndex)
    {
        constexpr size_t FROM_INDEX = size_t(FROM);
        constexpr size_t TO_INDEX = size_t(TO);
        static_assert(FROM_INDEX + 1 == TO_INDEX || TO_INDEX + 1 == FROM_INDEX);

        uint32_t* const starts = bodies.groupStarts;
        ObjectDenseIndex target;
        if constexpr (FROM_INDEX < TO_INDEX)
        {
            target = starts[TO_INDEX] - 1;
            starts[TO_INDEX]--;
        }
        else
        {
            target = starts[FROM_INDEX];
            starts[FROM_INDEX]++;
        }

        swapBodies(bodyIndex, target);
        bodies.groupCounts[FROM_INDEX]--;
        bodies.groupCounts[TO_INDEX]++;
        return target;
    }

    ObjectDenseIndex ObjectManager::sleepBody(ObjectDenseIndex b, uint32_t sleepIslandId)
    {
        if (!bodies.inGroup(b, BodyGroup::Dynamic)) [[unlikely]] return b;

        // Callers sleeping a whole island pass its shared id (>= 2). 0 means a standalone body: fresh id.
        if (sleepIslandId == 0) sleepIslandId = newSleepIslandId();
        bodies.sleepIslandId[b] = sleepIslandId;

        // preUpdate no longer refreshes render-old values for this body.
        bodies.renderOldComX[b] = bodies.comX[b];
        bodies.renderOldComY[b] = bodies.comY[b];
        bodies.renderOldRotation[b] = bodies.rotation[b];
        bodies.renderRotationWrapCount[b] = Real(0);
        bodies.sleepingTimer[b] = 0.0f;

        b = moveBodyToAdjacentGroup<BodyGroup::Dynamic, BodyGroup::Sleeping>(b);

        moveBodyColliders(b, BodyGroup::Sleeping);

        for (BodyAttachment& attachment : bodies.attachments[b])
        {
            moveConstraintToGroup(attachment.type, attachment.objectIndex, ConstraintGroup::Sleeping);
        }

        const Handle handle = bodies.handleAt(b);
        sleepIslands[sleepIslandId - FIRST_SLEEP_ISLAND_ID].bodies.push_back(handle);
        sleptSincePass = true;
        return b;
    }

    ObjectDenseIndex ObjectManager::wakeOne(ObjectDenseIndex b)
    {
        if (!bodies.inGroup(b, BodyGroup::Sleeping)) [[unlikely]] return b;

        bodies.sleepingTimer[b] = 0.0f;
        bodies.sleepIslandId[b] = 0;

        b = moveBodyToAdjacentGroup<BodyGroup::Sleeping, BodyGroup::Dynamic>(b);

        moveBodyColliders(b, BodyGroup::Dynamic);

        for (BodyAttachment& attachment : bodies.attachments[b])
        {
            moveConstraintToGroup(attachment.type, attachment.objectIndex, ConstraintGroup::Active);
        }

        return b;
    }

    uint32_t ObjectManager::newSleepIslandId()
    {
        uint32_t slot;
        if (!freeIslandSlots.empty())
        {
            slot = freeIslandSlots.back();
            freeIslandSlots.pop_back();
        }
        else
        {
            slot = static_cast<uint32_t>(sleepIslands.size());
            sleepIslands.emplace_back();
        }
        sleepIslands[slot].live = true;
        return slot + FIRST_SLEEP_ISLAND_ID;
    }

    void ObjectManager::wakeSleepIslands(std::span<const uint32_t> ids)
    {
        if (ids.empty() || bodies.range(BodyGroup::Sleeping).empty()) return;

        wakeScratch.clear();
        for (const uint32_t id : ids)
        {
            const size_t slot = size_t(id) - FIRST_SLEEP_ISLAND_ID; // id < FIRST wraps and fails the bound check.
            if (slot >= sleepIslands.size()) [[unlikely]] continue;

            SleepIsland& island = sleepIslands[slot];
            if (!island.live) continue; // Duplicate id, already woken.

            for (const Handle h : island.bodies)
            {
                const ObjectDenseIndex index = bodies.indexOf(h);
                if (index != Handle::NULL_DENSE_INDEX) [[likely]] wakeScratch.push_back(index);
            }
            island.bodies.clear();
            island.live = false;
            pendingFreeIslandSlots.push_back(static_cast<uint32_t>(slot));
            wokenIslandIds.push_back(id);
        }

        // Indices are resolved once and ordered, so each move swaps only with the boundary-side body.
        std::sort(wakeScratch.begin(), wakeScratch.end());
        forEachNearestBoundaryFirst<false>(wakeScratch, [this](ObjectDenseIndex i) { wakeOne(i); });
    }

    void ObjectManager::wakeCollectedIslandIds()
    {
        if (wakeIdScratch.empty()) return;
        wakeSleepIslands(wakeIdScratch);
    }

    std::span<const uint32_t> ObjectManager::takeWokenIslandIds()
    {
        wokenIslandIdsTaken.swap(wokenIslandIds);
        wokenIslandIds.clear();

        freeIslandSlots.insert(freeIslandSlots.end(), pendingFreeIslandSlots.begin(), pendingFreeIslandSlots.end());
        pendingFreeIslandSlots.clear();
        return wokenIslandIdsTaken;
    }

    ObjectDenseIndex ObjectManager::wakeBody(ObjectDenseIndex b)
    {
        if (!bodies.inGroup(b, BodyGroup::Sleeping)) [[unlikely]] return b;

        const uint32_t id = bodies.sleepIslandId[b];
        if (id == 0) [[unlikely]] return wakeOne(b);

        // Other members move too, so track this body by handle.
        const Handle handle = bodies.handleAt(b);
        wakeSleepIslands(std::span<const uint32_t>(&id, 1));
        return bodies.indexOfUnchecked(handle);
    }

    void ObjectManager::updateSleeping(
        float deltaTime, Real linearSpeedThreshold, Real angularSpeedThreshold, float sleepTimeThreshold,
        const IslandManager& islandManager, const Interactivity::BodyHolder& holder
    )
    {
        const IndexRange dynamic = bodies.range(BodyGroup::Dynamic);
        if (dynamic.empty()) return;

        const Real linearSq = linearSpeedThreshold * linearSpeedThreshold;

        float* const sleepingTimerPtr = bodies.sleepingTimer.data();
        const Real* const velXPtr = bodies.velocityX.data();
        const Real* const velYPtr = bodies.velocityY.data();
        const Real* const angVelPtr = bodies.angularVelocity.data();
        const ObjectDenseIndex* const islandIdPtr = bodies.islandId.data();

        // Bodies woken after the islands were built (commands) have NO_ISLAND: they map to a trailing slot that stays 0.
        const uint32_t islandCount = islandManager.getIslandCount();
        islandSleepFlags.assign(size_t(islandCount) + 1, 1u);
        uint32_t* const flags = islandSleepFlags.data();
        flags[islandCount] = 0;

        // Timers, and AND-reduce readiness per island.
        for (ObjectDenseIndex i = dynamic.begin; i < dynamic.end; i++)
        {
            const Real linearExcess = velXPtr[i] * velXPtr[i] + velYPtr[i] * velYPtr[i] - linearSq;
            const Real angularExcess = std::fabs(angVelPtr[i]) - angularSpeedThreshold;
            const bool quiet = std::fmax(linearExcess, angularExcess) <= Real(0);

            const float timer = (sleepingTimerPtr[i] + deltaTime) * float(quiet);
            sleepingTimerPtr[i] = timer;

            flags[std::min<uint32_t>(islandIdPtr[i], islandCount)] &= uint32_t(quiet) & uint32_t(timer >= sleepTimeThreshold);
        }

        for (const Interactivity::BodyHolder::HeldBody& held : holder.heldBodies)
        {
            const ObjectDenseIndex index = bodies.indexOfUnchecked(held.body);
            if (index - dynamic.begin < dynamic.size()) flags[std::min<uint32_t>(islandIdPtr[index], islandCount)] = 0;
        }

        sleepScratch.clear();
        for (ObjectDenseIndex i = dynamic.begin; i < dynamic.end; i++)
        {
            uint32_t& flag = flags[std::min<uint32_t>(islandIdPtr[i], islandCount)];
            if (flag == 0) continue;
            if (flag == 1) flag = newSleepIslandId();
            sleepScratch.push_back(i);
        }

        forEachNearestBoundaryFirst<true>(sleepScratch, [&](ObjectDenseIndex index)
            {
                sleepBody(index, flags[islandIdPtr[index]]);
            });
    }

    size_t ObjectManager::getSleepMemoryUsage() const noexcept
    {
        size_t total =
            getVectorMemoryUsage(wakeScratch) +
            getVectorMemoryUsage(wakeIdScratch) +
            getVectorMemoryUsage(sleepScratch) +
            getVectorMemoryUsage(islandSleepFlags) +
            getVectorMemoryUsage(wokenIslandIds) +
            getVectorMemoryUsage(wokenIslandIdsTaken) +
            getVectorMemoryUsage(freeIslandSlots) +
            getVectorMemoryUsage(pendingFreeIslandSlots) +
            getVectorMemoryUsage(sleepIslands);
        for (const SleepIsland& island : sleepIslands) total += getVectorMemoryUsage(island.bodies);
        return total;
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