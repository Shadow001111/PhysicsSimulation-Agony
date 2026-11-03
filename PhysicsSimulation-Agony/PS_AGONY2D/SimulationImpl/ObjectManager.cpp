#include "ObjectManager.h"
#include "PhysicsGeometry.h"
#include "PolygonBuild.h"

#include "PS_AGONY2D/Limits.h"
#include "PS_AGONY2D/Constants.h"

#include <cmath>
#include <iostream>

namespace PS_AGONY2D
{
    ObjectManager::ObjectManager(const std::vector<Material>& materials) :
        materials(materials)
    {
    }

    std::optional<ObjectDenseIndex> ObjectManager::createBody(const BodyCreateParams& params)
    {
        const ObjectDenseIndex newBodyIndex = static_cast<ObjectDenseIndex>(bodies.getCount());

        const Vec2 centerOfMass = params.centerOfMass.value_or(Vec2(0));

        const Real mass = std::fmax(Real(0), params.mass);
        const Real inertia = std::fmax(Real(0), params.inertia);

        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        // No shape attached yet, so there's nothing to derive inertia from: it starts
        // at zero (infinite resistance to rotation change is NOT implied - it just
        // means no angular response until a collider/inertia is set some other way).
        bodies.append(
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

        const Real radius = std::fmax(Real(0), params.radius);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(circles.getCount());

        const ObjectDenseIndex newCollider = createColliderInternal(
            params.bodyIndex, params.localOffset, params.localRotation, materialIndex, BodyType::Circle, newShapeIndex
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

        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectDenseIndex newShapeIndex = static_cast<ObjectDenseIndex>(boxes.getCount());

        const ObjectDenseIndex newCollider = createColliderInternal(
            params.bodyIndex, params.localOffset, params.localRotation, materialIndex, BodyType::Box, newShapeIndex
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

        const ObjectDenseIndex firstCollider = createConvexPolygonColliderInternal(
            params.bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[0])
        );
        for (size_t i = 1; i < convexPieces.size(); ++i)
        {
            createConvexPolygonColliderInternal(
                params.bodyIndex, params.localOffset, params.localRotation, materialIndex, std::move(convexPieces[i])
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
            params.bodyIndex, localOffset, localRotation, materialIndex, BodyType::Capsule, newShapeIndex
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

        const ObjectDenseIndex newBodyIndex = static_cast<ObjectDenseIndex>(bodies.getCount());
        bodies.append(
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

        const ObjectDenseIndex newBodyIndex = static_cast<ObjectDenseIndex>(bodies.getCount());
        bodies.append(
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

        const ObjectDenseIndex newBodyIndex = static_cast<ObjectDenseIndex>(bodies.getCount());
        bodies.append(
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

        const ObjectDenseIndex newBodyIndex = static_cast<ObjectDenseIndex>(bodies.getCount());
        bodies.append(
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

    void ObjectManager::destroyBody(ObjectDenseIndex bodyIndex, ConstraintSystemArray& constraintSystems)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyIndex >= bodyCount) return;

        // Cascade-delete constraints (springs, unchanged).
        while (!bodies.attachments[bodyIndex].empty())
        {
            const BodyAttachment attachment = bodies.attachments[bodyIndex].back();
            constraintSystems[static_cast<size_t>(attachment.type)]->removeConstraint(attachment.objectIndex, bodies);
        }

        // Cascade-delete every collider owned by this body.
        while (!bodies.colliderIndices[bodyIndex].empty())
        {
            const ObjectDenseIndex colliderIdx = bodies.colliderIndices[bodyIndex].back();
            destroyColliderInternal(bodyIndex, colliderIdx);
        }

        // --- Body removal (unchanged below this point, minus bodyType/shapeIndex shape-swap block) ---
        if (bodyIndex != bodyCount - 1)
        {
            const ObjectDenseIndex oldBackIndex = static_cast<ObjectDenseIndex>(bodyCount - 1);

            bodies.swapWithBack(bodyIndex);

            // Fix up colliders owned by the body that got swapped into bodyIndex.
            for (ObjectDenseIndex ci : bodies.colliderIndices[bodyIndex])
                colliders.bodyIndex[ci] = bodyIndex;

            for (const BodyAttachment& attachment : bodies.attachments[bodyIndex])
                constraintSystems[static_cast<size_t>(attachment.type)]->remapBodyIndex(attachment.objectIndex, oldBackIndex, bodyIndex);

            deletedBodies.emplace_back(bodyIndex, static_cast<ObjectDenseIndex>(bodyCount - 1));
        }
        else
        {
            deletedBodies.emplace_back(bodyIndex, bodyIndex);
        }

        bodies.popBack();
    }

    void ObjectManager::destroyCollider(ObjectDenseIndex colliderIndex, ConstraintSystemArray& constraintSystems)
    {
        if (colliderIndex >= colliders.getCount()) return;

        const ObjectDenseIndex bodyIndex = colliders.bodyIndex[colliderIndex];
        destroyColliderInternal(bodyIndex, colliderIndex);
    }

    ObjectDenseIndex ObjectManager::createColliderInternal(ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation, MaterialIndex materialIndex, BodyType shapeType, ObjectDenseIndex shapeIndex)
    {
        const ObjectDenseIndex newCollider = colliders.append(
            bodyIndex, localOffset, localRotation, materialIndex, shapeType, shapeIndex
        );
        bodies.addCollider(bodyIndex, newCollider);
        spatialDirty = true;

        std::vector<ObjectDenseIndex>& list = bodies.isStatic[bodyIndex] ? staticColliderIDs : dynamicColliderIDs;
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
            bodyIndex, localOffset, localRotation, materialIndex, BodyType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(geometry));

        return newCollider;
    }

    void ObjectManager::destroyColliderInternal(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIdx)
    {
        spatialDirty = true;

        const BodyType shapeType = colliders.shapeType[colliderIdx];
        const ObjectDenseIndex shapeIdx = colliders.shapeIndex[colliderIdx];

        // Remove from the underlying shape SoA (swap-remove), fixing up the collider
        // that now occupies shapeIdx (if any) to point at its new slot.
        if (shapeType == BodyType::Circle)
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
        else if (shapeType == BodyType::Box)
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
        else if (shapeType == BodyType::Polygon)
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
        else if (shapeType == BodyType::Capsule)
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
            std::vector<ObjectDenseIndex>& list = bodies.isStatic[bodyIndex] ? staticColliderIDs : dynamicColliderIDs;
            const uint32_t slot = colliderSlot[colliderIdx];
            const ObjectDenseIndex lastID = list.back();
            list[slot] = lastID;
            colliderSlot[lastID] = slot;
            list.pop_back();
        }

        // Remove from ColliderSoA itself.
        bodies.removeCollider(bodyIndex, colliderIdx);
        const size_t oldBackCollider = colliders.swapRemove(colliderIdx);
        if (oldBackCollider != colliderIdx)
        {
            const ObjectDenseIndex swappedOwner = colliders.bodyIndex[colliderIdx];
            bodies.removeCollider(swappedOwner, static_cast<ObjectDenseIndex>(oldBackCollider));
            bodies.addCollider(swappedOwner, colliderIdx);

            // Moved collider: oldBackCollider -> colliderIdx in its ID list.
            std::vector<ObjectDenseIndex>& movedList = bodies.isStatic[swappedOwner] ? staticColliderIDs : dynamicColliderIDs;
            const uint32_t movedSlot = colliderSlot[oldBackCollider];
            movedList[movedSlot] = static_cast<ObjectDenseIndex>(colliderIdx);
            colliderSlot[colliderIdx] = movedSlot;

            // Fix the shape SoA's back-reference: it still points at the collider's old index.
            const BodyType movedShapeType = colliders.shapeType[colliderIdx];
            const ObjectDenseIndex movedShapeIdx = colliders.shapeIndex[colliderIdx];
            if (movedShapeType == BodyType::Circle)
                circles.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == BodyType::Box)
                boxes.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == BodyType::Polygon)
                polygons.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == BodyType::Capsule)
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
            case BodyType::Circle:
                return PhysicsGeometry::calculateCircleArea(om.circles.radius[s]);
            case BodyType::Box:
                return PhysicsGeometry::calculateBoxArea(om.boxes.halfWidth[s] * Real(2), om.boxes.halfHeight[s] * Real(2));
            case BodyType::Capsule:
                return PhysicsGeometry::calculateCapsuleArea(om.capsules.halfLength[s] * Real(2), om.capsules.radius[s]);
            case BodyType::Polygon:
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

        if (c.type < T::SetColliderLocalOffset)
        {
            const uint32_t resolved = bodies.indexOf(c.handle);
            if (resolved == Handle::kNull) return;
            const ObjectDenseIndex b = static_cast<ObjectDenseIndex>(resolved);

            switch (c.type)
            {
            case T::SetBodyPosition:
                bodies.offsetX[b] = c.a;
                bodies.offsetY[b] = c.b;
                bodies.renderOldOffsetX[b] = c.a; // Teleport: no render interpolation.
                bodies.renderOldOffsetY[b] = c.b;
                spatialDirty = true;
                break;

            case T::SetBodyRotation:
            {
                const Real r = wrapAngle(c.a);
                bodies.rotation[b] = r;
                bodies.rotationCos[b] = std::cos(r);
                bodies.rotationSin[b] = std::sin(r);
                bodies.renderOldRotation[b] = r;
                bodies.renderRotationWrapCount[b] = Real(0);
                spatialDirty = true;
                break;
            }

            case T::SetBodyVelocity:
                bodies.velocityX[b] = c.a;
                bodies.velocityY[b] = c.b;
                break;

            case T::SetBodyAngularVelocity:
                bodies.angularVelocity[b] = c.a;
                break;

            case T::SetBodyMass:
                applyBodyMass(b, c.a);
                break;

            case T::SetBodyInertia:
            {
                if (bodies.isStatic[b]) break;
                const Real inertia = std::fmax(Real(0), c.a);
                bodies.inertia[b] = inertia;
                bodies.invInertia[b] = inertia > Real(0) ? Real(1) / inertia : Real(0);
                break;
            }

            case T::SetBodyLocalCenterOfMass:
                bodies.localCenterOfMassX[b] = c.a;
                bodies.localCenterOfMassY[b] = c.b;
                spatialDirty = true; // World center depends on it.
                break;

            case T::RecomputeBodyInertia:
                recomputeBodyInertia(b);
                break;

            case T::SetBodyMassFromDensity:
                applyBodyMassFromDensity(b, c.a);
                break;

            default:
                break;
            }
            return;
        }

        const uint32_t resolved = colliders.indexOf(c.handle);
        if (resolved == Handle::kNull) return;
        const ObjectDenseIndex ci = static_cast<ObjectDenseIndex>(resolved);
        const ObjectDenseIndex s = colliders.shapeIndex[ci];
        const BodyType shapeType = colliders.shapeType[ci];

        switch (c.type)
        {
        case T::SetColliderLocalOffset:
            colliders.localOffsetX[ci] = c.a;
            colliders.localOffsetY[ci] = c.b;
            spatialDirty = true;
            break;

        case T::SetColliderLocalRotation:
            colliders.localRotation[ci] = wrapAngle(c.a);
            spatialDirty = true;
            break;

        case T::SetColliderMaterial:
            colliders.materialIndex[ci] = c.index < materials.size() ? static_cast<MaterialIndex>(c.index) : MaterialIndex(0);
            break;

        case T::SetCircleRadius:
            if (shapeType != BodyType::Circle) break;
            circles.radius[s] = std::fmax(Real(0), c.a);
            spatialDirty = true;
            break;

        case T::SetBoxSize:
            if (shapeType != BodyType::Box) break;
            boxes.halfWidth[s] = std::fmax(Real(0), c.a) * Real(0.5);
            boxes.halfHeight[s] = std::fmax(Real(0), c.b) * Real(0.5);
            spatialDirty = true;
            break;

        case T::SetCapsuleShape:
            if (shapeType != BodyType::Capsule) break;
            capsules.halfLength[s] = std::fmax(Real(0), c.a) * Real(0.5);
            capsules.radius[s] = std::fmax(Real(0), c.b);
            spatialDirty = true;
            break;

        case T::SetPolygonGeometry:
            if (shapeType != BodyType::Polygon) break;
            polygons.geometry[s] = std::move(pendingPolygons[c.index]);
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
        const bool wasStatic = bodies.isStatic[b] != 0;

        if (newMass > Real(0))
        {
            Real inertia = bodies.inertia[b];
            if (oldMass > Real(0)) inertia *= newMass / oldMass;

            bodies.mass[b] = newMass;
            bodies.invMass[b] = Real(1) / newMass;
            bodies.inertia[b] = inertia;
            bodies.invInertia[b] = inertia > Real(0) ? Real(1) / inertia : Real(0);
            bodies.isStatic[b] = 0;
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
            bodies.isStatic[b] = 1;
        }

        const bool isStaticNow = bodies.isStatic[b] != 0;
        if (wasStatic != isStaticNow) moveBodyColliders(b, isStaticNow);
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
            case BodyType::Circle:
                own = PhysicsGeometry::calculateCircleInertia(m, circles.radius[s], Vec2(0));
                break;
            case BodyType::Box:
                own = PhysicsGeometry::calculateBoxInertia(m, boxes.halfWidth[s] * Real(2), boxes.halfHeight[s] * Real(2), Vec2(0));
                break;
            case BodyType::Capsule:
                own = PhysicsGeometry::calculateCapsuleInertia(m, capsules.halfLength[s] * Real(2), capsules.radius[s], Vec2(0));
                break;
            case BodyType::Polygon:
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

    void ObjectManager::moveBodyColliders(ObjectDenseIndex b, bool toStatic)
    {
        std::vector<ObjectDenseIndex>& from = toStatic ? dynamicColliderIDs : staticColliderIDs;
        std::vector<ObjectDenseIndex>& to = toStatic ? staticColliderIDs : dynamicColliderIDs;

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
        case BodyType::Circle:
            d.radius = circles.radius[s];
            break;
        case BodyType::Box:
            d.halfExtents = Vec2(boxes.halfWidth[s], boxes.halfHeight[s]);
            break;
        case BodyType::Capsule:
            d.halfLength = capsules.halfLength[s];
            d.radius = capsules.radius[s];
            break;
        case BodyType::Polygon:
            d.polygon = &polygons.geometry[s];
            break;
        default:
            break;
        }
        return d;
    }
}