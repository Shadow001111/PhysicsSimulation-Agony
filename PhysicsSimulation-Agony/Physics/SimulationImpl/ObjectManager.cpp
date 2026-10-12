#include "ObjectManager.h"
#include "PhysicsGeometry.h"
#include "PolygonBuild.h"
#include <cmath>

#include <iostream>

namespace PS_AGONY
{
    ObjectManager::ObjectManager(const std::vector<Material>& materials) :
        materials(materials)
    {
    }

    std::optional<ObjectIndex> ObjectManager::createBody(const BodyCreateParams& params)
    {
        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());

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

    std::optional<ColliderIndex> ObjectManager::createCircleCollider(const CircleColliderCreateParams& params)
    {
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createCircleCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        const Real radius = std::fmax(Real(0), params.radius);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectIndex newShapeIndex = static_cast<ObjectIndex>(circles.getCount());

        const ColliderIndex newCollider = createColliderInternal(
            params.bodyIndex, params.localOffset, params.localRotation, materialIndex, BodyType::Circle, newShapeIndex
        );
        circles.append(newCollider, radius);

        return newCollider;
    }

    std::optional<ColliderIndex> ObjectManager::createBoxCollider(const BoxColliderCreateParams& params)
    {
        if (params.bodyIndex >= bodies.getCount())
        {
            std::cerr << "[AGONY][Simulation::createBoxCollider]: Failed: bodyIndex is invalid.\n";
            return std::nullopt;
        }

        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectIndex newShapeIndex = static_cast<ObjectIndex>(boxes.getCount());

        const ColliderIndex newCollider = createColliderInternal(
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

        const ColliderIndex firstCollider = createConvexPolygonColliderInternal(
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

    std::optional<ObjectIndex> ObjectManager::createCircle(const CircleCreateParams& params)
    {
        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real radius = std::fmax(Real(0), params.radius);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));

        const Real inertia = PhysicsGeometry::calculateCircleInertia(mass, radius, centerOfMass);
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());
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

    std::optional<ObjectIndex> ObjectManager::createBox(const BoxCreateParams& params)
    {
        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));

        const Real inertia = PhysicsGeometry::calculateBoxInertia(mass, width, height, centerOfMass);
        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());
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

    std::optional<ObjectIndex> ObjectManager::createPolygon(const PolygonCreateParams& params)
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

        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());
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

    void ObjectManager::destroyBody(ObjectIndex bodyIndex, ConstraintSystemArray& constraintSystems)
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
            const ColliderIndex colliderIdx = bodies.colliderIndices[bodyIndex].back();
            destroyColliderInternal(bodyIndex, colliderIdx);
        }

        // --- Body removal (unchanged below this point, minus bodyType/shapeIndex shape-swap block) ---
        if (bodyIndex != bodyCount - 1)
        {
            const ObjectIndex oldBackIndex = static_cast<ObjectIndex>(bodyCount - 1);

            bodies.swapWithBack(bodyIndex);

            // Fix up colliders owned by the body that got swapped into bodyIndex.
            for (ColliderIndex ci : bodies.colliderIndices[bodyIndex])
                colliders.bodyIndex[ci] = bodyIndex;

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

    void ObjectManager::destroyCollider(ColliderIndex colliderIndex, ConstraintSystemArray& constraintSystems)
    {
        if (colliderIndex >= colliders.getCount()) return;

        const ObjectIndex bodyIndex = colliders.bodyIndex[colliderIndex];
        destroyColliderInternal(bodyIndex, colliderIndex);
    }

    ColliderIndex ObjectManager::createColliderInternal(ObjectIndex bodyIndex, Vec2 localOffset, Real localRotation, MaterialIndex materialIndex, BodyType shapeType, ObjectIndex shapeIndex)
    {
        const ColliderIndex newCollider = colliders.append(
            bodyIndex, localOffset, localRotation, materialIndex, shapeType, shapeIndex
        );
        bodies.addCollider(bodyIndex, newCollider);

        std::vector<ObjectIndex>& list = bodies.isStatic[bodyIndex] ? staticColliderIDs : dynamicColliderIDs;
        colliderSlot.resize(colliders.getCount());
        colliderSlot[newCollider] = static_cast<uint32_t>(list.size());
        list.push_back(static_cast<ObjectIndex>(newCollider));

        return newCollider;
    }

    ColliderIndex ObjectManager::createConvexPolygonColliderInternal(
        ObjectIndex bodyIndex, Vec2 localOffset, Real localRotation,
        MaterialIndex materialIndex, std::vector<Vec2> convexVertices
    )
    {
        const ObjectIndex newShapeIndex = static_cast<ObjectIndex>(polygons.getCount());

        VerticesContainer vertices{ convexVertices.data(), convexVertices.size() };

        const ColliderIndex newCollider = createColliderInternal(
            bodyIndex, localOffset, localRotation, materialIndex, BodyType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(vertices));

        return newCollider;
    }

    void ObjectManager::destroyColliderInternal(ObjectIndex bodyIndex, ColliderIndex colliderIdx)
    {
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

        // Remove from the persistent ID list (swap-remove, O(1)).
        {
            std::vector<ObjectIndex>& list = bodies.isStatic[bodyIndex] ? staticColliderIDs : dynamicColliderIDs;
            const uint32_t slot = colliderSlot[colliderIdx];
            const ObjectIndex lastID = list.back();
            list[slot] = lastID;
            colliderSlot[lastID] = slot;
            list.pop_back();
        }

        // Remove from ColliderSoA itself.
        bodies.removeCollider(bodyIndex, colliderIdx);
        const size_t oldBackCollider = colliders.swapRemove(colliderIdx);
        if (oldBackCollider != colliderIdx)
        {
            const ObjectIndex swappedOwner = colliders.bodyIndex[colliderIdx];
            bodies.removeCollider(swappedOwner, static_cast<ColliderIndex>(oldBackCollider));
            bodies.addCollider(swappedOwner, colliderIdx);

            // Moved collider: oldBackCollider -> colliderIdx in its ID list.
            std::vector<ObjectIndex>& movedList = bodies.isStatic[swappedOwner] ? staticColliderIDs : dynamicColliderIDs;
            const uint32_t movedSlot = colliderSlot[oldBackCollider];
            movedList[movedSlot] = static_cast<ObjectIndex>(colliderIdx);
            colliderSlot[colliderIdx] = movedSlot;

            // Fix the shape SoA's back-reference: it still points at the collider's old index.
            const BodyType movedShapeType = colliders.shapeType[colliderIdx];
            const ObjectIndex movedShapeIdx = colliders.shapeIndex[colliderIdx];
            if (movedShapeType == BodyType::Circle)
                circles.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == BodyType::Box)
                boxes.colliderIndices[movedShapeIdx] = colliderIdx;
            else if (movedShapeType == BodyType::Polygon)
                polygons.colliderIndices[movedShapeIdx] = colliderIdx;

            deletedColliders.emplace_back(
                static_cast<ObjectIndex>(colliderIdx), static_cast<ObjectIndex>(oldBackCollider)
            );
        }
        else
        {
            deletedColliders.emplace_back(static_cast<ObjectIndex>(colliderIdx), static_cast<ObjectIndex>(colliderIdx));
        }

        colliderSlot.pop_back();
    }
}