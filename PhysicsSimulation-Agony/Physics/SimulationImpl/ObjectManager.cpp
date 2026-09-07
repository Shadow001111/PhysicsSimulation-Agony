#include "ObjectManager.h"
#include "PhysicsGeometry.h"

#include <iostream>

namespace PS_AGONY
{
    ObjectManager::ObjectManager(const std::vector<Material>& materials) :
        materials(materials)
    {}

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

    std::optional<ColliderIndex> ObjectManager::createPolygonCollider(const PolygonColliderCreateParams& params)
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

        std::vector<Vec2> uniqueVertices = PhysicsGeometry::filterDuplicateVertices(params.localVertices, params.verticesCount);
        if (uniqueVertices.size() < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygonCollider]: Failed: unique vertices count is less than three.\n";
            return std::nullopt;
        }

        const MaterialIndex materialIndex = params.materialIndex < materials.size() ? params.materialIndex : 0;
        const ObjectIndex newShapeIndex = static_cast<ObjectIndex>(polygons.getCount());

        VerticesContainer vertices{ uniqueVertices.data(), uniqueVertices.size() };

        const ColliderIndex newCollider = createColliderInternal(
            params.bodyIndex, params.localOffset, params.localRotation, materialIndex, BodyType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(vertices));

        return newCollider;
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

        std::vector<Vec2> uniqueVertices = PhysicsGeometry::filterDuplicateVertices(params.localVertices, params.verticesCount);
        if (uniqueVertices.size() < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Unique vertices count is less than three.\n";
            return std::nullopt;
        }

        const Real mass = std::fmax(Real(0), params.base.mass);

        VerticesContainer vertices{ uniqueVertices.data(), uniqueVertices.size() };

        auto iCOM = PhysicsGeometry::calculatePolygonInertia(mass, vertices, std::nullopt);
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

        const Real invMass = mass == Real(0) ? Real(0) : Real(1) / mass;
        const Real invInertia = inertia == Real(0) ? Real(0) : Real(1) / inertia;

        const ObjectIndex newBodyIndex = static_cast<ObjectIndex>(bodies.getCount());
        bodies.append(
            params.base.position + trueCenterOfMass,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass, invMass,
            inertia, invInertia,
            neededCenterOfMass
        );

        // Vertices were already de-duplicated/re-centered above, so route the already-
        // constructed VerticesContainer straight to the shape SoA below instead of
        // going through createPolygonCollider() (which would redo dedup on raw pointers
        // and know nothing about the COM shift already baked into these vertices).
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;
        const ObjectIndex newShapeIndex = static_cast<ObjectIndex>(polygons.getCount());

        const ColliderIndex newCollider = createColliderInternal(
            newBodyIndex, Vec2(0), Real(0), materialIndex, BodyType::Polygon, newShapeIndex
        );
        polygons.append(newCollider, std::move(vertices));

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

        // Remove from ColliderSoA itself.
        bodies.removeCollider(bodyIndex, colliderIdx);
        const size_t oldBackCollider = colliders.swapRemove(colliderIdx);
        if (oldBackCollider != colliderIdx)
        {
            const ObjectIndex swappedOwner = colliders.bodyIndex[colliderIdx];
            bodies.removeCollider(swappedOwner, static_cast<ColliderIndex>(oldBackCollider));
            bodies.addCollider(swappedOwner, colliderIdx);

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
    }
}