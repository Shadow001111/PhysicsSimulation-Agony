#pragma once
#include "PS_AGONY2D/ConstraintSystem.h"

#include "PS_AGONY2D/Types/Types.h"

#include "PS_AGONY2D/SimulationImpl/ObjectManager.h"

#include <optional>
#include <span>

namespace PS_AGONY2D
{
	// Non-owning facade over ObjectManager.
	class ObjectAPI
	{
	private:
		ObjectManager& objectManager;
		ConstraintSystemArray& constraintSystems;
	public:
		ObjectAPI(ObjectManager& objectManager, ConstraintSystemArray& constraintSystems) noexcept
			: objectManager(objectManager), constraintSystems(constraintSystems) {}

		ObjectAPI(const ObjectAPI&) = delete;
		ObjectAPI& operator=(const ObjectAPI&) = delete;

		// Creation (dense indices).
		std::optional<ObjectDenseIndex> createBody(const BodyCreateParams& params) { return objectManager.createBody(params); }
		std::optional<ObjectDenseIndex> createCircleCollider(const CircleColliderCreateParams& params) { return objectManager.createCircleCollider(params); }
		std::optional<ObjectDenseIndex> createBoxCollider(const BoxColliderCreateParams& params) { return objectManager.createBoxCollider(params); }
		std::optional<PolygonColliderCreateResult> createPolygonCollider(const PolygonColliderCreateParams& params) { return objectManager.createPolygonCollider(params); }
		std::optional<ObjectDenseIndex> createCapsuleCollider(const CapsuleColliderCreateParams& params) { return objectManager.createCapsuleCollider(params); }

		std::optional<ObjectDenseIndex> createCircle(const CircleCreateParams& params) { return objectManager.createCircle(params); }
		std::optional<ObjectDenseIndex> createBox(const BoxCreateParams& params) { return objectManager.createBox(params); }
		std::optional<ObjectDenseIndex> createPolygon(const PolygonCreateParams& params) { return objectManager.createPolygon(params); }
		std::optional<ObjectDenseIndex> createCapsule(const CapsuleCreateParams& params) { return objectManager.createCapsule(params); }

		// Persistent handles. Valid across swap-and-pop; stale after destroy (generation bump).
		// Dense indices are only stable until the next create/destroy; handles survive it.
		Handle getBodyHandle(ObjectDenseIndex bodyIndex) const noexcept
		{
			return bodyIndex < objectManager.bodies.getCount() ? objectManager.bodies.handleAt(bodyIndex) : Handle{};
		}
		Handle getColliderHandle(ObjectDenseIndex colliderIndex) const noexcept
		{
			return colliderIndex < objectManager.colliders.getCount() ? objectManager.colliders.handleAt(colliderIndex) : Handle{};
		}

		bool isBodyValid(Handle h) const noexcept { return objectManager.bodies.isValid(h); }
		bool isColliderValid(Handle h) const noexcept { return objectManager.colliders.isValid(h); }

		// Handle::kNull if stale.
		ObjectDenseIndex getBodyIndex(Handle h) const noexcept { return objectManager.bodies.indexOf(h); }
		ObjectDenseIndex getColliderIndex(Handle h) const noexcept { return objectManager.colliders.indexOf(h); }

		// Hot path: no validation. Handle must be alive.
		ObjectDenseIndex getBodyIndexUnchecked(Handle h) const noexcept { return objectManager.bodies.indexOfUnchecked(h); }
		ObjectDenseIndex getColliderIndexUnchecked(Handle h) const noexcept { return objectManager.colliders.indexOfUnchecked(h); }

		// Handle-returning creators.
		std::optional<Handle> createBodyH(const BodyCreateParams& params) { return toBodyHandle(objectManager.createBody(params)); }
		std::optional<Handle> createCircleH(const CircleCreateParams& params) { return toBodyHandle(objectManager.createCircle(params)); }
		std::optional<Handle> createBoxH(const BoxCreateParams& params) { return toBodyHandle(objectManager.createBox(params)); }
		std::optional<Handle> createPolygonH(const PolygonCreateParams& params) { return toBodyHandle(objectManager.createPolygon(params)); }
		std::optional<Handle> createCapsuleH(const CapsuleCreateParams& params) { return toBodyHandle(objectManager.createCapsule(params)); }

		// Collider creators taking a body handle (params.bodyIndex is ignored). nullopt if the handle is stale.
		std::optional<Handle> createCircleColliderH(Handle body, CircleColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createCircleCollider(params));
		}
		std::optional<Handle> createBoxColliderH(Handle body, BoxColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createBoxCollider(params));
		}
		std::optional<Handle> createCapsuleColliderH(Handle body, CapsuleColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return toColliderHandle(objectManager.createCapsuleCollider(params));
		}
		// Pieces are contiguous in dense order: use getColliderHandle(first + i) right after the call.
		std::optional<PolygonColliderCreateResult> createPolygonColliderH(Handle body, PolygonColliderCreateParams params)
		{
			const uint32_t b = getBodyIndex(body);
			if (b == Handle::kNull) return std::nullopt;
			params.bodyIndex = static_cast<ObjectDenseIndex>(b);
			return objectManager.createPolygonCollider(params);
		}

		// Stale handle: no-op.
		void destroyBody(Handle h)
		{
			const ObjectDenseIndex i = getBodyIndex(h);
			if (i != Handle::kNull) destroyBody(i);
		}
		void destroyCollider(Handle h)
		{
			const uint32_t i = getColliderIndex(h);
			if (i != Handle::kNull) destroyCollider(static_cast<ObjectDenseIndex>(i));
		}

		void destroyBody(ObjectDenseIndex bodyIndex) { objectManager.destroyBody(bodyIndex, constraintSystems); }
		void destroyCollider(ObjectDenseIndex colliderIndex) { objectManager.destroyCollider(colliderIndex, constraintSystems); }

		// Deferred modifications. Queued and applied at the start of Simulation::update() and after every physics step,
		// so they are safe to call from collision callbacks. Stale handles are dropped. Semantics: see ObjectManager.h.
		void setBodyPosition(Handle body, Vec2 position) { objectManager.queueBodyPosition(body, position); }
		void setBodyRotation(Handle body, Real rotation) { objectManager.queueBodyRotation(body, rotation); }
		void setBodyVelocity(Handle body, Vec2 velocity) { objectManager.queueBodyVelocity(body, velocity); }
		void setBodyAngularVelocity(Handle body, Real angularVelocity) { objectManager.queueBodyAngularVelocity(body, angularVelocity); }
		void setBodyMass(Handle body, Real mass) { objectManager.queueBodyMass(body, mass); }
		void setBodyInertia(Handle body, Real inertia) { objectManager.queueBodyInertia(body, inertia); }
		void setBodyLocalCenterOfMass(Handle body, Vec2 localCenterOfMass) { objectManager.queueBodyLocalCenterOfMass(body, localCenterOfMass); }
		void recomputeBodyInertia(Handle body) { objectManager.queueRecomputeBodyInertia(body); }
		void setBodyMassFromDensity(Handle body, Real density) { objectManager.queueBodyMassFromDensity(body, density); }

		void setColliderLocalOffset(Handle collider, Vec2 localOffset) { objectManager.queueColliderLocalOffset(collider, localOffset); }
		void setColliderLocalRotation(Handle collider, Real localRotation) { objectManager.queueColliderLocalRotation(collider, localRotation); }
		void setColliderMaterial(Handle collider, MaterialIndex materialIndex) { objectManager.queueColliderMaterial(collider, materialIndex); }
		void setCircleRadius(Handle collider, Real radius) { objectManager.queueCircleRadius(collider, radius); }
		void setBoxSize(Handle collider, Vec2 size) { objectManager.queueBoxSize(collider, size); }
		void setCapsuleShape(Handle collider, Real length, Real radius) { objectManager.queueCapsuleShape(collider, length, radius); }
		// False (nothing queued) if the polygon is invalid or not convex.
		bool setPolygonShape(Handle collider, std::span<const Vec2> localVertices) { return objectManager.queueColliderPolygon(collider, localVertices); }

		// Viewers.
		BodySoAViewer getBodies() const noexcept { return BodySoAViewer(objectManager.bodies); }
		ColliderSoAViewer getColliders() const noexcept { return ColliderSoAViewer(objectManager.colliders); }
		CircleSoAViewer getCircles() const noexcept { return CircleSoAViewer(objectManager.circles); }
		BoxSoAViewer getBoxes() const noexcept { return BoxSoAViewer(objectManager.boxes); }
		PolygonSoAViewer getPolygons() const noexcept { return PolygonSoAViewer(objectManager.polygons); }
		CapsuleSoAViewer getCapsules() const noexcept { return CapsuleSoAViewer(objectManager.capsules); }

		std::optional<BodyData> getBodyData(ObjectDenseIndex index) const noexcept
		{
			return objectManager.bodies.getBodyData(index);
		}

		// Instant reads of current stored state (not the queue); nullopt if the handle is stale.
		std::optional<BodyData> getBodyDataH(Handle body) const noexcept
		{
			const uint32_t i = objectManager.bodies.indexOf(body);
			return i == Handle::kNull ? std::nullopt : objectManager.bodies.getBodyData(static_cast<ObjectDenseIndex>(i));
		}
		std::optional<ColliderData> getColliderDataH(Handle collider) const noexcept
		{
			const uint32_t i = objectManager.colliders.indexOf(collider);
			return i == Handle::kNull ? std::nullopt : objectManager.getColliderData(static_cast<ObjectDenseIndex>(i));
		}

		//
		std::span<const ObjectDenseIndex> getBodyColliders(ObjectDenseIndex bodyIndex)
		{
			if (bodyIndex >= objectManager.bodies.getCount()) { return {}; }
			std::span<const ObjectDenseIndex> indicesSpan = objectManager.bodies.colliderIndices[bodyIndex];
			return indicesSpan;
		}
	private:
		std::optional<Handle> toBodyHandle(std::optional<ObjectDenseIndex> index) const noexcept
		{
			return index ? std::optional<Handle>(objectManager.bodies.handleAt(*index)) : std::nullopt;
		}
		std::optional<Handle> toColliderHandle(std::optional<ObjectDenseIndex> index) const noexcept
		{
			return index ? std::optional<Handle>(objectManager.colliders.handleAt(*index)) : std::nullopt;
		}
	};
}
