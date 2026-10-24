#pragma once
#include "PolygonDecomposition.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/ConstraintSystem.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/Material.h"

#include <optional>
#include <span>
#include <vector>

namespace PS_AGONY2D
{
	struct BodyCreateParams
	{
		Vec2 position{};
		Vec2 velocity{};
		Real rotation{ 0 };
		Real angularVelocity{ 0 };
		Real mass{ 0 };
		Real inertia{ 0 };
		std::optional<Vec2> centerOfMass = std::nullopt;
		MaterialIndex materialIndex{ 0 };
	};

	// Params for attaching a new collider to an EXISTING body immediately (no standalone colliders).
	struct CircleColliderCreateParams
	{
		ObjectIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Real radius{ 0 };
	};

	struct BoxColliderCreateParams
	{
		ObjectIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Vec2 size{ 0 };
	};

	struct PolygonColliderCreateParams
	{
		ObjectIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Vec2* localVertices = nullptr;
		size_t verticesCount = 0;
		PhysicsGeometry::ConcaveDecompositionMethod concaveDecompositionMethod = PhysicsGeometry::ConcaveDecompositionMethod::Bayazit;
		PhysicsGeometry::ConvexDecompositionMethod convexDecompositionMethod = PhysicsGeometry::ConvexDecompositionMethod::Fan;
	};

	// If 'localStart' and 'localEnd' are set, they dominate.
	struct CapsuleColliderCreateParams
	{
		ObjectIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Real length{ 0 };
		Real radius{ 0 };
		std::optional<Vec2> localStart{};
		std::optional<Vec2> localEnd{};
	};

	// Result of createPolygonCollider(). A concave input outline may be split into several
	// convex colliders on the same body; they are appended back-to-back in colliders/polygons
	// SoA storage during that single call, so every valid collider index is contiguous:
	// [firstColliderIndex, firstColliderIndex + colliderCount).
	struct PolygonColliderCreateResult
	{
		ColliderIndex firstColliderIndex{ 0 };
		size_t colliderCount{ 0 };
	};

	struct CircleCreateParams
	{
		BodyCreateParams base; // Would better to just inherit, but field initializer can't work like this :c. For now.
		Real radius{ 0 };
	};

	struct BoxCreateParams
	{
		BodyCreateParams base;
		Vec2 size{ 0 };
	};

	struct PolygonCreateParams
	{
		BodyCreateParams base;
		std::span<Vec2> localVertices;
		PhysicsGeometry::ConcaveDecompositionMethod concaveDecompositionMethod = PhysicsGeometry::ConcaveDecompositionMethod::Bayazit;
		PhysicsGeometry::ConvexDecompositionMethod convexDecompositionMethod = PhysicsGeometry::ConvexDecompositionMethod::Fan;
	};

	// If 'start' and 'end' are set, they dominate.
	struct CapsuleCreateParams
	{
		BodyCreateParams base;
		Real length{ 0 };
		Real radius{ 0 };
		std::optional<Vec2> start{};
		std::optional<Vec2> end{};
	};

	class ObjectManager
	{
	public:
		BodySoA bodies;
		ColliderSoA colliders;
		CircleSoA circles;
		BoxSoA boxes;
		PolygonSoA polygons;
		CapsuleSoA capsules;

		std::vector<ObjectDeletion> deletedBodies;
		std::vector<ObjectDeletion> deletedColliders;

		const std::vector<Material>& materials;

		// Set when a collider is created/destroyed. Cleared by Simulation once the BVH trees are rebuilt.
		bool spatialDirty = false;

	private:
		// Persistent collider ID lists (maintained in O(1) on create/destroy; never rebuilt per frame).
		// A collider lives in exactly one list, chosen by its body's isStatic flag (fixed at creation).
		std::vector<ObjectIndex> dynamicColliderIDs;
		std::vector<ObjectIndex> staticColliderIDs;
		std::vector<uint32_t> colliderSlot; // colliderIndex -> position inside its list.

	public:
		std::span<const ObjectIndex> getDynamicColliderIDs() const noexcept { return dynamicColliderIDs; }
		std::span<const ObjectIndex> getStaticColliderIDs() const noexcept { return staticColliderIDs; }

		explicit ObjectManager(const std::vector<Material>& materials);

		// Creates a body with NO colliders attached. Mass/inertia are taken directly
		// from params; since there's no shape yet, inertia is not auto-derived and
		// attaching colliders afterward does not recompute mass/inertia/COM for you.
		std::optional<ObjectIndex> createBody(const BodyCreateParams& params);

		// Attaches a new collider of the given shape to an EXISTING body immediately.
		// Returns std::nullopt if bodyIndex is invalid.
		std::optional<ColliderIndex> createCircleCollider(const CircleColliderCreateParams& params);
		std::optional<ColliderIndex> createBoxCollider(const BoxColliderCreateParams& params);
		std::optional<PolygonColliderCreateResult> createPolygonCollider(const PolygonColliderCreateParams& params);
		std::optional<ColliderIndex> createCapsuleCollider(const CapsuleColliderCreateParams& params);

		std::optional<ObjectIndex> createCircle(const CircleCreateParams& params);
		std::optional<ObjectIndex> createBox(const BoxCreateParams& params);
		std::optional<ObjectIndex> createPolygon(const PolygonCreateParams& params);
		std::optional<ObjectIndex> createCapsule(const CapsuleCreateParams& params);

		// Destroys a body and cascade-deletes every collider/spring attached to it.
		void destroyBody(ObjectIndex bodyIndex, ConstraintSystemArray& constraintSystems);

		// Destroys a single collider without touching its owning body.
		void destroyCollider(ColliderIndex colliderIndex, ConstraintSystemArray& constraintSystems);
	private:
		//
		ColliderIndex createColliderInternal(
			ObjectIndex bodyIndex, Vec2 localOffset, Real localRotation,
			MaterialIndex materialIndex, BodyType shapeType, ObjectIndex shapeIndex
		);

		// Appends a single convex-polygon collider (assumed already convex, CCW, >=3 vertices -
		// no validation is performed here) to bodyIndex. Shared by createPolygon() and
		// createPolygonCollider() so the collider/shape-SoA wiring is written once.
		ColliderIndex createConvexPolygonColliderInternal(
			ObjectIndex bodyIndex, Vec2 localOffset, Real localRotation,
			MaterialIndex materialIndex, std::vector<Vec2> convexVertices
		);

		// Removes a single collider (and its underlying shape-SoA entry) from bodyIndex's
		// owned list. Shared by destroyCollider() and destroyBody()'s cascade loop.
		void destroyColliderInternal(ObjectIndex bodyIndex, ColliderIndex colliderIndex);
	};
}