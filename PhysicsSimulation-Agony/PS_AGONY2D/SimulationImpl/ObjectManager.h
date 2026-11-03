#pragma once
#include "PolygonDecomposition.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/ConstraintSystem.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/Material.h"

#include <cstdint>
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
		ObjectDenseIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Real radius{ 0 };
	};

	struct BoxColliderCreateParams
	{
		ObjectDenseIndex bodyIndex;
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		Vec2 size{ 0 };
	};

	struct PolygonColliderCreateParams
	{
		ObjectDenseIndex bodyIndex;
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
		ObjectDenseIndex bodyIndex;
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
		ObjectDenseIndex firstColliderIndex{ 0 };
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

	// Instant snapshot of a collider and its shape. Only the fields of its shapeType are meaningful.
	// 'polygon' points into SoA storage: valid only until the next create/destroy/flush.
	struct ColliderData
	{
		Handle bodyHandle{};
		Vec2 localOffset{ 0 };
		Real localRotation{ 0 };
		Vec2 worldPosition{ 0 };
		Real worldRotation{ 0 };
		MaterialIndex materialIndex{ 0 };
		BodyType shapeType{};
		CollisionCallbackIndex callbackId{ 0 };
		Real eventThreshold{ 0 };
		Real radius{ 0 };            // Circle, Capsule.
		Vec2 halfExtents{ 0 };       // Box.
		Real halfLength{ 0 };        // Capsule.
		const PolygonGeometry* polygon = nullptr; // Polygon.
	};

	// Body commands must stay before SetColliderLocalOffset (used to route resolution).
	enum class ObjectCommandType : uint8_t
	{
		DestroyBody,
		SetBodyPosition,
		SetBodyRotation,
		SetBodyVelocity,
		SetBodyAngularVelocity,
		SetBodyMass,
		SetBodyInertia,
		SetBodyLocalCenterOfMass,
		RecomputeBodyInertia,
		SetBodyMassFromDensity,

		DestroyCollider,
		SetColliderLocalOffset,
		SetColliderLocalRotation,
		SetColliderMaterial,
		SetCircleRadius,
		SetBoxSize,
		SetCapsuleShape,
		SetPolygonGeometry
	};

	struct ObjectCommand
	{
		ObjectCommandType type{};
		Handle handle{};
		Real a{ 0 };
		Real b{ 0 };
		uint32_t index{ 0 }; // Material index or pending polygon slot.
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
		ConstraintSystemArray& constraintSystems;

		// Set when a collider is created/destroyed. Cleared by Simulation once the BVH trees are rebuilt.
		bool spatialDirty = false;
	private:
		// Persistent collider ID lists (maintained in O(1) on create/destroy; never rebuilt per frame).
		// A collider lives in exactly one list, chosen by its body's isStatic flag (moved when mass toggles 0 <-> nonzero).
		std::vector<ObjectDenseIndex> dynamicColliderIDs;
		std::vector<ObjectDenseIndex> staticColliderIDs;
		std::vector<uint32_t> colliderSlot; // colliderIndex -> position inside its list.
	public:
		std::span<const ObjectDenseIndex> getDynamicColliderIDs() const noexcept { return dynamicColliderIDs; }
		std::span<const ObjectDenseIndex> getStaticColliderIDs() const noexcept { return staticColliderIDs; }

		explicit ObjectManager(
			const std::vector<Material>& materials,
			ConstraintSystemArray& constraintSystems
		);

		// Creates a body with NO colliders attached. Mass/inertia are taken directly
		// from params; since there's no shape yet, inertia is not auto-derived and
		// attaching colliders afterward does not recompute mass/inertia/COM for you.
		std::optional<ObjectDenseIndex> createBody(const BodyCreateParams& params);

		// Attaches a new collider of the given shape to an EXISTING body immediately.
		// Returns std::nullopt if bodyIndex is invalid.
		std::optional<ObjectDenseIndex> createCircleCollider(const CircleColliderCreateParams& params);
		std::optional<ObjectDenseIndex> createBoxCollider(const BoxColliderCreateParams& params);
		std::optional<PolygonColliderCreateResult> createPolygonCollider(const PolygonColliderCreateParams& params);
		std::optional<ObjectDenseIndex> createCapsuleCollider(const CapsuleColliderCreateParams& params);

		std::optional<ObjectDenseIndex> createCircle(const CircleCreateParams& params);
		std::optional<ObjectDenseIndex> createBox(const BoxCreateParams& params);
		std::optional<ObjectDenseIndex> createPolygon(const PolygonCreateParams& params);
		std::optional<ObjectDenseIndex> createCapsule(const CapsuleCreateParams& params);

		// Destroys a body and cascade-deletes every collider/spring attached to it.
		// Not safe for callbacks.
		void destroyBodyImmediate(ObjectDenseIndex bodyIndex);

		// Destroys a single collider without touching its owning body.
		// Not safe for callbacks.
		void destroyColliderImmediate(ObjectDenseIndex colliderIndex);

		void destroyBody(Handle handle) { pushCommand(ObjectCommandType::DestroyBody, handle); }
		void destroyCollider(Handle handle) { pushCommand(ObjectCommandType::DestroyCollider, handle); }

		// Deferred modification commands.
		// Queued in call order, applied by flushCommands(). Stale handles and shape-type mismatches are dropped.
		// Values are sanitized on apply (negatives clamped, angles wrapped, material index validated).
		// Body position is the body origin offset (BodyData::offset), not the world center of mass.
		void queueBodyPosition(Handle body, Vec2 position) { pushCommand(ObjectCommandType::SetBodyPosition, body, position.x, position.y); }
		void queueBodyRotation(Handle body, Real rotation) { pushCommand(ObjectCommandType::SetBodyRotation, body, rotation); }
		void queueBodyVelocity(Handle body, Vec2 velocity) { pushCommand(ObjectCommandType::SetBodyVelocity, body, velocity.x, velocity.y); }
		void queueBodyAngularVelocity(Handle body, Real angularVelocity) { pushCommand(ObjectCommandType::SetBodyAngularVelocity, body, angularVelocity); }

		// Mass 0 -> static: inertia and velocities zeroed, colliders move to the static list.
		// Nonzero -> dynamic. Inertia is scaled by newMass / oldMass; coming from 0 it is left as is (usually 0),
		// so queue queueRecomputeBodyInertia afterwards. Ignored for inertia of static bodies.
		void queueBodyMass(Handle body, Real mass) { pushCommand(ObjectCommandType::SetBodyMass, body, mass); }
		void queueBodyInertia(Handle body, Real inertia) { pushCommand(ObjectCommandType::SetBodyInertia, body, inertia); }

		// Moves the rotation pivot only; collider geometry is untouched. Does not recompute inertia.
		void queueBodyLocalCenterOfMass(Handle body, Vec2 localCenterOfMass) { pushCommand(ObjectCommandType::SetBodyLocalCenterOfMass, body, localCenterOfMass.x, localCenterOfMass.y); }

		// Inertia from current mass + collider shapes + local COM (mass split by area, parallel axis). No-op for static
		// bodies or bodies without area.
		void queueRecomputeBodyInertia(Handle body) { pushCommand(ObjectCommandType::RecomputeBodyInertia, body); }

		// mass = density * total collider area (no-op if the body has no area). Density 0 makes the body static.
		void queueBodyMassFromDensity(Handle body, Real density) { pushCommand(ObjectCommandType::SetBodyMassFromDensity, body, density); }

		void queueColliderLocalOffset(Handle collider, Vec2 localOffset) { pushCommand(ObjectCommandType::SetColliderLocalOffset, collider, localOffset.x, localOffset.y); }
		void queueColliderLocalRotation(Handle collider, Real localRotation) { pushCommand(ObjectCommandType::SetColliderLocalRotation, collider, localRotation); }
		void queueColliderMaterial(Handle collider, MaterialIndex materialIndex) { pushCommand(ObjectCommandType::SetColliderMaterial, collider, 0, 0, static_cast<uint32_t>(materialIndex)); }

		// Shape commands do nothing if the collider's shape type differs. Mass/inertia are NOT touched.
		void queueCircleRadius(Handle collider, Real radius) { pushCommand(ObjectCommandType::SetCircleRadius, collider, radius); }
		void queueBoxSize(Handle collider, Vec2 size) { pushCommand(ObjectCommandType::SetBoxSize, collider, size.x, size.y); }
		void queueCapsuleShape(Handle collider, Real length, Real radius) { pushCommand(ObjectCommandType::SetCapsuleShape, collider, length, radius); }

		// Validated now (>= 3 vertices, simple, convex, <= MAX_POLYGON_VERTEX_COUNT after cleanup); false = nothing queued.
		// Vertices are collider-local and are not re-centered.
		bool queueColliderPolygon(Handle collider, std::span<const Vec2> localVertices);

		// Applies and clears the queue. Must not run during collision-event dispatch or solving.
		void flushCommands();

		// Instant reads (no queue). nullopt if index is out of range.
		std::optional<ColliderData> getColliderData(ObjectDenseIndex colliderIndex) const noexcept;
	private:
		std::vector<ObjectCommand> commands;
		std::vector<PolygonGeometry> pendingPolygons;

		void pushCommand(ObjectCommandType type, Handle handle, Real a = 0, Real b = 0, uint32_t index = 0)
		{
			ObjectCommand command;
			command.type = type;
			command.handle = handle;
			command.a = a;
			command.b = b;
			command.index = index;
			commands.push_back(command);
		}

		void applyCommand(const ObjectCommand& command);
		void applyBodyMass(ObjectDenseIndex bodyIndex, Real newMass);
		void applyBodyMassFromDensity(ObjectDenseIndex bodyIndex, Real density);
		void recomputeBodyInertia(ObjectDenseIndex bodyIndex);
		void moveBodyColliders(ObjectDenseIndex bodyIndex, bool toStatic);

		//
		ObjectDenseIndex createColliderInternal(
			ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation,
			MaterialIndex materialIndex, BodyType shapeType, ObjectDenseIndex shapeIndex
		);

		// Appends a single convex-polygon collider (assumed already convex, CCW, >=3 vertices -
		// no validation is performed here) to bodyIndex. Shared by createPolygon() and
		// createPolygonCollider() so the collider/shape-SoA wiring is written once.
		ObjectDenseIndex createConvexPolygonColliderInternal(
			ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation,
			MaterialIndex materialIndex, std::vector<Vec2> convexVertices
		);

		// Removes a single collider (and its underlying shape-SoA entry) from bodyIndex's
		// owned list. Shared by destroyCollider() and destroyBody()'s cascade loop.
		void destroyColliderInternal(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIndex);
	};
}