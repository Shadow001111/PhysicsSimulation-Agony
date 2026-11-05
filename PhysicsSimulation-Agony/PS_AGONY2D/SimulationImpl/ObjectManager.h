#pragma once
#include "ObjectCreateParams.h"

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

		std::vector<ObjectDeletion> deletedColliders;

		// Bodies that changed sleep state since Simulation last drained them. Handles may be stale if destroyed later.
		std::vector<Handle> sleptBodies;
		std::vector<Handle> wokenBodies;

		const std::vector<Material>& materials;
		ConstraintSystemArray& constraintSystems;

		// Set when a collider is created/destroyed. Cleared by Simulation once the BVH trees are rebuilt.
		bool spatialDirty = false;
	private:
		// Persistent collider ID lists.
		// A collider lives in exactly one list, chosen by its body's isStatic flag (moved when mass toggles 0 <-> nonzero).
		std::vector<ObjectDenseIndex> dynamicColliderIDs;
		std::vector<ObjectDenseIndex> staticColliderIDs;
		std::vector<ObjectDenseIndex> sleepingColliderIDs;
		std::vector<ObjectDenseIndex> colliderSlot; // colliderIndex -> position inside its list.
	public:
		std::span<const ObjectDenseIndex> getDynamicColliderIDs() const noexcept { return dynamicColliderIDs; }
		std::span<const ObjectDenseIndex> getStaticColliderIDs() const noexcept { return staticColliderIDs; }
		std::span<const ObjectDenseIndex> getSleepingColliderIDs() const noexcept { return sleepingColliderIDs; }

		explicit ObjectManager(
			const std::vector<Material>& materials,
			ConstraintSystemArray& constraintSystems
		);

		// Creates a body with NO colliders attached. Mass/inertia are taken directly
		// from params; since there's no shape yet, inertia is not auto-derived and
		// attaching colliders afterward does not recompute mass/inertia/COM for you.
		std::optional<ObjectDenseIndex> createBody(const BodyCreateParams& params);

		// Attaches a new collider of the given shape to an EXISTING body immediately. Wakes the body if it sleeps.
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

		// Sleeping: moves the body to the Sleeping group and its colliders to the sleeping list. Body data is kept.
		// Any modification of a sleeping body or its colliders (commands, collider create/destroy) wakes it first.
		// Both return the NEW dense index (group moves change body indices). No-op if the body is in the wrong group.
		// Not safe for callbacks or during solving.
		//
		// Sleeping islands: bodies put to sleep together share a sleepIslandId (>= 2); pass it as sleepIslandId, or 0
		// for a standalone body (fresh id). wakeBody wakes the body's WHOLE island, so other bodies change index too:
		// only the returned index is valid afterwards. When sleeping many bodies, call in DESCENDING index order.
		ObjectDenseIndex sleepBody(ObjectDenseIndex bodyIndex, uint32_t sleepIslandId = 0);
		ObjectDenseIndex wakeBody(ObjectDenseIndex bodyIndex);

		// Wakes every Sleeping body whose sleepIslandId is in the list (sorted, unique, no zeros). One scan of the
		// Sleeping range regardless of list size.
		void wakeSleepIslands(std::span<const uint32_t> sortedUniqueIds);

		// Unique among live sleeping islands. Never returns 0 or 1.
		uint32_t newSleepIslandId() noexcept
		{
			if (++nextSleepIslandId < 2) nextSleepIslandId = 2;
			return nextSleepIslandId;
		}

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

		uint32_t nextSleepIslandId = 1;
		std::vector<ObjectDenseIndex> wakeScratch;

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
		void moveBodyColliders(ObjectDenseIndex bodyIndex, BodyGroup from, BodyGroup to);
		// Wakes exactly one body (no island logic). Returns its new index.
		ObjectDenseIndex wakeOne(ObjectDenseIndex bodyIndex);
		std::vector<ObjectDenseIndex>& colliderListFor(BodyGroup group) noexcept;

		// Body group machinery. Body memory order is [Dynamic][Static][Sleeping][Unused].
		// swapBodies: swaps two bodies and re-points their colliders. Constraint body indices are not remapped.
		void swapBodies(ObjectDenseIndex a, ObjectDenseIndex b);
		// Moves a body across group boundaries with boundary swaps (<= 3). Returns the new dense index. Collider lists untouched.
		ObjectDenseIndex moveBodyToGroup(ObjectDenseIndex bodyIndex, BodyGroup target);
		// BodySoA::append + moves the new body into Static (invMass == 0) or Dynamic. Returns its final index.
		ObjectDenseIndex appendBody(
			Vec2 pos, Vec2 vel, Real rot, Real anglVel,
			Real mass, Real invMass, Real inertia, Real invInertia,
			Vec2 localCenterOfMass
		);

		//
		ObjectDenseIndex createColliderInternal(
			ObjectDenseIndex bodyIndex, Vec2 localOffset, Real localRotation,
			MaterialIndex materialIndex, ColliderShapeType shapeType, ObjectDenseIndex shapeIndex
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