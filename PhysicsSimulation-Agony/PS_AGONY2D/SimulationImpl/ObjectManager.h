#pragma once
#include "ObjectCreateParams.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/Material.h"

#include "PS_AGONY2D/SimulationImpl/AllConstraintSystem.h"
#include "PS_AGONY2D/IslandManager.h"
#include "PS_AGONY2D/Interactivity/BodyHolder.h"

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

	// TODO: Add internal stuff to getMemoryUsage.
	class ObjectManager
	{
	public:
		BodySoA bodies;
		ColliderSoA colliders;
		CircleSoA circles;
		BoxSoA boxes;
		PolygonSoA polygons;
		CapsuleSoA capsules;

		// Set by sleepBody, cleared by consumeSleptFlag.
		bool sleptSincePass = false;

		const std::vector<Material>& materials;
		ConstraintSystemArray& constraintSystems;

		// Set when a collider is created/destroyed. Cleared by Simulation once the BVH trees are rebuilt.
		bool spatialDirty = false;

		// Static colliders modified or destroyed since the last clear (by handle). Their saved contacts are stale.
		std::span<const Handle> getEditedStaticColliders() const noexcept { return editedStaticColliders; }
		void clearEditedStaticColliders() noexcept { editedStaticColliders.clear(); }

		// Marks the collider as edited if its body is Static (no-op otherwise), so islands resting on it wake.
		void markColliderEdited(ObjectDenseIndex colliderIndex) { markStaticColliderEdited(colliderIndex); }

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

		// Constraint lifetime. ObjectManager owns body attachments; creating/destroying wakes the touched bodies
		// (and their sleeping islands). Not safe for callbacks or during solving.
		// createSpring: nullopt if a body handle is stale.
		std::optional<Handle> createSpring(const SpringConstraintSystem::CreateParams& params);
		std::optional<Handle> createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& params);
		std::optional<Handle> createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& params);
		std::optional<Handle> createSliderJoint(const SliderJointConstraintSystem::CreateParams& params);
		std::optional<Handle> createMotor(const MotorConstraintSystem::CreateParams& params);

		void destroySpringImmediate(Handle h);
		void destroyRevoluteJointImmediate(Handle h);
		void destroyPrismaticJointImmediate(Handle h);
		void destroySliderJointImmediate(Handle h);
		void destroyMotorImmediate(Handle h);
		void destroyConstraintImmediate(ConstraintType type, ObjectDenseIndex constraintIndex);

		// Sleeping: moves the body to the Sleeping group and its colliders to the sleeping list. Body data is kept.
		// Any modification of a sleeping body or its colliders (commands, collider create/destroy) wakes it first.
		// Both return the NEW dense index (group moves change body indices). No-op if the body is in the wrong group.
		// Not safe for callbacks or during solving.
		//
		// Sleeping islands: bodies put to sleep together share a sleepIslandId (>= 2); pass it as sleepIslandId, or 0
		// for a standalone body (fresh id). wakeBody wakes the body's WHOLE island, so other bodies change index too:
		// only the returned index is valid afterwards. Single calls only: batches go through updateSleeping and
		// wakeSleepIslands, which order the moves by SLEEPING_AFTER_DYNAMIC.
		ObjectDenseIndex sleepBody(ObjectDenseIndex bodyIndex, uint32_t sleepIslandId = 0);
		ObjectDenseIndex wakeBody(ObjectDenseIndex bodyIndex);

		// Wakes the given sleeping islands (ids >= 2, any order, duplicates and dead ids are skipped).
		// Cost is proportional to the woken bodies only: members come from the island's own list.
		// Woken ids are queued for takeWokenIslandIds.
		void wakeSleepIslands(std::span<const uint32_t> sortedUniqueIds);

		// Wakes the sleeping islands of every body in the contacts. Contact needs colliderA and colliderB handles.
		template<class ContactRange>
		void wakeIslandsOfContacts(const ContactRange& contacts)
		{
			const IndexRange sleeping = bodies.range(BodyGroup::Sleeping);
			if (sleeping.empty()) return;

			const ObjectDenseIndex* const bodyIndexPtr = colliders.bodyIndex.data();
			const uint32_t* const sleepIdPtr = bodies.sleepIslandId.data();

			wakeIdScratch.clear();
			for (const auto& contact : contacts)
			{
				const ObjectDenseIndex bodyA = bodyIndexPtr[colliders.handles.indexOfUnchecked(contact.colliderA)];
				const ObjectDenseIndex bodyB = bodyIndexPtr[colliders.handles.indexOfUnchecked(contact.colliderB)];

				if (bodyA - sleeping.begin < sleeping.size()) pushWakeId(sleepIdPtr[bodyA]);
				if (bodyB - sleeping.begin < sleeping.size()) pushWakeId(sleepIdPtr[bodyB]);
			}
			wakeCollectedIslandIds();
		}

		// Advances sleep timers of Dynamic bodies and sleeps every island whose bodies are all quiet long enough.
		// Islands containing held bodies stay awake.
		void updateSleeping(
			float deltaTime, Real linearSpeedThreshold, Real angularSpeedThreshold, float sleepTimeThreshold,
			const IslandManager& islandManager, const Interactivity::BodyHolder& holder
		);

		// Island ids woken since the last call, in wake order. Also releases the ids for reuse, so the caller must
		// consume the list (clear the island's stored data) before the next sleep pass. Reused by the next call.
		std::span<const uint32_t> takeWokenIslandIds();
		bool hasWokenIslands() const noexcept { return !wokenIslandIds.empty(); }

		// True if any body was put to sleep since the last call (its contact data has to be stashed). Clears the flag.
		bool consumeSleptFlag() noexcept { const bool v = sleptSincePass; sleptSincePass = false; return v; }

		size_t getSleepMemoryUsage() const noexcept;

		// Unique among live sleeping islands (id = pool slot + 2, recycled after the wake is drained). Never 0 or 1.
		uint32_t newSleepIslandId();

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
		// Commands include bodies and constraints.
		void flushCommands();

		// Instant reads (no queue). nullopt if index is out of range.
		std::optional<ColliderData> getColliderData(ObjectDenseIndex colliderIndex) const noexcept;
	private:
		std::vector<ObjectCommand> commands;
		std::vector<PolygonGeometry> pendingPolygons;

		struct SleepIsland
		{
			std::vector<Handle> bodies; // Members, by handle (dense indices move).
			bool live = false;
		};
		std::vector<SleepIsland> sleepIslands; // Indexed by sleepIslandId - 2.
		std::vector<uint32_t> freeIslandSlots;
		std::vector<uint32_t> pendingFreeIslandSlots; // Woken, released by takeWokenIslandIds.
		std::vector<uint32_t> wokenIslandIds;
		std::vector<uint32_t> wokenIslandIdsTaken;
		void pushWakeId(uint32_t id) { if (wakeIdScratch.empty() || wakeIdScratch.back() != id) wakeIdScratch.push_back(id); }
		std::vector<ObjectDenseIndex> wakeScratch;
		std::vector<uint32_t> wakeIdScratch;
		std::vector<ObjectDenseIndex> sleepScratch;
		std::vector<uint32_t> islandSleepFlags; // 0 = stays awake, 1 = may sleep, >= 2 = assigned sleep island id.
		std::vector<Handle> constraintWakeScratch;
		std::vector<Handle> editedStaticColliders;

		// No-ops unless the owning body is Static.
		void markStaticColliderEdited(ObjectDenseIndex colliderIndex);
		void markStaticBodyEdited(ObjectDenseIndex bodyIndex);

		// Wakes both bodies (indices move: re-resolved by handle). False if either handle is stale.
		bool wakeAndResolvePair(Handle hA, Handle hB, ObjectDenseIndex& a, ObjectDenseIndex& b);
		// Two-body create: wake, append via System::createAt, attach to both bodies. Defined in the .cpp.
		template<class System, class Params>
		std::optional<Handle> createPairConstraint(ConstraintType type, const Params& params);
		template<class System>
		void destroyConstraintByHandle(ConstraintType type, Handle h);

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
		// Moves every collider of the body (and their shapes) to the group.
		void moveBodyColliders(ObjectDenseIndex bodyIndex, BodyGroup to);
		// Wakes exactly one body (no island logic). Returns its new index.
		ObjectDenseIndex wakeOne(ObjectDenseIndex bodyIndex);
		// Wakes the islands in wakeIdScratch (duplicates are skipped).
		void wakeCollectedIslandIds();
		// Moves a body across the boundary shared by two adjacent groups with a single swap. Returns the new index.
		template<BodyGroup FROM, BodyGroup TO>
		ObjectDenseIndex moveBodyToAdjacentGroup(ObjectDenseIndex bodyIndex);

		// Collider/shape group machinery (same scheme as bodies, see moveBodyToGroup).
		// swapColliders: swaps two colliders and re-points owner bodies' lists and shape back-references.
		void swapColliders(ObjectDenseIndex a, ObjectDenseIndex b);
		// swapShapes: swaps two entries of one shape SoA and re-points their colliders.
		void swapShapes(ColliderShapeType type, ObjectDenseIndex a, ObjectDenseIndex b);
		GroupRanges& shapeGroups(ColliderShapeType type) noexcept;
		size_t shapeCount(ColliderShapeType type) const noexcept;
		void popBackShape(ColliderShapeType type);
		// Moves a collider and its shape across group boundaries (boundary swaps). Returns the new collider index.
		ObjectDenseIndex moveColliderToGroup(ObjectDenseIndex colliderIndex, BodyGroup target);
		// Moves a just-appended collider (+ its already appended shape) from Unused to its body's group. Returns the new collider index.
		ObjectDenseIndex placeNewCollider(ObjectDenseIndex colliderIndex);
		std::vector<Handle> colliderMoveScratch;

		// Body group machinery. Body memory order follows BodyGroup.
		// swapBodies: swaps two bodies and re-points their colliders. Constraint body indices are not remapped.
		void swapBodies(ObjectDenseIndex a, ObjectDenseIndex b);

		// Constraint group machinery (per system, see ConstraintGroup). Same scheme as bodies.
		// swapConstraints: swaps two constraints of one system and relinks their body attachments.
		void swapConstraints(ConstraintType type, ObjectDenseIndex a, ObjectDenseIndex b);
		// Moves a constraint across group boundaries (boundary swaps). Returns the new dense index.
		ObjectDenseIndex moveConstraintToGroup(ConstraintType type, ObjectDenseIndex index, ConstraintGroup target);
		// Inactive if every body is static, else Active (Sleeping constraints are left alone). Returns the new index.
		ObjectDenseIndex refreshConstraintGroup(ConstraintType type, ObjectDenseIndex index);
		// Refreshes every constraint attached to the body (call after its static/dynamic state changed).
		void refreshBodyConstraintGroups(ObjectDenseIndex bodyIndex);
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