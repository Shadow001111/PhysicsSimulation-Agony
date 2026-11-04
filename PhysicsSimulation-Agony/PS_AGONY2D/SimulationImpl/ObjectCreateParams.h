#pragma once
#include "PolygonDecomposition.h"

#include "PS_AGONY2D/ObjectSoA.h"
#include "PS_AGONY2D/Types/Types.h"

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
		ColliderShapeType shapeType{};
		CollisionCallbackIndex callbackId{ 0 };
		Real eventThreshold{ 0 };
		Real radius{ 0 };            // Circle, Capsule.
		Vec2 halfExtents{ 0 };       // Box.
		Real halfLength{ 0 };        // Capsule.
		const PolygonGeometry* polygon = nullptr; // Polygon.
	};
}