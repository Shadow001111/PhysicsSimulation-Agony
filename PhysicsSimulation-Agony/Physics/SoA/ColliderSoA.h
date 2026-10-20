#pragma once
#include "Physics/ContainerUtilities.h"
#include "Physics/Types.h"
#include "Physics/CollisionEvent.h"

// Define all scalar SoA fields.
// localRotation: rigid local angle; wrapped to [0, 2pi) once, at creation.
// worldRotation: body.rotation + localRotation, re-wrapped every step.
// callbackId: collision callback slot (0 = none). eventThreshold: impact impulse needed to fire it.
#define PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(F) \
	F(bodyIndex, ObjectIndex) \
	F(localOffsetX, Real) \
	F(localOffsetY, Real) \
	F(localRotation, Real) \
	F(worldPosX, Real) \
	F(worldPosY, Real) \
	F(worldRotation, Real) \
	F(worldRotationCos, Real) \
	F(worldRotationSin, Real) \
	F(materialIndex, MaterialIndex) \
	F(shapeType, BodyType) \
	F(shapeIndex, ObjectIndex) \
	F(aabbMinX, Real) \
	F(aabbMinY, Real) \
	F(aabbMaxX, Real) \
	F(aabbMaxY, Real) \
	F(callbackId, CollisionCallbackIndex) \
	F(eventThreshold, Real)

namespace PS_AGONY
{
	struct ColliderSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(DECLARE_FIELD)
			#undef DECLARE_FIELD

			ColliderIndex append(
				ObjectIndex bodyIndexIn,
				Vec2 localOffset,
				Real localRotationIn,
				MaterialIndex materialIndexIn,
				BodyType shapeTypeIn,
				ObjectIndex shapeIndexIn
			);

		void swapWithBack(size_t index);
		void popBack();
		size_t swapRemove(size_t index);

		size_t getCount() const noexcept { return bodyIndex.size(); }
		size_t getMemoryUsage() const noexcept;
	};

	class ColliderSoAViewer
	{
		size_t count = 0;
	public:
		#define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
			#undef DECLARE_VIEWER_FIELD

			ColliderSoAViewer() = default;
		explicit ColliderSoAViewer(const ColliderSoA& data) noexcept;

		size_t getCount() const noexcept { return count; }
	};

	class AABBSoAViewer
	{
		size_t count = 0;
	public:
		const Real* minX = nullptr;
		const Real* minY = nullptr;
		const Real* maxX = nullptr;
		const Real* maxY = nullptr;

		AABBSoAViewer() = default;
		explicit AABBSoAViewer(const ColliderSoA& data) noexcept;

		size_t getCount() const noexcept { return count; }
	};
}