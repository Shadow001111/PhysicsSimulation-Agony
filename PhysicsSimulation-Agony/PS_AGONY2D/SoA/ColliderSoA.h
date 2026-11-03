#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/HandleTable.h"
#include "PS_AGONY2D/Types/Types.h"
#include "PS_AGONY2D/Types/CollisionEvent.h"

// Define all scalar SoA fields.
#define PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(F) \
	F(bodyIndex, ObjectDenseIndex) \
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
	F(shapeIndex, ObjectDenseIndex) \
	F(aabbMinX, Real) \
	F(aabbMinY, Real) \
	F(aabbMaxX, Real) \
	F(aabbMaxY, Real) \
	F(callbackId, CollisionCallbackIndex) \
	F(eventThreshold, Real)

namespace PS_AGONY2D
{
	struct ColliderSoA
	{
		#define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(DECLARE_FIELD)
		#undef DECLARE_FIELD

		HandleTable handles;

		ObjectDenseIndex append(
			ObjectDenseIndex bodyIndexIn,
			Vec2 localOffset,
			Real localRotationIn,
			MaterialIndex materialIndexIn,
			BodyType shapeTypeIn,
			ObjectDenseIndex shapeIndexIn
		);

		void swapWithBack(ObjectDenseIndex index);
		void popBack();
		ObjectDenseIndex swapRemove(ObjectDenseIndex index);

		bool isValid(Handle h) const noexcept { return handles.isValid(h); }
		ObjectDenseIndex indexOf(Handle h) const noexcept { return handles.indexOf(h); }
		ObjectDenseIndex indexOfUnchecked(Handle h) const noexcept { return handles.indexOfUnchecked(h); }
		Handle handleAt(ObjectDenseIndex index) const noexcept { return handles.handleAt(index); }

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