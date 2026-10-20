#include "ColliderSoA.h"
#include "Physics/Constants.h"
#include <cmath>
#include <utility>

namespace PS_AGONY
{
	ColliderIndex ColliderSoA::append(
		ObjectIndex bodyIndexIn,
		Vec2 localOffset,
		Real localRotationIn,
		MaterialIndex materialIndexIn,
		BodyType shapeTypeIn,
		ObjectIndex shapeIndexIn
	)
	{
		const ColliderIndex newIndex = static_cast<ColliderIndex>(bodyIndex.size());

		bodyIndex.push_back(bodyIndexIn);

		localOffsetX.push_back(localOffset.x);
		localOffsetY.push_back(localOffset.y);

		// Wrap once here; local rotation is rigid and never changes afterward.
		Real wrappedLocal = std::fmod(localRotationIn, Constants::TWO_PI);
		if (wrappedLocal < Real(0)) wrappedLocal += Constants::TWO_PI;
		localRotation.push_back(wrappedLocal);

		worldPosX.push_back(0);
		worldPosY.push_back(0);
		worldRotation.push_back(wrappedLocal);
		worldRotationCos.push_back(std::cos(wrappedLocal));
		worldRotationSin.push_back(std::sin(wrappedLocal));

		materialIndex.push_back(materialIndexIn);
		shapeType.push_back(shapeTypeIn);
		shapeIndex.push_back(shapeIndexIn);

		aabbMinX.push_back(0);
		aabbMinY.push_back(0);
		aabbMaxX.push_back(0);
		aabbMaxY.push_back(0);

		callbackId.push_back(0);
		eventThreshold.push_back(NO_COLLISION_EVENT_THRESHOLD);

		return newIndex;
	}

	void ColliderSoA::swapWithBack(size_t index)
	{
		if (index >= bodyIndex.size()) [[unlikely]]
		{
			return;
		}

		#define SWAP_FIELD(name, type) std::swap(name[index], name.back());
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(SWAP_FIELD)
			#undef SWAP_FIELD
	}

	void ColliderSoA::popBack()
	{
		if (bodyIndex.empty()) [[unlikely]]
		{
			return;
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(POP_FIELD)
			#undef POP_FIELD
	}

	size_t ColliderSoA::swapRemove(size_t index)
	{
		const size_t lastIndex = bodyIndex.size() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t ColliderSoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(MEM_FIELD)
			#undef MEM_FIELD

			return total;
	}

	ColliderSoAViewer::ColliderSoAViewer(const ColliderSoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
			#undef INIT_VIEWER_FIELD
	}

	AABBSoAViewer::AABBSoAViewer(const ColliderSoA& data) noexcept :
		count(data.getCount()),
		minX(data.aabbMinX.data()),
		minY(data.aabbMinY.data()),
		maxX(data.aabbMaxX.data()),
		maxY(data.aabbMaxY.data())
	{
	}
}