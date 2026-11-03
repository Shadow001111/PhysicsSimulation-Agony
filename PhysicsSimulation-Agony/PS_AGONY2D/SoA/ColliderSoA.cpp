#include "ColliderSoA.h"
#include "PS_AGONY2D/Constants.h"
#include <cmath>

namespace PS_AGONY2D
{
	ObjectDenseIndex ColliderSoA::append(
		ObjectDenseIndex bodyIndexIn,
		Vec2 localOffset,
		Real localRotationIn,
		MaterialIndex materialIndexIn,
		BodyType shapeTypeIn,
		ObjectDenseIndex shapeIndexIn
	)
	{
		const ObjectDenseIndex newIndex = getCount();

		this->bodyIndex.push_back(bodyIndexIn);

		this->localOffsetX.push_back(localOffset.x);
		this->localOffsetY.push_back(localOffset.y);

		Real wrappedLocal = std::fmod(localRotationIn, Constants::TWO_PI);
		if (wrappedLocal < Real(0)) wrappedLocal += Constants::TWO_PI;
		this->localRotation.push_back(wrappedLocal);

		this->worldPosX.push_back(0);
		this->worldPosY.push_back(0);
		this->worldRotation.push_back(wrappedLocal);
		this->worldRotationCos.push_back(std::cos(wrappedLocal)); // TODO: Use fast cos/sin.
		this->worldRotationSin.push_back(std::sin(wrappedLocal));

		this->materialIndex.push_back(materialIndexIn);
		this->shapeType.push_back(shapeTypeIn);
		this->shapeIndex.push_back(shapeIndexIn);

		this->aabbMinX.push_back(0);
		this->aabbMinY.push_back(0);
		this->aabbMaxX.push_back(0);
		this->aabbMaxY.push_back(0);

		this->callbackId.push_back(0);
		this->eventThreshold.push_back(NO_COLLISION_EVENT_THRESHOLD);

		this->handles.push();

		return newIndex;
	}

	void ColliderSoA::swapWithBack(ObjectDenseIndex index)
	{
		if (index >= bodyIndex.size()) [[unlikely]]
		{
			return;
		}
		handles.swapWithBack(index);

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
		handles.popBack();

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_COLLIDER_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD
	}

	ObjectDenseIndex ColliderSoA::swapRemove(ObjectDenseIndex index)
	{
		const ObjectDenseIndex lastIndex = bodyIndex.size() - 1;
		if (index != lastIndex)
		{
			swapWithBack(index);
		}
		popBack();
		return lastIndex;
	}

	size_t ColliderSoA::getMemoryUsage() const noexcept
	{
		size_t total = handles.getMemoryUsage();

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