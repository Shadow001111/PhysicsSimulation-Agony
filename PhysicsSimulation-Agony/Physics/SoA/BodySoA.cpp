#include "BodySoA.h"
#include <cmath>
#include <utility>


namespace PS_AGONY
{
	void BodySoA::append(
		Vec2 pos, Vec2 vel, Real rot, Real anglVel,
		Real mass, Real invMass, Real inertia, Real invInertia,
		Vec2 localCenterOfMass
	)
	{
		this->renderOldOffsetX.push_back(pos.x);
		this->renderOldOffsetY.push_back(pos.y);
		this->renderOldRotation.push_back(rot);
		this->renderRotationWrapCount.push_back(0);

		this->offsetX.push_back(pos.x);
		this->offsetY.push_back(pos.y);
		this->localCenterOfMassX.push_back(localCenterOfMass.x);
		this->localCenterOfMassY.push_back(localCenterOfMass.y);
		this->worldCenterX.push_back(0);
		this->worldCenterY.push_back(0);
		this->velocityX.push_back(vel.x);
		this->velocityY.push_back(vel.y);
		this->rotation.push_back(rot);
		this->angularVelocity.push_back(anglVel);
		this->mass.push_back(mass);
		this->invMass.push_back(invMass);
		this->inertia.push_back(inertia);
		this->invInertia.push_back(invInertia);
		this->rotationCos.push_back(std::cos(rot));
		this->rotationSin.push_back(std::sin(rot));
		this->isStatic.push_back(invMass == 0);
		this->attachments.emplace_back();
		this->colliderIndices.emplace_back();
	}

	#define PS_AGONY_SWAP_WITH_BACK(vector, index) std::swap((vector)[(index)], (vector).back())

	void BodySoA::swapWithBack(size_t index)
	{
		if (index >= offsetX.size()) [[unlikely]]
		{
			return;
		}

		#define SWAP_FIELD(name, type) PS_AGONY_SWAP_WITH_BACK(name, index);
		PS_AGONY_BODY_SOA_SCALAR_FIELDS(SWAP_FIELD)
		#undef SWAP_FIELD

		#define SWAP_VECTOR_FIELD(name, innerType) PS_AGONY_SWAP_WITH_BACK(name, index);
		PS_AGONY_BODY_SOA_VECTOR_FIELDS(SWAP_VECTOR_FIELD)
		#undef SWAP_VECTOR_FIELD
	}

	void BodySoA::popBack()
	{
		if (offsetX.empty()) [[unlikely]]
		{
			return;
		}

		#define POP_FIELD(name, type) name.pop_back();
		PS_AGONY_BODY_SOA_SCALAR_FIELDS(POP_FIELD)
		#undef POP_FIELD

		#define POP_VECTOR_FIELD(name, innerType) name.pop_back();
		PS_AGONY_BODY_SOA_VECTOR_FIELDS(POP_VECTOR_FIELD)
		#undef POP_VECTOR_FIELD
	}

	void BodySoA::addAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex)
	{
		if (bodyIndex >= attachments.size()) [[unlikely]]
		{
			return;
		}
		attachments[bodyIndex].push_back({ type, objectIndex });
	}

	void BodySoA::removeAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex)
	{
		if (bodyIndex >= attachments.size()) [[unlikely]]
		{
			return;
		}
		auto& list = attachments[bodyIndex];
		for (size_t i = 0; i < list.size(); i++)
		{
			if (list[i].type == type && list[i].objectIndex == objectIndex)
			{
				list[i] = list.back();
				list.pop_back();
				return;
			}
		}
	}

	void BodySoA::addCollider(size_t bodyIndex, ColliderIndex colliderIndex)
	{
		if (bodyIndex >= colliderIndices.size()) [[unlikely]] return;
		colliderIndices[bodyIndex].push_back(colliderIndex);
	}

	void BodySoA::removeCollider(size_t bodyIndex, ColliderIndex colliderIndex)
	{
		if (bodyIndex >= colliderIndices.size()) [[unlikely]] return;
		auto& list = colliderIndices[bodyIndex];
		for (size_t i = 0; i < list.size(); i++)
		{
			if (list[i] == colliderIndex)
			{
				list[i] = list.back();
				list.pop_back();
				return;
			}
		}
	}

	size_t BodySoA::getMemoryUsage() const noexcept
	{
		size_t total = 0;

		#define MEM_FIELD(name, type) total += getVectorMemoryUsage(name);
		PS_AGONY_BODY_SOA_SCALAR_FIELDS(MEM_FIELD)
		#undef MEM_FIELD

		#define MEM_VECTOR_FIELD(name, innerType) \
		total += getVectorMemoryUsage(name); \
		for (const auto& innerVec : name) { total += getVectorMemoryUsage(innerVec); }

		PS_AGONY_BODY_SOA_VECTOR_FIELDS(MEM_VECTOR_FIELD)
		#undef MEM_VECTOR_FIELD

		return total;
	}

	BodySoAViewer::BodySoAViewer(const BodySoA& data) noexcept :
		count(data.getCount())
	{
		#define INIT_VIEWER_FIELD(name, type) name = data.name.data();
		PS_AGONY_BODY_SOA_SCALAR_FIELDS(INIT_VIEWER_FIELD)
		PS_AGONY_BODY_SOA_VECTOR_FIELDS(INIT_VIEWER_FIELD)
		#undef INIT_VIEWER_FIELD
	}
}
