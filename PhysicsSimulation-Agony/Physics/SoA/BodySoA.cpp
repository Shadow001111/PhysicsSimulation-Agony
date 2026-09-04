#include "BodySoA.h"

namespace PS_AGONY
{
	void BodySoA::append(
		Vec2 pos,
		Vec2 vel,
		Real rot,
		Real anglVel,
		Real mass,
		Real invMass,
		Real inertia,
		Real invInertia,
		Vec2 localCenterOfMass,
		MaterialIndex materialIndex,
		BodyType bodyType,
		ObjectIndex shapeIndex
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
		this->materialIndex.push_back(materialIndex);
		this->aabb.minX.push_back(0);
		this->aabb.minY.push_back(0);
		this->aabb.maxX.push_back(0);
		this->aabb.maxY.push_back(0);
		this->bodyType.push_back(bodyType);
		this->shapeIndex.push_back(shapeIndex);
		this->attachments.emplace_back();
	}

	#define PS_AGONY_SWAP_WITH_BACK(vector, index) std::swap((vector)[(index)], (vector).back())

	void BodySoA::swapWithBack(size_t index)
	{
		if (index >= offsetX.size()) [[unlikely]]
		{
			return;
		}

		PS_AGONY_SWAP_WITH_BACK(renderOldOffsetX, index);
		PS_AGONY_SWAP_WITH_BACK(renderOldOffsetY, index);
		PS_AGONY_SWAP_WITH_BACK(renderOldRotation, index);
		PS_AGONY_SWAP_WITH_BACK(renderRotationWrapCount, index);

		PS_AGONY_SWAP_WITH_BACK(offsetX, index);
		PS_AGONY_SWAP_WITH_BACK(offsetY, index);
		PS_AGONY_SWAP_WITH_BACK(localCenterOfMassX, index);
		PS_AGONY_SWAP_WITH_BACK(localCenterOfMassY, index);
		PS_AGONY_SWAP_WITH_BACK(worldCenterX, index);
		PS_AGONY_SWAP_WITH_BACK(worldCenterY, index);
		PS_AGONY_SWAP_WITH_BACK(velocityX, index);
		PS_AGONY_SWAP_WITH_BACK(velocityY, index);
		PS_AGONY_SWAP_WITH_BACK(rotation, index);
		PS_AGONY_SWAP_WITH_BACK(angularVelocity, index);
		PS_AGONY_SWAP_WITH_BACK(mass, index);
		PS_AGONY_SWAP_WITH_BACK(invMass, index);
		PS_AGONY_SWAP_WITH_BACK(inertia, index);
		PS_AGONY_SWAP_WITH_BACK(invInertia, index);
		PS_AGONY_SWAP_WITH_BACK(rotationCos, index);
		PS_AGONY_SWAP_WITH_BACK(rotationSin, index);
		PS_AGONY_SWAP_WITH_BACK(isStatic, index);
		PS_AGONY_SWAP_WITH_BACK(materialIndex, index);
		PS_AGONY_SWAP_WITH_BACK(aabb.minX, index);
		PS_AGONY_SWAP_WITH_BACK(aabb.minY, index);
		PS_AGONY_SWAP_WITH_BACK(aabb.maxX, index);
		PS_AGONY_SWAP_WITH_BACK(aabb.maxY, index);
		PS_AGONY_SWAP_WITH_BACK(bodyType, index);
		PS_AGONY_SWAP_WITH_BACK(shapeIndex, index);
		PS_AGONY_SWAP_WITH_BACK(attachments, index);
	}

	void BodySoA::popBack()
	{
		if (offsetX.empty()) [[unlikely]]
		{
			return;
		}

		renderOldOffsetX.pop_back();
		renderOldOffsetY.pop_back();
		renderOldRotation.pop_back();
		renderRotationWrapCount.pop_back();

		offsetX.pop_back();
		offsetY.pop_back();
		localCenterOfMassX.pop_back();
		localCenterOfMassY.pop_back();
		worldCenterX.pop_back();
		worldCenterY.pop_back();
		velocityX.pop_back();
		velocityY.pop_back();
		rotation.pop_back();
		angularVelocity.pop_back();
		mass.pop_back();
		invMass.pop_back();
		inertia.pop_back();
		invInertia.pop_back();
		rotationCos.pop_back();
		rotationSin.pop_back();
		isStatic.pop_back();
		materialIndex.pop_back();
		aabb.minX.pop_back();
		aabb.minY.pop_back();
		aabb.maxX.pop_back();
		aabb.maxY.pop_back();
		bodyType.pop_back();
		shapeIndex.pop_back();
		attachments.pop_back();
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

	size_t BodySoA::getMemoryUsage() const noexcept
	{
		size_t total =
			getVectorMemoryUsage(renderOldOffsetX) +
			getVectorMemoryUsage(renderOldOffsetY) +
			getVectorMemoryUsage(renderOldRotation) +
			getVectorMemoryUsage(renderRotationWrapCount) +

			getVectorMemoryUsage(offsetX) +
			getVectorMemoryUsage(offsetY) +
			getVectorMemoryUsage(localCenterOfMassX) +
			getVectorMemoryUsage(localCenterOfMassY) +
			getVectorMemoryUsage(worldCenterX) +
			getVectorMemoryUsage(worldCenterY) +
			getVectorMemoryUsage(velocityX) +
			getVectorMemoryUsage(velocityY) +
			getVectorMemoryUsage(rotation) +
			getVectorMemoryUsage(angularVelocity) +
			getVectorMemoryUsage(mass) +
			getVectorMemoryUsage(invMass) +
			getVectorMemoryUsage(inertia) +
			getVectorMemoryUsage(invInertia) +
			getVectorMemoryUsage(rotationCos) +
			getVectorMemoryUsage(rotationSin) +
			getVectorMemoryUsage(isStatic) +
			getVectorMemoryUsage(materialIndex) +
			aabb.getMemoryUsage() +
			getVectorMemoryUsage(bodyType) +
			getVectorMemoryUsage(shapeIndex);

		total += getVectorMemoryUsage(attachments);
		for (const auto& bodyAttachments : attachments)
		{
			total += getVectorMemoryUsage(bodyAttachments);
		}

		return total;
	}
}