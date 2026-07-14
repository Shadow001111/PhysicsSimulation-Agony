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
		BodyIndex shapeIndex
	)
	{
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
	}

	void BodySoA::swapWithBack(size_t index)
	{
		if (index >= offsetX.size()) [[unlikely]]
		{
			return;
		}

		std::swap(offsetX[index], offsetX.back());
		std::swap(offsetY[index], offsetY.back());
		std::swap(localCenterOfMassX[index], localCenterOfMassX.back());
		std::swap(localCenterOfMassY[index], localCenterOfMassY.back());
		std::swap(worldCenterX[index], worldCenterX.back());
		std::swap(worldCenterY[index], worldCenterY.back());
		std::swap(velocityX[index], velocityX.back());
		std::swap(velocityY[index], velocityY.back());
		std::swap(rotation[index], rotation.back());
		std::swap(angularVelocity[index], angularVelocity.back());
		std::swap(mass[index], mass.back());
		std::swap(invMass[index], invMass.back());
		std::swap(inertia[index], inertia.back());
		std::swap(invInertia[index], invInertia.back());
		std::swap(rotationCos[index], rotationCos.back());
		std::swap(rotationSin[index], rotationSin.back());
		std::swap(isStatic[index], isStatic.back());
		std::swap(materialIndex[index], materialIndex.back());
		std::swap(aabb.minX[index], aabb.minX.back());
		std::swap(aabb.minY[index], aabb.minY.back());
		std::swap(aabb.maxX[index], aabb.maxX.back());
		std::swap(aabb.maxY[index], aabb.maxY.back());
		std::swap(bodyType[index], bodyType.back());
		std::swap(shapeIndex[index], shapeIndex.back());
	}

	void BodySoA::popBack()
	{
		if (offsetX.empty()) [[unlikely]]
		{
			return;
		}

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
	}

	size_t BodySoA::getMemoryUsage() const noexcept
	{
		return
			PS_AGONY::getVectorMemoryUsage(offsetX) +
			PS_AGONY::getVectorMemoryUsage(offsetY) +
			PS_AGONY::getVectorMemoryUsage(localCenterOfMassX) +
			PS_AGONY::getVectorMemoryUsage(localCenterOfMassY) +
			PS_AGONY::getVectorMemoryUsage(worldCenterX) +
			PS_AGONY::getVectorMemoryUsage(worldCenterY) +
			PS_AGONY::getVectorMemoryUsage(velocityX) +
			PS_AGONY::getVectorMemoryUsage(velocityY) +
			PS_AGONY::getVectorMemoryUsage(rotation) +
			PS_AGONY::getVectorMemoryUsage(angularVelocity) +
			PS_AGONY::getVectorMemoryUsage(mass) +
			PS_AGONY::getVectorMemoryUsage(invMass) +
			PS_AGONY::getVectorMemoryUsage(inertia) +
			PS_AGONY::getVectorMemoryUsage(invInertia) +
			PS_AGONY::getVectorMemoryUsage(rotationCos) +
			PS_AGONY::getVectorMemoryUsage(rotationSin) +
			PS_AGONY::getVectorMemoryUsage(isStatic) +
			PS_AGONY::getVectorMemoryUsage(materialIndex) +
			aabb.getMemoryUsage() +
			PS_AGONY::getVectorMemoryUsage(bodyType) +
			PS_AGONY::getVectorMemoryUsage(shapeIndex);
	}
}