#pragma once
#include "ConstraintSystem.h"

#include "SimulationImpl/ObjectManager.h"

#include <cstdint>

namespace PS_AGONY2D
{
	// Non-owning facade over the constraint systems.
	class ConstraintAPI
	{
	private:
		ObjectManager& objectManager;
		SpringConstraintSystem& springs;
		RevoluteJointConstraintSystem& revoluteJoints;
		RodConstraintSystem& rods;
		PrismaticJointConstraintSystem& prismaticJoints;
		SliderJointConstraintSystem& sliderJoints;
		MotorConstraintSystem& motors;
	public:
		ConstraintAPI(
			ObjectManager& objectManager,
			SpringConstraintSystem& springs,
			RevoluteJointConstraintSystem& revoluteJoints,
			RodConstraintSystem& rods,
			PrismaticJointConstraintSystem& prismaticJoints,
			SliderJointConstraintSystem& sliderJoints,
			MotorConstraintSystem& motors) noexcept
			: objectManager(objectManager), springs(springs), revoluteJoints(revoluteJoints), rods(rods),
			prismaticJoints(prismaticJoints), sliderJoints(sliderJoints), motors(motors) {}

		ConstraintAPI(const ConstraintAPI&) = delete;
		ConstraintAPI& operator=(const ConstraintAPI&) = delete;

		void createSpring(const SpringConstraintSystem::CreateParams& params) { springs.create(params, objectManager.bodies); }
		void destroySpring(uint32_t index) { springs.removeConstraint(index, objectManager.bodies); }

		void createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& params) { revoluteJoints.create(params, objectManager.bodies); }
		void destroyRevoluteJoint(uint32_t index) { revoluteJoints.removeConstraint(index, objectManager.bodies); }

		void createRod(const RodConstraintSystem::CreateParams& params) { rods.create(params, objectManager.bodies); }
		void destroyRod(uint32_t index) { rods.removeConstraint(index, objectManager.bodies); }

		void createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& params) { prismaticJoints.create(params, objectManager.bodies); }
		void destroyPrismaticJoint(uint32_t index) { prismaticJoints.removeConstraint(index, objectManager.bodies); }

		void createSliderJoint(const SliderJointConstraintSystem::CreateParams& params) { sliderJoints.create(params, objectManager.bodies); }
		void destroySliderJoint(uint32_t index) { sliderJoints.removeConstraint(index, objectManager.bodies); }

		void createMotor(const MotorConstraintSystem::CreateParams& params) { motors.create(params, objectManager.bodies); }
		void destroyMotor(uint32_t index) { motors.removeConstraint(index, objectManager.bodies); }

		SpringSoAViewer getSprings() const noexcept { return springs.getViewer(); }
		RevoluteJointSoAViewer getJoints() const noexcept { return revoluteJoints.getViewer(); }
		RodSoAViewer getRods() const noexcept { return rods.getViewer(); }
		PrismaticJointSoAViewer getPrismaticJoints() const noexcept { return prismaticJoints.getViewer(); }
		SliderJointSoAViewer getSliderJoints() const noexcept { return sliderJoints.getViewer(); }
		MotorSoAViewer getMotors() const noexcept { return motors.getViewer(); }
	};
}
