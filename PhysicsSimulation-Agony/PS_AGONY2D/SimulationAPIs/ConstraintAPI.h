#pragma once
#include "PS_AGONY2D/ConstraintSystem.h"

#include "PS_AGONY2D/SimulationImpl/ObjectManager.h"

#include <cstdint>

namespace PS_AGONY2D
{
	// Non-owning facade over the constraint systems.
	class ConstraintAPI
	{
	private:
		ObjectManager& objectManager;
		IConstraintSystem** systemsArray = nullptr;
	public:
		ConstraintAPI(
			ObjectManager& objectManager,
			IConstraintSystem** systemsArray
		) noexcept :
			objectManager(objectManager),
			systemsArray(systemsArray)
		{}

		ConstraintAPI(const ConstraintAPI&) = delete;
		ConstraintAPI& operator=(const ConstraintAPI&) = delete;

		void createSpring(const SpringConstraintSystem::CreateParams& params) { springs().create(params, objectManager.bodies); }
		void destroySpring(uint32_t index) { springs().removeConstraint(index, objectManager.bodies); }

		void createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& params) { revoluteJoints().create(params, objectManager.bodies); }
		void destroyRevoluteJoint(uint32_t index) { revoluteJoints().removeConstraint(index, objectManager.bodies); }

		void createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& params) { prismaticJoints().create(params, objectManager.bodies); }
		void destroyPrismaticJoint(uint32_t index) { prismaticJoints().removeConstraint(index, objectManager.bodies); }

		void createSliderJoint(const SliderJointConstraintSystem::CreateParams& params) { sliderJoints().create(params, objectManager.bodies); }
		void destroySliderJoint(uint32_t index) { sliderJoints().removeConstraint(index, objectManager.bodies); }

		void createMotor(const MotorConstraintSystem::CreateParams& params) { motors().create(params, objectManager.bodies); }
		void destroyMotor(uint32_t index) { motors().removeConstraint(index, objectManager.bodies); }

		SpringSoAViewer getSprings() const noexcept { return springs().getViewer(); }
		RevoluteJointSoAViewer getJoints() const noexcept { return revoluteJoints().getViewer(); }
		PrismaticJointSoAViewer getPrismaticJoints() const noexcept { return prismaticJoints().getViewer(); }
		SliderJointSoAViewer getSliderJoints() const noexcept { return sliderJoints().getViewer(); }
		MotorSoAViewer getMotors() const noexcept { return motors().getViewer(); }
	private:
		SpringConstraintSystem& springs() noexcept {
			return static_cast<SpringConstraintSystem&>(*systemsArray[size_t(ConstraintType::Spring)]);
		}
		const SpringConstraintSystem& springs() const noexcept {
			return static_cast<const SpringConstraintSystem&>(*systemsArray[size_t(ConstraintType::Spring)]);
		}

		RevoluteJointConstraintSystem& revoluteJoints() noexcept {
			return static_cast<RevoluteJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::RevoluteJoint)]);
		}
		const RevoluteJointConstraintSystem& revoluteJoints() const noexcept {
			return static_cast<const RevoluteJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::RevoluteJoint)]);
		}

		PrismaticJointConstraintSystem& prismaticJoints() noexcept {
			return static_cast<PrismaticJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::PrismaticJoint)]);
		}
		const PrismaticJointConstraintSystem& prismaticJoints() const noexcept {
			return static_cast<const PrismaticJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::PrismaticJoint)]);
		}

		SliderJointConstraintSystem& sliderJoints() noexcept {
			return static_cast<SliderJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::SliderJoint)]);
		}
		const SliderJointConstraintSystem& sliderJoints() const noexcept {
			return static_cast<const SliderJointConstraintSystem&>(*systemsArray[size_t(ConstraintType::SliderJoint)]);
		}

		MotorConstraintSystem& motors() noexcept {
			return static_cast<MotorConstraintSystem&>(*systemsArray[size_t(ConstraintType::Motor)]);
		}
		const MotorConstraintSystem& motors() const noexcept {
			return static_cast<const MotorConstraintSystem&>(*systemsArray[size_t(ConstraintType::Motor)]);
		}
	};
}
