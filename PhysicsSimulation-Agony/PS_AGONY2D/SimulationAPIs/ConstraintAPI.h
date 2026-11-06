#pragma once
#include "PS_AGONY2D/SimulationImpl/AllConstraintSystem.h"
#include "PS_AGONY2D/SimulationImpl/ObjectManager.h"

#include <cstdint>
#include <optional>

namespace PS_AGONY2D
{
	// Non-owning facade over the constraint systems.
	// Handles are persistent (valid across swap-and-pop, stale after destroy); dense indices are not.
	// Modifications are deferred: queued and applied by IConstraintSystem::flushCommands(). Stale handles are dropped.
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

		// Creation (body handles in params). nullopt if a body handle is stale.
		std::optional<Handle> createSpring(const SpringConstraintSystem::CreateParams& p) { return objectManager.createSpring(p); }
		std::optional<Handle> createRevoluteJoint(const RevoluteJointConstraintSystem::CreateParams& p) { return objectManager.createRevoluteJoint(p); }
		std::optional<Handle> createPrismaticJoint(const PrismaticJointConstraintSystem::CreateParams& p) { return objectManager.createPrismaticJoint(p); }
		std::optional<Handle> createSliderJoint(const SliderJointConstraintSystem::CreateParams& p) { return objectManager.createSliderJoint(p); }
		std::optional<Handle> createMotor(const MotorConstraintSystem::CreateParams& p) { return objectManager.createMotor(p); }

		// Destruction.
		void destroySpring(ObjectDenseIndex index) { objectManager.destroyConstraintImmediate(ConstraintType::Spring, index); }
		void destroyRevoluteJoint(ObjectDenseIndex index) { objectManager.destroyConstraintImmediate(ConstraintType::RevoluteJoint, index); }
		void destroyPrismaticJoint(ObjectDenseIndex index) { objectManager.destroyConstraintImmediate(ConstraintType::PrismaticJoint, index); }
		void destroySliderJoint(ObjectDenseIndex index) { objectManager.destroyConstraintImmediate(ConstraintType::SliderJoint, index); }
		void destroyMotor(ObjectDenseIndex index) { objectManager.destroyConstraintImmediate(ConstraintType::Motor, index); }

		void destroySpring(Handle h) { objectManager.destroySpringImmediate(h); }
		void destroyRevoluteJoint(Handle h) { objectManager.destroyRevoluteJointImmediate(h); }
		void destroyPrismaticJoint(Handle h) { objectManager.destroyPrismaticJointImmediate(h); }
		void destroySliderJoint(Handle h) { objectManager.destroySliderJointImmediate(h); }
		void destroyMotor(Handle h) { objectManager.destroyMotorImmediate(h); }

		// Handles.
		Handle getSpringHandle(ObjectDenseIndex index) const noexcept { return handleOf(springs(), index); }
		Handle getRevoluteJointHandle(ObjectDenseIndex index) const noexcept { return handleOf(revoluteJoints(), index); }
		Handle getPrismaticJointHandle(ObjectDenseIndex index) const noexcept { return handleOf(prismaticJoints(), index); }
		Handle getSliderJointHandle(ObjectDenseIndex index) const noexcept { return handleOf(sliderJoints(), index); }
		Handle getMotorHandle(ObjectDenseIndex index) const noexcept { return handleOf(motors(), index); }

		bool isSpringValid(Handle h) const noexcept { return springs().isValid(h); }
		bool isRevoluteJointValid(Handle h) const noexcept { return revoluteJoints().isValid(h); }
		bool isPrismaticJointValid(Handle h) const noexcept { return prismaticJoints().isValid(h); }
		bool isSliderJointValid(Handle h) const noexcept { return sliderJoints().isValid(h); }
		bool isMotorValid(Handle h) const noexcept { return motors().isValid(h); }

		ObjectDenseIndex getSpringIndex(Handle h) const noexcept { return springs().indexOf(h); }
		ObjectDenseIndex getRevoluteJointIndex(Handle h) const noexcept { return revoluteJoints().indexOf(h); }
		ObjectDenseIndex getPrismaticJointIndex(Handle h) const noexcept { return prismaticJoints().indexOf(h); }
		ObjectDenseIndex getSliderJointIndex(Handle h) const noexcept { return sliderJoints().indexOf(h); }
		ObjectDenseIndex getMotorIndex(Handle h) const noexcept { return motors().indexOf(h); }

		// Deferred modifications:
		// Spring:
		void setSpringLocalAnchorA(Handle h, Vec2 v) { springs().queueLocalAnchorA(h, v); }
		void setSpringLocalAnchorB(Handle h, Vec2 v) { springs().queueLocalAnchorB(h, v); }
		void setSpringRestLength(Handle h, Real v) { springs().queueRestLength(h, v); }
		void setSpringStiffness(Handle h, Real v) { springs().queueStiffness(h, v); }
		void setSpringDamping(Handle h, Real v) { springs().queueDamping(h, v); }

		// Revolute joint:
		void setRevoluteJointLocalAnchorA(Handle h, Vec2 v) { revoluteJoints().queueLocalAnchorA(h, v); }
		void setRevoluteJointLocalAnchorB(Handle h, Vec2 v) { revoluteJoints().queueLocalAnchorB(h, v); }
		void setRevoluteJointStiffness(Handle h, Real v) { revoluteJoints().queueStiffness(h, v); }
		void setRevoluteJointDamping(Handle h, Real v) { revoluteJoints().queueDamping(h, v); }

		// Prismatic joint:
		void setPrismaticJointLocalAnchorA(Handle h, Vec2 v) { prismaticJoints().queueLocalAnchorA(h, v); }
		void setPrismaticJointLocalAnchorB(Handle h, Vec2 v) { prismaticJoints().queueLocalAnchorB(h, v); }
		// Re-normalized on apply; a zero-length axis is dropped.
		void setPrismaticJointLocalAxisA(Handle h, Vec2 axis) { prismaticJoints().queueLocalAxisA(h, axis); }
		void setPrismaticJointStiffness(Handle h, Real v) { prismaticJoints().queueStiffness(h, v); }
		void setPrismaticJointDamping(Handle h, Real v) { prismaticJoints().queueDamping(h, v); }
		// Order-insensitive.
		void setPrismaticJointLimits(Handle h, Real lower, Real upper) { prismaticJoints().queueLimits(h, lower, upper); }
		void setPrismaticJointEnableLimit(Handle h, bool enable) { prismaticJoints().queueEnableLimit(h, enable); }
		// Locks rotation at the bodies' current relative angle.
		void resetPrismaticJointReferenceAngle(Handle h) { prismaticJoints().queueResetReferenceAngle(h); }

		// Slider joint:
		void setSliderJointLocalAnchorA(Handle h, Vec2 v) { sliderJoints().queueLocalAnchorA(h, v); }
		void setSliderJointLocalAnchorB(Handle h, Vec2 v) { sliderJoints().queueLocalAnchorB(h, v); }
		// Re-normalized on apply; a zero-length axis is dropped.
		void setSliderJointLocalAxisA(Handle h, Vec2 axis) { sliderJoints().queueLocalAxisA(h, axis); }
		void setSliderJointStiffness(Handle h, Real v) { sliderJoints().queueStiffness(h, v); }
		void setSliderJointDamping(Handle h, Real v) { sliderJoints().queueDamping(h, v); }
		// Order-insensitive.
		void setSliderJointLimits(Handle h, Real lower, Real upper) { sliderJoints().queueLimits(h, lower, upper); }
		void setSliderJointEnableLimit(Handle h, bool enable) { sliderJoints().queueEnableLimit(h, enable); }

		// Motor:
		void setMotorTargetSpeed(Handle h, Real v) { motors().queueTargetSpeed(h, v); }
		void setMotorMaxTorque(Handle h, Real v) { motors().queueMaxTorque(h, v); }

		// Instant reads of stored state (not the queue); nullopt if stale.
		std::optional<SpringConstraintSystem::Data> getSpringDataH(Handle h) const noexcept
			{ return springs().getData(h, objectManager.bodies); }
		std::optional<RevoluteJointConstraintSystem::Data> getRevoluteJointDataH(Handle h) const noexcept
			{ return revoluteJoints().getData(h, objectManager.bodies); }
		std::optional<PrismaticJointConstraintSystem::Data> getPrismaticJointDataH(Handle h) const noexcept
			{ return prismaticJoints().getData(h, objectManager.bodies); }
		std::optional<SliderJointConstraintSystem::Data> getSliderJointDataH(Handle h) const noexcept
			{ return sliderJoints().getData(h, objectManager.bodies); }
		std::optional<MotorConstraintSystem::Data> getMotorDataH(Handle h) const noexcept
			{ return motors().getData(h, objectManager.bodies); }

		// Dense-index variants; nullopt if out of range.
		std::optional<SpringConstraintSystem::Data> getSpringData(ObjectDenseIndex i) const noexcept
			{ return springs().getDataAt(i, objectManager.bodies); }
		std::optional<RevoluteJointConstraintSystem::Data> getRevoluteJointData(ObjectDenseIndex i) const noexcept
			{ return revoluteJoints().getDataAt(i, objectManager.bodies); }
		std::optional<PrismaticJointConstraintSystem::Data> getPrismaticJointData(ObjectDenseIndex i) const noexcept
			{ return prismaticJoints().getDataAt(i, objectManager.bodies); }
		std::optional<SliderJointConstraintSystem::Data> getSliderJointData(ObjectDenseIndex i) const noexcept
			{ return sliderJoints().getDataAt(i, objectManager.bodies); }
		std::optional<MotorConstraintSystem::Data> getMotorData(ObjectDenseIndex i) const noexcept
			{ return motors().getDataAt(i, objectManager.bodies); }

		// Viewers.
		SpringSoAViewer getSprings() const noexcept
			{ return springs().getViewer(); }
		RevoluteJointSoAViewer getRevoluteJoints() const noexcept
			{ return revoluteJoints().getViewer(); }
		PrismaticJointSoAViewer getPrismaticJoints() const noexcept
			{ return prismaticJoints().getViewer(); }
		SliderJointSoAViewer getSliderJoints() const noexcept
			{ return sliderJoints().getViewer(); }
		MotorSoAViewer getMotors() const noexcept
			{ return motors().getViewer(); }
	private:
		template<class System>
		static Handle handleOf(const System& system, ObjectDenseIndex index) noexcept
		{
			return index < system.getCount() ? system.handleAt(index) : Handle{};
		}

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