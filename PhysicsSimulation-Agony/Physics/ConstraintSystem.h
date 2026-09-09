#pragma once
#include "SoA/BodySoA.h"
#include "SoA/Constraints/SpringSoA.h"
#include "SoA/Constraints/JointSoA.h"
#include "SoA/Constraints/RodSoA.h"

#include <array>

namespace PS_AGONY
{
	// One implementation per ConstraintType, letting a body cascade-delete
	// whatever is attached to it without BodySoA needing to know concrete
	// constraint types. Each system is also where its constraint SoA gets
	// wired up to BodySoA - the SoA itself (e.g. SpringSoA) stays ignorant
	// of bodies entirely.
	class IConstraintSystem
	{
	public:
		virtual ~IConstraintSystem() = default;

		virtual void removeConstraint(uint32_t index, BodySoA& bodies) = 0;
		virtual void remapBodyIndex(uint32_t index, ObjectIndex oldBodyIndex, ObjectIndex newBodyIndex) = 0;

		virtual const char* getName() const noexcept = 0;
	};

	using ConstraintSystemArray = std::array<IConstraintSystem*, static_cast<size_t>(ConstraintType::COUNT)>;

	class SpringConstraintSystem final : public IConstraintSystem
	{
		SpringSoA& springs;
	public:
		explicit SpringConstraintSystem(SpringSoA& springs) noexcept : springs(springs) {}

		// Creates a spring and registers it with both bodies it touches.
		void createSpring(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real restLen, Real stiffness, Real damping, BodySoA& bodies)
		{
			const uint32_t newIndex = springs.append(indexA, indexB, anchorA, anchorB, restLen, stiffness, damping);
			bodies.addAttachment(indexA, ConstraintType::Spring, newIndex);
			bodies.addAttachment(indexB, ConstraintType::Spring, newIndex);
		}

		void removeConstraint(uint32_t index, BodySoA& bodies) override
		{
			if (index >= springs.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}

			// Unlink the spring being removed from both of its bodies.
			bodies.removeAttachment(springs.bodyIndexA[index], ConstraintType::Spring, index);
			bodies.removeAttachment(springs.bodyIndexB[index], ConstraintType::Spring, index);

			const size_t oldBackIndex = springs.swapRemove(index);

			// If a spring got swapped into 'index', relink it there.
			if (oldBackIndex != index)
			{
				const uint32_t newIndex = static_cast<uint32_t>(index);
				const uint32_t oldIndex = static_cast<uint32_t>(oldBackIndex);

				bodies.removeAttachment(springs.bodyIndexA[index], ConstraintType::Spring, oldIndex);
				bodies.addAttachment(springs.bodyIndexA[index], ConstraintType::Spring, newIndex);
				bodies.removeAttachment(springs.bodyIndexB[index], ConstraintType::Spring, oldIndex);
				bodies.addAttachment(springs.bodyIndexB[index], ConstraintType::Spring, newIndex);
			}
		}

		void remapBodyIndex(uint32_t index, ObjectIndex oldBodyIndex, ObjectIndex newBodyIndex) override
		{
			if (index >= springs.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}
			if (springs.bodyIndexA[index] == oldBodyIndex) springs.bodyIndexA[index] = newBodyIndex;
			if (springs.bodyIndexB[index] == oldBodyIndex) springs.bodyIndexB[index] = newBodyIndex;
		}

		const char* getName() const noexcept override { return "Spring"; }
	};

	class JointConstraintSystem final : public IConstraintSystem
	{
		JointSoA& joints;
	public:
		explicit JointConstraintSystem(JointSoA& joints) noexcept : joints(joints) {}

		// Creates a revolute joint and registers it with both bodies it touches.
		void createJoint(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real stiffness, Real damping, BodySoA& bodies)
		{
			const uint32_t newIndex = joints.append(indexA, indexB, anchorA, anchorB, stiffness, damping);
			bodies.addAttachment(indexA, ConstraintType::Joint, newIndex);
			bodies.addAttachment(indexB, ConstraintType::Joint, newIndex);
		}

		void removeConstraint(uint32_t index, BodySoA& bodies) override
		{
			if (index >= joints.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}

			// Unlink the joint being removed from both of its bodies.
			bodies.removeAttachment(joints.bodyIndexA[index], ConstraintType::Joint, index);
			bodies.removeAttachment(joints.bodyIndexB[index], ConstraintType::Joint, index);

			const size_t oldBackIndex = joints.swapRemove(index);

			// If a joint got swapped into 'index', relink it there.
			if (oldBackIndex != index)
			{
				const uint32_t newIndex = static_cast<uint32_t>(index);
				const uint32_t oldIndex = static_cast<uint32_t>(oldBackIndex);

				bodies.removeAttachment(joints.bodyIndexA[index], ConstraintType::Joint, oldIndex);
				bodies.addAttachment(joints.bodyIndexA[index], ConstraintType::Joint, newIndex);
				bodies.removeAttachment(joints.bodyIndexB[index], ConstraintType::Joint, oldIndex);
				bodies.addAttachment(joints.bodyIndexB[index], ConstraintType::Joint, newIndex);
			}
		}

		void remapBodyIndex(uint32_t index, ObjectIndex oldBodyIndex, ObjectIndex newBodyIndex) override
		{
			if (index >= joints.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}
			if (joints.bodyIndexA[index] == oldBodyIndex) joints.bodyIndexA[index] = newBodyIndex;
			if (joints.bodyIndexB[index] == oldBodyIndex) joints.bodyIndexB[index] = newBodyIndex;
		}

		const char* getName() const noexcept override { return "Joint"; }
	};

	class RodConstraintSystem final : public IConstraintSystem
	{
		RodSoA& rods;
	public:
		explicit RodConstraintSystem(RodSoA& rods) noexcept : rods(rods) {}

		// Creates a rod and registers it with both bodies it touches.
		void createRod(ObjectIndex indexA, ObjectIndex indexB, Vec2 anchorA, Vec2 anchorB, Real length, BodySoA& bodies)
		{
			const uint32_t newIndex = rods.append(indexA, indexB, anchorA, anchorB, length);
			bodies.addAttachment(indexA, ConstraintType::Rod, newIndex);
			bodies.addAttachment(indexB, ConstraintType::Rod, newIndex);
		}

		void removeConstraint(uint32_t index, BodySoA& bodies) override
		{
			if (index >= rods.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}

			// Unlink the rod being removed from both of its bodies.
			bodies.removeAttachment(rods.bodyIndexA[index], ConstraintType::Rod, index);
			bodies.removeAttachment(rods.bodyIndexB[index], ConstraintType::Rod, index);

			const size_t oldBackIndex = rods.swapRemove(index);

			// If a rod got swapped into 'index', relink it there.
			if (oldBackIndex != index)
			{
				const uint32_t newIndex = static_cast<uint32_t>(index);
				const uint32_t oldIndex = static_cast<uint32_t>(oldBackIndex);

				bodies.removeAttachment(rods.bodyIndexA[index], ConstraintType::Rod, oldIndex);
				bodies.addAttachment(rods.bodyIndexA[index], ConstraintType::Rod, newIndex);
				bodies.removeAttachment(rods.bodyIndexB[index], ConstraintType::Rod, oldIndex);
				bodies.addAttachment(rods.bodyIndexB[index], ConstraintType::Rod, newIndex);
			}
		}

		void remapBodyIndex(uint32_t index, ObjectIndex oldBodyIndex, ObjectIndex newBodyIndex) override
		{
			if (index >= rods.bodyIndexA.size()) [[unlikely]]
			{
				return;
			}
			if (rods.bodyIndexA[index] == oldBodyIndex) rods.bodyIndexA[index] = newBodyIndex;
			if (rods.bodyIndexB[index] == oldBodyIndex) rods.bodyIndexB[index] = newBodyIndex;
		}

		const char* getName() const noexcept override { return "Rod"; }
	};
}