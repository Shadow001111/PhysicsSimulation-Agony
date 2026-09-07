#pragma once
#include "SoA/BodySoA.h"
#include "SoA/Constraints/SpringSoA.h"

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
	};
}