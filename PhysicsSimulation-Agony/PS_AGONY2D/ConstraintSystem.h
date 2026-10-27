#pragma once
#include "SoA/BodySoA.h"
#include "SoA/Constraints/SpringSoA.h"
#include "SoA/Constraints/RevoluteJointSoA.h"
#include "SoA/Constraints/RodSoA.h"
#include "SoA/Constraints/PrismaticJointSoA.h"
#include "SoA/Constraints/SliderJointSoA.h"
#include "SoA/Constraints/MotorSoA.h"

#include "Solving/SpringSolver.h"
#include "Solving/RevoluteJointSolver.h"
#include "Solving/RodSolver.h"
#include "Solving/PrismaticJointSolver.h"
#include "Solving/SliderJointSolver.h"
#include "Solving/MotorSolver.h"

#include <array>
#include <cmath>

namespace PS_AGONY2D
{
	// One implementation per ConstraintType. Owns the constraint SoA and its solver, wires
	// them to BodySoA (attachments, cascade deletion, index remapping) and solves them.
	class IConstraintSystem
	{
	public:
		uint32_t iterations = 6;

		virtual ~IConstraintSystem() = default;

		virtual void removeConstraint(uint32_t index, BodySoA& bodies) = 0;
		virtual void remapBodyIndex(uint32_t index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) = 0;

		// Binds data views and (re)plans if this type changed. Solving is driven by ConstraintScheduler through getSolver().
		virtual void prepare(BodySoA& bodies) = 0;
		virtual IConstraintSolver& getSolver() noexcept = 0;

		virtual size_t getCount() const noexcept = 0;
		virtual size_t getMemoryUsage() const noexcept = 0;
		virtual size_t getSolverMemoryUsage() const noexcept = 0;

		virtual ConstraintType getType() const noexcept = 0;
		virtual const char* getName() const noexcept = 0;
	};

	using ConstraintSystemArray = std::array<IConstraintSystem*, static_cast<size_t>(ConstraintType::COUNT)>;

	template<ConstraintType Type, class SoA, class Solver, class Viewer>
	class ConstraintSystemBase : public IConstraintSystem
	{
	protected:
		SoA soa;
		Solver solver;
		bool wasSoAChanged = false;

		// Single-body constraints (motors) expose `bodyIndex` instead of `bodyIndexA/B`.
		static constexpr bool IsSingleBody = requires(SoA & s) { s.bodyIndex; };

		// Appends a single-body constraint and registers it with its body.
		template<class... Args>
		uint32_t createSingleImpl(ObjectDenseIndex index, BodySoA& bodies, Args&&... args)
		{
			const uint32_t newIndex = static_cast<uint32_t>(soa.append(index, std::forward<Args>(args)...));
			bodies.addAttachment(index, Type, newIndex);
			wasSoAChanged = true;
			return newIndex;
		}

		// Appends a constraint and registers it with both bodies it touches.
		template<class... Args>
		uint32_t createImpl(ObjectDenseIndex indexA, ObjectDenseIndex indexB, BodySoA& bodies, Args&&... args)
		{
			const ObjectDenseIndex newIndex = static_cast<uint32_t>(soa.append(indexA, indexB, std::forward<Args>(args)...));
			bodies.addAttachment(indexA, Type, newIndex);
			bodies.addAttachment(indexB, Type, newIndex);
			wasSoAChanged = true;
			return newIndex;
		}
	public:
		void removeConstraint(ObjectDenseIndex index, BodySoA& bodies) final
		{
			if (index >= soa.getCount()) [[unlikely]]
			{
				return;
			}

			if constexpr (IsSingleBody)
			{
				bodies.removeAttachment(soa.bodyIndex[index], Type, index);

				const ObjectDenseIndex oldBackIndex = soa.swapRemove(index);
				if (oldBackIndex != index)
				{
					bodies.removeAttachment(soa.bodyIndex[index], Type, oldBackIndex);
					bodies.addAttachment(soa.bodyIndex[index], Type, index);
				}

				wasSoAChanged = true;
				return;
			}
			else
			{
				// Unlink the constraint being removed from both of its bodies.
				bodies.removeAttachment(soa.bodyIndexA[index], Type, index);
				bodies.removeAttachment(soa.bodyIndexB[index], Type, index);

				const uint32_t oldBackIndex = static_cast<uint32_t>(soa.swapRemove(index));

				// If a constraint got swapped into 'index', relink it there.
				if (oldBackIndex != index)
				{
					bodies.removeAttachment(soa.bodyIndexA[index], Type, oldBackIndex);
					bodies.addAttachment(soa.bodyIndexA[index], Type, index);
					bodies.removeAttachment(soa.bodyIndexB[index], Type, oldBackIndex);
					bodies.addAttachment(soa.bodyIndexB[index], Type, index);
				}

				wasSoAChanged = true;
			}
		}

		void remapBodyIndex(uint32_t index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) final
		{
			if (index >= soa.getCount()) [[unlikely]]
			{
				return;
			}
			wasSoAChanged = true; // Body refs changed -> the plan is stale.
			if constexpr (IsSingleBody)
			{
				if (soa.bodyIndex[index] == oldBodyIndex) soa.bodyIndex[index] = newBodyIndex;
			}
			else
			{
				if (soa.bodyIndexA[index] == oldBodyIndex) soa.bodyIndexA[index] = newBodyIndex;
				if (soa.bodyIndexB[index] == oldBodyIndex) soa.bodyIndexB[index] = newBodyIndex;
			}
		}

		void prepare(BodySoA& bodies) final
		{
			solver.setDataViewers(bodies, Viewer(soa));
			solver.prepare(wasSoAChanged);
			wasSoAChanged = false;
		}

		IConstraintSolver& getSolver() noexcept final { return solver; }

		Viewer getViewer() const noexcept { return Viewer(soa); }

		size_t getCount() const noexcept final { return soa.getCount(); }
		size_t getMemoryUsage() const noexcept final { return soa.getMemoryUsage(); }
		size_t getSolverMemoryUsage() const noexcept final { return solver.getMemoryUsage(); }

		ConstraintType getType() const noexcept final { return Type; }
	};

	class SpringConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Spring, SpringSoA, SpringSolver, SpringSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndexA;
			ObjectDenseIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Real restLength;
			Real stiffness;
			Real damping;
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			createImpl(p.bodyIndexA, p.bodyIndexB, bodies, p.localAnchorA, p.localAnchorB, p.restLength, p.stiffness, p.damping);
		}

		const char* getName() const noexcept override { return "Spring"; }
	};

	class RevoluteJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Joint, RevoluteJointSoA, RevoluteJointSolver, RevoluteJointSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndexA;
			ObjectDenseIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Real stiffness;
			Real damping;
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			createImpl(p.bodyIndexA, p.bodyIndexB, bodies, p.localAnchorA, p.localAnchorB, p.stiffness, p.damping);
		}

		const char* getName() const noexcept override { return "RevoluteJoint"; }
	};

	class RodConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Rod, RodSoA, RodSolver, RodSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndexA;
			ObjectDenseIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Real length;
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			createImpl(p.bodyIndexA, p.bodyIndexB, bodies, p.localAnchorA, p.localAnchorB, p.length);
		}

		const char* getName() const noexcept override { return "Rod"; }
	};

	class PrismaticJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Prismatic, PrismaticJointSoA, PrismaticJointSolver, PrismaticJointSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndexA;
			ObjectDenseIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Vec2 localAxisA; // Axis in body A's local space that translation is locked to; need not be pre-normalized.
			Real stiffness;
			Real damping;
			Real lowerLimit = 0;
			Real upperLimit = 0;
			bool enableLimit = false;
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			// Normalize the axis defensively; a zero-length axis would make the joint singular.
			const Real axisLenSq = glm::dot(p.localAxisA, p.localAxisA);
			const Vec2 axis = axisLenSq > Real(1e-12) ? p.localAxisA / glm::sqrt(axisLenSq) : Vec2(Real(1), Real(0));

			// Capture the current relative angle so the joint locks rotation at its creation pose,
			// wrapped to (-pi, pi] so it stays robust to each body's rotation wrapping independently.
			constexpr Real PI = Real(3.14159265358979323846);
			constexpr Real TWO_PI = Real(2) * PI;
			Real refAngle = bodies.rotation[p.bodyIndexB] - bodies.rotation[p.bodyIndexA];
			refAngle -= TWO_PI * std::floor((refAngle + PI) / TWO_PI);

			createImpl(
				p.bodyIndexA, p.bodyIndexB, bodies,
				p.localAnchorA, p.localAnchorB, axis, refAngle,
				p.stiffness, p.damping, p.lowerLimit, p.upperLimit, p.enableLimit
			);
		}

		const char* getName() const noexcept override { return "PrismaticJoint"; }
	};

	class SliderJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Slider, SliderJointSoA, SliderJointSolver, SliderJointSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndexA;
			ObjectDenseIndex bodyIndexB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Vec2 localAxisA; // Axis in body A's local space; need not be pre-normalized.
			Real stiffness;
			Real damping;
			Real lowerLimit = 0;
			Real upperLimit = 0;
			bool enableLimit = false;
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			// Normalize the axis defensively; a zero-length axis would make the joint singular.
			const Real axisLenSq = glm::dot(p.localAxisA, p.localAxisA);
			const Vec2 axis = axisLenSq > Real(1e-12) ? p.localAxisA / glm::sqrt(axisLenSq) : Vec2(Real(1), Real(0));

			createImpl(
				p.bodyIndexA, p.bodyIndexB, bodies,
				p.localAnchorA, p.localAnchorB, axis,
				p.stiffness, p.damping, p.lowerLimit, p.upperLimit, p.enableLimit
			);
		}

		const char* getName() const noexcept override { return "SliderJoint"; }
	};

	class MotorConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Motor, MotorSoA, MotorSolver, MotorSoAViewer>
	{
	public:
		struct CreateParams
		{
			ObjectDenseIndex bodyIndex;
			Real targetSpeed;  // Target angular velocity.
			Real maxTorque;    // Torque limit.
		};

		void create(const CreateParams& p, BodySoA& bodies)
		{
			createSingleImpl(p.bodyIndex, bodies, p.targetSpeed, p.maxTorque);
		}

		const char* getName() const noexcept override { return "Motor"; }
	};
}