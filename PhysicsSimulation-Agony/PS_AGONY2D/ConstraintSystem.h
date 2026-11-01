#pragma once
#include "SoA/BodySoA.h"
#include "SoA/Constraints/SpringSoA.h"
#include "SoA/Constraints/RevoluteJointSoA.h"
#include "SoA/Constraints/PrismaticJointSoA.h"
#include "SoA/Constraints/SliderJointSoA.h"
#include "SoA/Constraints/MotorSoA.h"

#include "Solving/SpringSolver.h"
#include "Solving/RevoluteJointSolver.h"
#include "Solving/PrismaticJointSolver.h"
#include "Solving/SliderJointSolver.h"
#include "Solving/MotorSolver.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace PS_AGONY2D
{
	// Same layout as ObjectCommand. 'type' is the per-system Cmd enum.
	struct ConstraintCommand
	{
		uint8_t type{ 0 };
		Handle handle{};
		Real a{ 0 };
		Real b{ 0 };
		uint32_t index{ 0 };
	};

	// One implementation per ConstraintType. Owns the constraint SoA and its solver, wires
	// them to BodySoA (attachments, cascade deletion, index remapping) and solves them.
	class IConstraintSystem
	{
	public:
		uint32_t iterations = 6;

		virtual ~IConstraintSystem() = default;

		virtual void removeConstraint(uint32_t index, BodySoA& bodies) = 0;
		virtual void remapBodyIndex(uint32_t index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) = 0;

		// Applies and clears the queued modification commands. Call right after ObjectManager::flushCommands(),
		// never during solving. Stale handles are dropped; values are sanitized on apply. Never replans.
		virtual void flushCommands(const BodySoA& bodies) = 0;

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
		std::vector<ConstraintCommand> commands;

		// Single-body constraints (motors) expose `bodyIndex` instead of `bodyIndexA/B`.
		static constexpr bool IsSingleBody = requires(SoA & s) { s.bodyIndex; };

		template<class E>
		void pushCommand(E type, Handle handle, Real a = 0, Real b = 0, uint32_t index = 0)
		{
			ConstraintCommand command;
			command.type = static_cast<uint8_t>(type);
			command.handle = handle;
			command.a = a;
			command.b = b;
			command.index = index;
			commands.push_back(command);
		}

		// Resolves each handle and calls apply(command, denseIndex). Stale handles are dropped.
		// Value edits never touch wasSoAChanged.
		template<class F>
		void flushImpl(F&& apply)
		{
			if (commands.empty()) return;

			for (const ConstraintCommand& command : commands)
			{
				const uint32_t resolved = soa.indexOf(command.handle);
				if (resolved == Handle::kNull) continue;
				apply(command, resolved);
			}
			commands.clear();
		}

		// Normalizes in place; false if the axis is (near) zero-length.
		static bool normalizeAxis(Vec2& axis) noexcept
		{
			const Real lenSq = glm::dot(axis, axis);
			if (!(lenSq > Real(1e-12))) return false;
			axis /= glm::sqrt(lenSq);
			return true;
		}

		// Appends a single-body constraint and registers it with its body.
		template<class... Args>
		Handle createSingleImpl(ObjectDenseIndex index, BodySoA& bodies, Args&&... args)
		{
			const uint32_t newIndex = static_cast<uint32_t>(soa.append(index, std::forward<Args>(args)...));
			bodies.addAttachment(index, Type, newIndex);
			wasSoAChanged = true;
			return soa.handleAt(newIndex);
		}

		// Appends a constraint and registers it with both bodies it touches.
		template<class... Args>
		Handle createImpl(ObjectDenseIndex indexA, ObjectDenseIndex indexB, BodySoA& bodies, Args&&... args)
		{
			const ObjectDenseIndex newIndex = static_cast<uint32_t>(soa.append(indexA, indexB, std::forward<Args>(args)...));
			bodies.addAttachment(indexA, Type, newIndex);
			bodies.addAttachment(indexB, Type, newIndex);
			wasSoAChanged = true;
			return soa.handleAt(newIndex);
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

		// Instant snapshot. Body handles are Handle::kNull-safe copies of BodySoA handles.
		struct Data
		{
			Handle bodyHandleA{};
			Handle bodyHandleB{};
			Vec2 localAnchorA{ 0 };
			Vec2 localAnchorB{ 0 };
			Real restLength{ 0 };
			Real stiffness{ 0 };
			Real damping{ 0 };
		};

		Handle create(const CreateParams& p, BodySoA& bodies)
		{
			return createImpl(p.bodyIndexA, p.bodyIndexB, bodies, p.localAnchorA, p.localAnchorB, p.restLength, p.stiffness, p.damping);
		}

		// ---- Deferred modification commands (applied by flushCommands) ----
		void queueLocalAnchorA(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorA, h, v.x, v.y); }
		void queueLocalAnchorB(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorB, h, v.x, v.y); }
		void queueRestLength(Handle h, Real v) { pushCommand(Cmd::RestLength, h, v); }
		void queueStiffness(Handle h, Real v) { pushCommand(Cmd::Stiffness, h, v); }
		void queueDamping(Handle h, Real v) { pushCommand(Cmd::Damping, h, v); }

		void flushCommands(const BodySoA&) override
		{
			flushImpl([this](const ConstraintCommand& c, uint32_t i)
				{
					switch (static_cast<Cmd>(c.type))
					{
					case Cmd::LocalAnchorA: soa.localAnchorA[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAnchorB: soa.localAnchorB[i] = Vec2(c.a, c.b); break;
					case Cmd::RestLength: soa.restLength[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Stiffness: soa.stiffness[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Damping: soa.damping[i] = std::fmax(Real(0), c.a); break;
					}
				});
		}

		// ---- Instant reads (no queue). nullopt if stale / out of range. ----
		std::optional<Data> getData(Handle h, const BodySoA& bodies) const noexcept
		{
			const uint32_t i = soa.indexOf(h);
			if (i == Handle::kNull) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(uint32_t i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			d.bodyHandleA = bodies.handleAt(soa.bodyIndexA[i]);
			d.bodyHandleB = bodies.handleAt(soa.bodyIndexB[i]);
			d.localAnchorA = soa.localAnchorA[i];
			d.localAnchorB = soa.localAnchorB[i];
			d.restLength = soa.restLength[i];
			d.stiffness = soa.stiffness[i];
			d.damping = soa.damping[i];
			return d;
		}

		const char* getName() const noexcept override { return "Spring"; }

	private:
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, RestLength, Stiffness, Damping };
	};

	class RevoluteJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::RevoluteJoint, RevoluteJointSoA, RevoluteJointSolver, RevoluteJointSoAViewer>
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

		struct Data
		{
			Handle bodyHandleA{};
			Handle bodyHandleB{};
			Vec2 localAnchorA{ 0 };
			Vec2 localAnchorB{ 0 };
			Real stiffness{ 0 };
			Real damping{ 0 };
		};

		Handle create(const CreateParams& p, BodySoA& bodies)
		{
			return createImpl(p.bodyIndexA, p.bodyIndexB, bodies, p.localAnchorA, p.localAnchorB, p.stiffness, p.damping);
		}

		void queueLocalAnchorA(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorA, h, v.x, v.y); }
		void queueLocalAnchorB(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorB, h, v.x, v.y); }
		void queueStiffness(Handle h, Real v) { pushCommand(Cmd::Stiffness, h, v); }
		void queueDamping(Handle h, Real v) { pushCommand(Cmd::Damping, h, v); }

		void flushCommands(const BodySoA&) override
		{
			flushImpl([this](const ConstraintCommand& c, uint32_t i)
				{
					switch (static_cast<Cmd>(c.type))
					{
					case Cmd::LocalAnchorA: soa.localAnchorA[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAnchorB: soa.localAnchorB[i] = Vec2(c.a, c.b); break;
					case Cmd::Stiffness: soa.stiffness[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Damping: soa.damping[i] = std::fmax(Real(0), c.a); break;
					}
				});
		}

		std::optional<Data> getData(Handle h, const BodySoA& bodies) const noexcept
		{
			const uint32_t i = soa.indexOf(h);
			if (i == Handle::kNull) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(uint32_t i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			d.bodyHandleA = bodies.handleAt(soa.bodyIndexA[i]);
			d.bodyHandleB = bodies.handleAt(soa.bodyIndexB[i]);
			d.localAnchorA = soa.localAnchorA[i];
			d.localAnchorB = soa.localAnchorB[i];
			d.stiffness = soa.stiffness[i];
			d.damping = soa.damping[i];
			return d;
		}

		const char* getName() const noexcept override { return "RevoluteJoint"; }

	private:
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, Stiffness, Damping };
	};

	class PrismaticJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::PrismaticJoint, PrismaticJointSoA, PrismaticJointSolver, PrismaticJointSoAViewer>
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

		struct Data
		{
			Handle bodyHandleA{};
			Handle bodyHandleB{};
			Vec2 localAnchorA{ 0 };
			Vec2 localAnchorB{ 0 };
			Vec2 localAxisA{ 1, 0 };
			Real referenceAngle{ 0 };
			Real stiffness{ 0 };
			Real damping{ 0 };
			Real lowerLimit{ 0 };
			Real upperLimit{ 0 };
			bool enableLimit{ false };
		};

		Handle create(const CreateParams& p, BodySoA& bodies)
		{
			// Normalize the axis defensively; a zero-length axis would make the joint singular.
			const Real axisLenSq = glm::dot(p.localAxisA, p.localAxisA);
			const Vec2 axis = axisLenSq > Real(1e-12) ? p.localAxisA / glm::sqrt(axisLenSq) : Vec2(Real(1), Real(0));

			return createImpl(
				p.bodyIndexA, p.bodyIndexB, bodies,
				p.localAnchorA, p.localAnchorB, axis, computeReferenceAngle(bodies, p.bodyIndexA, p.bodyIndexB),
				p.stiffness, p.damping, p.lowerLimit, p.upperLimit, p.enableLimit
			);
		}

		void queueLocalAnchorA(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorA, h, v.x, v.y); }
		void queueLocalAnchorB(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorB, h, v.x, v.y); }
		// Re-normalized on apply; a zero-length axis is dropped.
		void queueLocalAxisA(Handle h, Vec2 axis) { pushCommand(Cmd::LocalAxisA, h, axis.x, axis.y); }
		void queueStiffness(Handle h, Real v) { pushCommand(Cmd::Stiffness, h, v); }
		void queueDamping(Handle h, Real v) { pushCommand(Cmd::Damping, h, v); }
		// Order-insensitive: stored as [fmin, fmax].
		void queueLimits(Handle h, Real lower, Real upper) { pushCommand(Cmd::Limits, h, lower, upper); }
		void queueEnableLimit(Handle h, bool enable) { pushCommand(Cmd::EnableLimit, h, 0, 0, enable ? 1u : 0u); }
		// Locks rotation at the bodies' current relative angle.
		void queueResetReferenceAngle(Handle h) { pushCommand(Cmd::ResetReferenceAngle, h); }

		void flushCommands(const BodySoA& bodies) override
		{
			flushImpl([this, &bodies](const ConstraintCommand& c, uint32_t i)
				{
					switch (static_cast<Cmd>(c.type))
					{
					case Cmd::LocalAnchorA: soa.localAnchorA[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAnchorB: soa.localAnchorB[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAxisA:
					{
						Vec2 axis(c.a, c.b);
						if (normalizeAxis(axis)) soa.localAxisA[i] = axis;
						break;
					}
					case Cmd::Stiffness: soa.stiffness[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Damping: soa.damping[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Limits:
						soa.lowerLimit[i] = std::fmin(c.a, c.b);
						soa.upperLimit[i] = std::fmax(c.a, c.b);
						break;
					case Cmd::EnableLimit: soa.enableLimit[i] = static_cast<uint8_t>(c.index != 0); break;
					case Cmd::ResetReferenceAngle:
						soa.referenceAngle[i] = computeReferenceAngle(bodies, soa.bodyIndexA[i], soa.bodyIndexB[i]);
						break;
					}
				});
		}

		std::optional<Data> getData(Handle h, const BodySoA& bodies) const noexcept
		{
			const uint32_t i = soa.indexOf(h);
			if (i == Handle::kNull) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(uint32_t i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			d.bodyHandleA = bodies.handleAt(soa.bodyIndexA[i]);
			d.bodyHandleB = bodies.handleAt(soa.bodyIndexB[i]);
			d.localAnchorA = soa.localAnchorA[i];
			d.localAnchorB = soa.localAnchorB[i];
			d.localAxisA = soa.localAxisA[i];
			d.referenceAngle = soa.referenceAngle[i];
			d.stiffness = soa.stiffness[i];
			d.damping = soa.damping[i];
			d.lowerLimit = soa.lowerLimit[i];
			d.upperLimit = soa.upperLimit[i];
			d.enableLimit = soa.enableLimit[i] != 0;
			return d;
		}

		const char* getName() const noexcept override { return "PrismaticJoint"; }

	private:
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, LocalAxisA, Stiffness, Damping, Limits, EnableLimit, ResetReferenceAngle };

		// (bodyB.rotation - bodyA.rotation) wrapped to (-pi, pi], robust to each body's rotation wrapping independently.
		static Real computeReferenceAngle(const BodySoA& bodies, ObjectDenseIndex a, ObjectDenseIndex b) noexcept
		{
			constexpr Real PI = Real(3.14159265358979323846);
			constexpr Real TWO_PI = Real(2) * PI;
			Real refAngle = bodies.rotation[b] - bodies.rotation[a];
			refAngle -= TWO_PI * std::floor((refAngle + PI) / TWO_PI);
			return refAngle;
		}
	};

	class SliderJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::SliderJoint, SliderJointSoA, SliderJointSolver, SliderJointSoAViewer>
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

		struct Data
		{
			Handle bodyHandleA{};
			Handle bodyHandleB{};
			Vec2 localAnchorA{ 0 };
			Vec2 localAnchorB{ 0 };
			Vec2 localAxisA{ 1, 0 };
			Real stiffness{ 0 };
			Real damping{ 0 };
			Real lowerLimit{ 0 };
			Real upperLimit{ 0 };
			bool enableLimit{ false };
		};

		Handle create(const CreateParams& p, BodySoA& bodies)
		{
			// Normalize the axis defensively; a zero-length axis would make the joint singular.
			const Real axisLenSq = glm::dot(p.localAxisA, p.localAxisA);
			const Vec2 axis = axisLenSq > Real(1e-12) ? p.localAxisA / glm::sqrt(axisLenSq) : Vec2(Real(1), Real(0));

			return createImpl(
				p.bodyIndexA, p.bodyIndexB, bodies,
				p.localAnchorA, p.localAnchorB, axis,
				p.stiffness, p.damping, p.lowerLimit, p.upperLimit, p.enableLimit
			);
		}

		void queueLocalAnchorA(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorA, h, v.x, v.y); }
		void queueLocalAnchorB(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorB, h, v.x, v.y); }
		// Re-normalized on apply; a zero-length axis is dropped.
		void queueLocalAxisA(Handle h, Vec2 axis) { pushCommand(Cmd::LocalAxisA, h, axis.x, axis.y); }
		void queueStiffness(Handle h, Real v) { pushCommand(Cmd::Stiffness, h, v); }
		void queueDamping(Handle h, Real v) { pushCommand(Cmd::Damping, h, v); }
		// Order-insensitive: stored as [fmin, fmax].
		void queueLimits(Handle h, Real lower, Real upper) { pushCommand(Cmd::Limits, h, lower, upper); }
		void queueEnableLimit(Handle h, bool enable) { pushCommand(Cmd::EnableLimit, h, 0, 0, enable ? 1u : 0u); }

		void flushCommands(const BodySoA&) override
		{
			flushImpl([this](const ConstraintCommand& c, uint32_t i)
				{
					switch (static_cast<Cmd>(c.type))
					{
					case Cmd::LocalAnchorA: soa.localAnchorA[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAnchorB: soa.localAnchorB[i] = Vec2(c.a, c.b); break;
					case Cmd::LocalAxisA:
					{
						Vec2 axis(c.a, c.b);
						if (normalizeAxis(axis)) soa.localAxisA[i] = axis;
						break;
					}
					case Cmd::Stiffness: soa.stiffness[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Damping: soa.damping[i] = std::fmax(Real(0), c.a); break;
					case Cmd::Limits:
						soa.lowerLimit[i] = std::fmin(c.a, c.b);
						soa.upperLimit[i] = std::fmax(c.a, c.b);
						break;
					case Cmd::EnableLimit: soa.enableLimit[i] = static_cast<uint8_t>(c.index != 0); break;
					}
				});
		}

		std::optional<Data> getData(Handle h, const BodySoA& bodies) const noexcept
		{
			const uint32_t i = soa.indexOf(h);
			if (i == Handle::kNull) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(uint32_t i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			d.bodyHandleA = bodies.handleAt(soa.bodyIndexA[i]);
			d.bodyHandleB = bodies.handleAt(soa.bodyIndexB[i]);
			d.localAnchorA = soa.localAnchorA[i];
			d.localAnchorB = soa.localAnchorB[i];
			d.localAxisA = soa.localAxisA[i];
			d.stiffness = soa.stiffness[i];
			d.damping = soa.damping[i];
			d.lowerLimit = soa.lowerLimit[i];
			d.upperLimit = soa.upperLimit[i];
			d.enableLimit = soa.enableLimit[i] != 0;
			return d;
		}

		const char* getName() const noexcept override { return "SliderJoint"; }

	private:
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, LocalAxisA, Stiffness, Damping, Limits, EnableLimit };
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

		struct Data
		{
			Handle bodyHandle{};
			Real targetSpeed{ 0 };
			Real maxTorque{ 0 };
			Real accumulatedImpulse{ 0 }; // Last solved impulse (warm-start); useful as torque feedback.
		};

		Handle create(const CreateParams& p, BodySoA& bodies)
		{
			return createSingleImpl(p.bodyIndex, bodies, p.targetSpeed, p.maxTorque);
		}

		// accumulatedImpulse is intentionally left untouched on change.
		void queueTargetSpeed(Handle h, Real v) { pushCommand(Cmd::TargetSpeed, h, v); }
		void queueMaxTorque(Handle h, Real v) { pushCommand(Cmd::MaxTorque, h, v); }

		void flushCommands(const BodySoA&) override
		{
			flushImpl([this](const ConstraintCommand& c, uint32_t i)
				{
					switch (static_cast<Cmd>(c.type))
					{
					case Cmd::TargetSpeed: soa.targetSpeed[i] = c.a; break;
					case Cmd::MaxTorque: soa.maxTorque[i] = std::fmax(Real(0), c.a); break;
					}
				});
		}

		std::optional<Data> getData(Handle h, const BodySoA& bodies) const noexcept
		{
			const uint32_t i = soa.indexOf(h);
			if (i == Handle::kNull) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(uint32_t i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			d.bodyHandle = bodies.handleAt(soa.bodyIndex[i]);
			d.targetSpeed = soa.targetSpeed[i];
			d.maxTorque = soa.maxTorque[i];
			d.accumulatedImpulse = soa.accumulatedImpulse[i];
			return d;
		}

		const char* getName() const noexcept override { return "Motor"; }

	private:
		enum class Cmd : uint8_t { TargetSpeed, MaxTorque };
	};
}