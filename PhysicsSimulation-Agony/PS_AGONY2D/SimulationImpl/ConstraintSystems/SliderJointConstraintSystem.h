#pragma once
#include "ConstraintSystemBase.h"

#include "PS_AGONY2D/SoA/Constraints/SliderJointSoA.h"
#include "PS_AGONY2D/Solving/SliderJointSolver.h"

namespace PS_AGONY2D
{
	class SliderJointConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::SliderJoint, SliderJointSoA, SliderJointSoAViewer, SliderJointSolver>
	{
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, LocalAxisA, Stiffness, Damping, Limits, EnableLimit };
	public:
		struct CreateParams
		{
			Handle bodyHandleA;
			Handle bodyHandleB;
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

		// Call through ObjectManager::createSliderJoint (wakes bodies, registers attachments).
		ObjectDenseIndex createAt(ObjectDenseIndex a, ObjectDenseIndex b, const CreateParams& p)
		{
			// Normalize the axis defensively; a zero-length axis would make the joint singular.
			const Real axisLenSq = glm::dot(p.localAxisA, p.localAxisA);
			const Vec2 axis = axisLenSq > Real(1e-12) ? p.localAxisA / glm::sqrt(axisLenSq) : Vec2(Real(1), Real(0));

			return createImpl(
				a, b,
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

		void flushCommands(const BodySoA& bodies, std::vector<Handle>& bodiesToWake) override
		{
			flushImpl(bodies, bodiesToWake, [this](const ConstraintCommand& c, ObjectDenseIndex i)
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
			const ObjectDenseIndex i = soa.indexOf(h);
			if (i == Handle::NULL_DENSE_INDEX) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(ObjectDenseIndex i, const BodySoA& bodies) const noexcept
		{
			if (i >= soa.getCount()) return std::nullopt;
			Data d;
			const ObjectPair bodyIndexPair = soa.bodyIndexPair[i];
			d.bodyHandleA = bodies.handleAt(bodyIndexPair.a);
			d.bodyHandleB = bodies.handleAt(bodyIndexPair.b);
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
	};
}