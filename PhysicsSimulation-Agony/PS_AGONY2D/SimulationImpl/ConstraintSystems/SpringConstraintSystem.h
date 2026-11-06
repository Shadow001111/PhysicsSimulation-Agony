#pragma once
#include "ConstraintSystemBase.h"

#include "PS_AGONY2D/SoA/Constraints/SpringSoA.h"
#include "PS_AGONY2D/Solving/SpringSolver.h"

namespace PS_AGONY2D
{
	class SpringConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Spring, SpringSoA, SpringSoAViewer, SpringSolver>
	{
		enum class Cmd : uint8_t { LocalAnchorA, LocalAnchorB, RestLength, Stiffness, Damping };
	public:
		struct CreateParams
		{
			Handle bodyHandleA;
			Handle bodyHandleB;
			Vec2 localAnchorA;
			Vec2 localAnchorB;
			Real restLength;
			Real stiffness;
			Real damping;
		};

		// Create: nullopt if a body handle is stale.
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

		std::optional<Handle> create(const CreateParams& p, BodySoA& bodies)
		{
			ObjectDenseIndex a, b;
			if (!resolve(bodies, p.bodyHandleA, p.bodyHandleB, a, b)) return std::nullopt;
			return createImpl(a, b, bodies, p.localAnchorA, p.localAnchorB, p.restLength, p.stiffness, p.damping);
		}

		// Deferred modification commands.
		void queueLocalAnchorA(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorA, h, v.x, v.y); }
		void queueLocalAnchorB(Handle h, Vec2 v) { pushCommand(Cmd::LocalAnchorB, h, v.x, v.y); }
		void queueRestLength(Handle h, Real v) { pushCommand(Cmd::RestLength, h, v); }
		void queueStiffness(Handle h, Real v) { pushCommand(Cmd::Stiffness, h, v); }
		void queueDamping(Handle h, Real v) { pushCommand(Cmd::Damping, h, v); }

		void flushCommands(const BodySoA&) override
		{
			flushImpl([this](const ConstraintCommand& c, ObjectDenseIndex i)
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

		// Instant reads (no queue). nullopt if stale / out of range.
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
	};
}