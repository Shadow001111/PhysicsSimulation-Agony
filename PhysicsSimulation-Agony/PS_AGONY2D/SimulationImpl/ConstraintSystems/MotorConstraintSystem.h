#pragma once
#include "ConstraintSystemBase.h"

#include "PS_AGONY2D/SoA/Constraints/MotorSoA.h"
#include "PS_AGONY2D/Solving/MotorSolver.h"

namespace PS_AGONY2D
{
	class MotorConstraintSystem final :
		public ConstraintSystemBase<ConstraintType::Motor, MotorSoA, MotorSoAViewer, MotorSolver>
	{
		enum class Cmd : uint8_t { TargetSpeed, MaxTorque };
	public:
		struct CreateParams
		{
			Handle bodyHandle;
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

		// Call through ObjectManager::createMotor (wakes the body, registers the attachment).
		ObjectDenseIndex createAt(ObjectDenseIndex bodyIndex, const CreateParams& p)
		{
			return createSingleImpl(bodyIndex, p.targetSpeed, p.maxTorque);
		}

		// 'accumulatedImpulse is intentionally' left untouched on change.
		void queueTargetSpeed(Handle h, Real v) { pushCommand(Cmd::TargetSpeed, h, v); }
		void queueMaxTorque(Handle h, Real v) { pushCommand(Cmd::MaxTorque, h, v); }

		void flushCommands(const BodySoA& bodies, std::vector<Handle>& bodiesToWake) override
		{
			flushImpl(bodies, bodiesToWake, [this](const ConstraintCommand& c, ObjectDenseIndex i)
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
			const ObjectDenseIndex i = soa.indexOf(h);
			if (i == Handle::NULL_DENSE_INDEX) return std::nullopt;
			return getDataAt(i, bodies);
		}

		std::optional<Data> getDataAt(ObjectDenseIndex i, const BodySoA& bodies) const noexcept
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
	};
}