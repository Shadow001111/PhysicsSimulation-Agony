#pragma once
#include "PS_AGONY2D/SoA/BodySoA.h"

#include "PS_AGONY2D/Solving/ConstraintSolverBase.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace PS_AGONY2D
{
	// 'type' is the per-system Cmd enum.
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

		virtual void removeConstraint(ObjectDenseIndex index, BodySoA& bodies) = 0;
		virtual void remapBodyIndex(ObjectDenseIndex index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) = 0;

		// Applies and clears the queued modification commands.
		// Stale handles are dropped; values are sanitized on apply.
		virtual void flushCommands(const BodySoA& bodies) = 0;

		// Binds data views and (re)plans if this type changed. Solving is driven by ConstraintScheduler through getSolver().
		virtual void prepare(BodySoA& bodies) = 0;

		virtual void appendBodyPairs(std::vector<ObjectPair>& pairs) const = 0;

		virtual IConstraintSolver& getSolver() noexcept = 0;

		virtual size_t getCount() const noexcept = 0;
		virtual size_t getMemoryUsage() const noexcept = 0;
		virtual size_t getSolverMemoryUsage() const noexcept = 0;

		virtual ConstraintType getType() const noexcept = 0;
		virtual const char* getName() const noexcept = 0;
	};

	using ConstraintSystemArray = std::array<IConstraintSystem*, static_cast<size_t>(ConstraintType::COUNT)>;

	template<ConstraintType Type, class SoA, class Viewer, class Solver>
	class ConstraintSystemBase : public IConstraintSystem
	{
	protected:
		SoA soa;
		Solver solver;
		bool wasSoAChanged = false;
		std::vector<ConstraintCommand> commands;

		// Single-body constraints expose `bodyIndex` instead of `bodyIndexA/B`.
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
				if (resolved == Handle::NULL_DENSE_INDEX) continue;
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

		// Resolves body handles to dense indices. False if any is stale.
		static bool resolve(const BodySoA& bodies, Handle hA, Handle hB, ObjectDenseIndex& a, ObjectDenseIndex& b) noexcept
		{
			a = bodies.indexOf(hA);
			b = bodies.indexOf(hB);
			return a != Handle::NULL_DENSE_INDEX && b != Handle::NULL_DENSE_INDEX;
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
			}
			else
			{
				// Unlink the constraint being removed from both of its bodies.
				const ObjectPair bodyIndexPair = soa.bodyIndexPair[index];
				bodies.removeAttachment(bodyIndexPair.a, Type, index);
				bodies.removeAttachment(bodyIndexPair.b, Type, index);

				const ObjectDenseIndex oldBackIndex = soa.swapRemove(index);

				// If a constraint got swapped into 'index', relink it there.
				if (oldBackIndex != index)
				{
					const ObjectPair bodyIndexPair = soa.bodyIndexPair[index];
					bodies.removeAttachment(bodyIndexPair.a, Type, oldBackIndex);
					bodies.addAttachment(bodyIndexPair.a, Type, index);
					bodies.removeAttachment(bodyIndexPair.b, Type, oldBackIndex);
					bodies.addAttachment(bodyIndexPair.b, Type, index);
				}
			}
			wasSoAChanged = true;
		}

		void remapBodyIndex(ObjectDenseIndex index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) final
		{
			if (index >= soa.getCount()) [[unlikely]]
			{
				return;
			}
			if constexpr (IsSingleBody)
			{
				if (soa.bodyIndex[index] == oldBodyIndex) soa.bodyIndex[index] = newBodyIndex;
			}
			else
			{
				ObjectPair& bodyIndexPair = soa.bodyIndexPair[index];
				if (bodyIndexPair.a == oldBodyIndex) bodyIndexPair.a = newBodyIndex;
				if (bodyIndexPair.b == oldBodyIndex) bodyIndexPair.b = newBodyIndex;
			}
			wasSoAChanged = true;
		}

		void prepare(BodySoA& bodies) final
		{
			solver.setDataViewers(bodies, Viewer(soa));
			solver.prepare(wasSoAChanged);
			wasSoAChanged = false;
		}

		void appendBodyPairs(std::vector<ObjectPair>& pairs) const final
		{
			if constexpr (!IsSingleBody)
			{
				pairs.insert(pairs.end(), soa.bodyIndexPair.begin(), soa.bodyIndexPair.end());
			}
		}

		IConstraintSolver& getSolver() noexcept final { return solver; }

		Viewer getViewer() const noexcept { return Viewer(soa); }

		// Handle lookup. indexOf returns Handle::kNull if stale.
		bool isValid(Handle h) const noexcept { return soa.indexOf(h) != Handle::NULL_DENSE_INDEX; }
		ObjectDenseIndex indexOf(Handle h) const noexcept { return soa.indexOf(h); }
		Handle handleAt(ObjectDenseIndex index) const noexcept { return soa.handleAt(index); }

		size_t getCount() const noexcept final { return soa.getCount(); }
		size_t getMemoryUsage() const noexcept final { return soa.getMemoryUsage(); }
		size_t getSolverMemoryUsage() const noexcept final { return solver.getMemoryUsage(); }

		ConstraintType getType() const noexcept final { return Type; }
	};
}