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

		// Raw swap-and-pop. Returns the old back index (== index if nothing moved), NULL_DENSE_INDEX if out of range.
		// Does NOT touch body attachments: ObjectManager owns that.
		virtual ObjectDenseIndex swapRemove(ObjectDenseIndex index) = 0;
		// Body indices of a constraint (b == NULL_DENSE_INDEX for single-body). index must be in range.
		virtual void getBodyIndices(ObjectDenseIndex index, ObjectDenseIndex& a, ObjectDenseIndex& b) const noexcept = 0;
		virtual void remapBodyIndex(ObjectDenseIndex index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) = 0;

		// Applies and clears the queued modification commands.
		// Stale handles are dropped; values are sanitized on apply.
		// Appends the body handles of every edited constraint to bodiesToWake (ObjectManager wakes them).
		virtual void flushCommands(const BodySoA& bodies, std::vector<Handle>& bodiesToWake) = 0;

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

		// Resolves each handle, calls apply(command, denseIndex) and reports the touched bodies. Stale handles are dropped.
		// Value edits never touch wasSoAChanged.
		template<class F>
		void flushImpl(const BodySoA& bodies, std::vector<Handle>& wake, F&& apply)
		{
			if (commands.empty()) return;

			for (const ConstraintCommand& command : commands)
			{
				const uint32_t resolved = soa.indexOf(command.handle);
				if (resolved == Handle::NULL_DENSE_INDEX) continue;
				apply(command, resolved);

				ObjectDenseIndex a, b;
				getBodyIndices(resolved, a, b);
				wake.push_back(bodies.handleAt(a));
				if (b != Handle::NULL_DENSE_INDEX) wake.push_back(bodies.handleAt(b));
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

		// Appends a single-body constraint. Returns its dense index. Attachments are ObjectManager's job.
		template<class... Args>
		ObjectDenseIndex createSingleImpl(ObjectDenseIndex index, Args&&... args)
		{
			const ObjectDenseIndex newIndex = static_cast<ObjectDenseIndex>(soa.append(index, std::forward<Args>(args)...));
			wasSoAChanged = true;
			return newIndex;
		}

		// Appends a constraint. Returns its dense index. Attachments are ObjectManager's job.
		template<class... Args>
		ObjectDenseIndex createImpl(ObjectDenseIndex indexA, ObjectDenseIndex indexB, Args&&... args)
		{
			const ObjectDenseIndex newIndex = static_cast<ObjectDenseIndex>(soa.append(indexA, indexB, std::forward<Args>(args)...));
			wasSoAChanged = true;
			return newIndex;
		}
	public:
		ObjectDenseIndex swapRemove(ObjectDenseIndex index) final
		{
			if (index >= soa.getCount()) [[unlikely]] return Handle::NULL_DENSE_INDEX;
			const ObjectDenseIndex oldBackIndex = soa.swapRemove(index);
			wasSoAChanged = true;
			return oldBackIndex;
		}

		void getBodyIndices(ObjectDenseIndex index, ObjectDenseIndex& a, ObjectDenseIndex& b) const noexcept final
		{
			if constexpr (IsSingleBody)
			{
				a = soa.bodyIndex[index];
				b = Handle::NULL_DENSE_INDEX;
			}
			else
			{
				const ObjectPair pair = soa.bodyIndexPair[index];
				a = pair.a;
				b = pair.b;
			}
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