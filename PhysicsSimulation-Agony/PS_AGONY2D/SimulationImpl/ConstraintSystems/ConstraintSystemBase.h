#pragma once
#include "PS_AGONY2D/SoA/BodySoA.h"

#include "PS_AGONY2D/Solving/ConstraintSolverBase.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
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

	// Dense order [Inactive][Sleeping][Active][Unused], ascending by change frequency like BodyGroup.
	// Inactive: every body of the constraint is static. Sleeping: reserved, not implemented yet.
	enum class ConstraintGroup : uint8_t { Inactive, Sleeping, Active, Unused };

	// One implementation per ConstraintType. Owns the constraint SoA and its solver, wires
	// them to BodySoA (attachments, cascade deletion, index remapping) and solves them.
	class IConstraintSystem
	{
	public:
		uint32_t iterations = 6;

		// groupStarts[g] = first index of group g, [Unused] = end of live constraints. Changed by
		// ObjectManager::moveConstraintToGroup only.
		uint32_t groupStarts[size_t(ConstraintGroup::Unused) + 1]{};

		// Set when the Active range changed (content, order or body layout). Consumed by prepare().
		bool wasSoAChanged = false;

		IndexRange range(ConstraintGroup group) const noexcept
		{
			const size_t index = static_cast<size_t>(group);
			if (index < size_t(ConstraintGroup::Unused)) [[likely]]
			{
				return { groupStarts[index], groupStarts[index + 1] };
			}
			return { groupStarts[size_t(ConstraintGroup::Unused)], static_cast<ObjectDenseIndex>(getCount()) };
		}

		ConstraintGroup groupOf(ObjectDenseIndex index) const noexcept
		{
			for (size_t i = 0; i < size_t(ConstraintGroup::Unused); i++)
			{
				if (index >= groupStarts[i] && index < groupStarts[i + 1])
				{
					return static_cast<ConstraintGroup>(i);
				}
			}
			return ConstraintGroup::Unused;
		}

		virtual ~IConstraintSystem() = default;

		// Swaps two constraints in every field (handles included). Does not touch groups or body attachments: ObjectManager owns that.
		virtual void swap(ObjectDenseIndex a, ObjectDenseIndex b) = 0;
		// Removes the last constraint. Callers move it to the Unused group first.
		virtual void popBack() = 0;
		// Body indices of a constraint (b == NULL_DENSE_INDEX for single-body). index must be in range.
		virtual void getBodyIndices(ObjectDenseIndex index, ObjectDenseIndex& a, ObjectDenseIndex& b) const noexcept = 0;
		// Handle -> dense index (Handle::NULL_DENSE_INDEX if stale).
		virtual ObjectDenseIndex indexOf(Handle h) const noexcept = 0;
		virtual void remapBodyIndex(ObjectDenseIndex index, ObjectDenseIndex oldBodyIndex, ObjectDenseIndex newBodyIndex) = 0;

		// Applies and clears the queued modification commands.
		// Stale handles are dropped; values are sanitized on apply.
		// Appends the body handles of every edited constraint to bodiesToWake (ObjectManager wakes them).
		virtual void flushCommands(const BodySoA& bodies, std::vector<Handle>& bodiesToWake) = 0;

		// Binds data views and (re)plans if this type changed. Solving is driven by ConstraintScheduler through getSolver().
		virtual void prepare(BodySoA& bodies) = 0;

		// Body pairs of the Active range only (empty for single-body types). Element i is constraint index range(Active).begin + i.
		virtual std::span<const ObjectPair> getActiveEdges() const = 0;

		virtual IConstraintSolver& getSolver() noexcept = 0;

		virtual size_t getCount() const noexcept = 0;
		virtual size_t getSoAMemoryUsage() const noexcept = 0;
		virtual size_t getSolverMemoryUsage() const noexcept = 0;

		virtual ConstraintType getType() const noexcept = 0;
		virtual const char* getName() const noexcept = 0;
	};

	using ConstraintSystemArray = std::array<IConstraintSystem*, static_cast<size_t>(ConstraintType::COUNT)>;

	template<ConstraintType Type, class SoA, class Viewer, class Solver, class Derived>
	class ConstraintSystemBase : public IConstraintSystem
	{
	public:
		static constexpr ConstraintType TYPE = Type;
		// Single-body constraints expose `bodyIndex` instead of `bodyIndexA/B`.
		static constexpr bool IsSingleBody = requires(SoA & s) { s.bodyIndex; };
	protected:
		SoA soa;
		Solver solver;
		std::vector<ConstraintCommand> commands;

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

		// Normalized axis, or (1, 0) if zero-length.
		static Vec2 sanitizeAxis(Vec2 axis) noexcept
		{
			return normalizeAxis(axis) ? axis : Vec2(Real(1), Real(0));
		}

		// Appends a constraint (body indices first in args). Returns its dense index. Attachments are ObjectManager's job.
		template<class... Args>
		ObjectDenseIndex createImpl(Args&&... args)
		{
			return static_cast<ObjectDenseIndex>(soa.append(std::forward<Args>(args)...));
		}
	public:
		void swap(ObjectDenseIndex a, ObjectDenseIndex b) final { soa.swap(a, b); }
		void popBack() final { soa.popBack(); }

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
			if (index >= groupStarts[size_t(ConstraintGroup::Active)]) wasSoAChanged = true;
		}

		void prepare(BodySoA& bodies) final
		{
			solver.setDataViewers(bodies, Viewer(soa, range(ConstraintGroup::Active)));
			solver.prepare(wasSoAChanged);
			wasSoAChanged = false;
		}

		std::span<const ObjectPair> getActiveEdges() const final
		{
			if constexpr (IsSingleBody)
			{
				return {};
			}
			else
			{
				const IndexRange active = range(ConstraintGroup::Active);
				return std::span<const ObjectPair>(soa.bodyIndexPair.data() + active.begin, active.end - active.begin);
			}
		}

		IConstraintSolver& getSolver() noexcept final { return solver; }

		Viewer getViewer() const noexcept { return Viewer(soa); }

		// Handle lookup. indexOf returns Handle::NULL_DENSE_INDEX if stale.
		bool isValid(Handle h) const noexcept { return soa.indexOf(h) != Handle::NULL_DENSE_INDEX; }
		ObjectDenseIndex indexOf(Handle h) const noexcept final { return soa.indexOf(h); }

		// nullopt if stale. Derived provides getDataAt (nullopt if out of range).
		auto getData(Handle h, const BodySoA& bodies) const noexcept
		{
			return static_cast<const Derived&>(*this).getDataAt(soa.indexOf(h), bodies);
		}

		Handle handleAt(ObjectDenseIndex index) const noexcept { return soa.handleAt(index); }

		size_t getCount() const noexcept final { return soa.getCount(); }
		size_t getSoAMemoryUsage() const noexcept final { return soa.getMemoryUsage(); }
		size_t getSolverMemoryUsage() const noexcept final { return solver.getMemoryUsage(); }

		ConstraintType getType() const noexcept final { return Type; }
	};
}