from __future__ import annotations

import argparse
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

# Cleaned up global regex patterns to prevent greedy bleeding across tokens
ALIGN_DEF_RE = re.compile(r"^\s*align\(([^)]+)\)\s*is\s*:(.*)$", re.IGNORECASE)
ALIGN_ATTR_RE = re.compile(r"align\((.*?)\)", re.IGNORECASE)
SIZE_ATTR_RE = re.compile(r"size\((.*?)\)", re.IGNORECASE)
INT_RE = re.compile(r"\d+")
COMMENT_RE = re.compile(r"^\s*//")


@dataclass(frozen=True)
class BufferInterval:
    name: str
    depth: int
    start: int
    end: int
    alignment_name: str
    alignment: int
    size: int


def buffers_conflict(a: BufferInterval, b: BufferInterval) -> bool:
    # Two buffers conflict (cannot share a region) exactly when their live
    # ranges overlap in time, using a half-open [start, end) comparison.
    return a.end > b.start and a.start < b.end


@dataclass
class Region:
    id: int
    alignment: int
    size: int = 0
    members: List[BufferInterval] = field(default_factory=list)

    def can_accept(self, buffer: BufferInterval) -> bool:
        # A region only has room for a new buffer if none of its current
        # occupants are alive at the same time as that buffer.
        for b in self.members:
            if buffers_conflict(buffer, b):
                return False
        return True

    def add(self, buffer: BufferInterval) -> None:
        self.members.append(buffer)
        # A region's footprint is driven by its largest occupant, and its
        # alignment requirement by the strictest occupant, since all
        # occupants share the same underlying memory over time.
        self.alignment = max(self.alignment, buffer.alignment)
        self.size = max(self.size, buffer.size)


def is_power_of_two(x: int) -> bool:
    return x > 0 and (x & (x - 1)) == 0


def parse_alignment_definitions(lines: Iterable[str]) -> Dict[str, List[int]]:
    out: Dict[str, List[int]] = {}
    for line in lines:
        m = ALIGN_DEF_RE.match(line)
        if not m:
            continue
        name = m.group(1).strip()
        ints = [int(x) for x in INT_RE.findall(m.group(2))]
        if not ints:
            raise ValueError(f"Alignment definition for {name!r} has no integers: {line!r}")
        for v in ints:
            if not is_power_of_two(v):
                raise ValueError(f"Alignment value {v} for {name!r} is not a power of two")
        out[name] = ints
    return out


def resolve_alignment(token: str, align_defs: Dict[str, List[int]]) -> int:
    token = token.strip()
    if token.isdigit():
        value = int(token)
        if not is_power_of_two(value):
            raise ValueError(f"Alignment {value} is not a power of two")
        return value
    if token not in align_defs:
        raise KeyError(f"Unknown alignment name: {token!r}")
    value = max(align_defs[token])
    if not is_power_of_two(value):
        raise ValueError(f"Alignment list for {token!r} contains non-power-of-two values")
    return value


def parse_usage_lines(
    lines: List[str],
    align_defs: Dict[str, List[int]],
) -> List[BufferInterval]:
    intervals: List[BufferInterval] = []
    stack: List[Tuple[int, str, int, int, int]] = []
    parsed_count = 0

    for line in lines:
        if ALIGN_DEF_RE.match(line):
            continue

        if not line.strip() or COMMENT_RE.match(line):
            while stack:
                d, n, a, s, start = stack.pop()
                intervals.append(BufferInterval(n, d, start, parsed_count, str(a), a, s))
            continue

        align_m = ALIGN_ATTR_RE.search(line)
        if not align_m:
            continue

        size_m = SIZE_ATTR_RE.search(line)
        first_attr_idx = align_m.start()
        if size_m and size_m.start() < first_attr_idx:
            first_attr_idx = size_m.start()

        content = line[:first_attr_idx]
        align_token = align_m.group(1).strip()
        
        size = 0
        if size_m:
            size_token = size_m.group(1).strip()
            if size_token.isdigit():
                size = int(size_token)
            else:
                raise ValueError(f"Invalid size value {size_token!r} in line: {line!r}")

        leading_len = len(content) - len(content.lstrip(" |"))
        depth = leading_len
        name = content[leading_len:].strip()

        if not name:
            raise ValueError(f"Missing buffer name in line: {line!r}")

        alignment = resolve_alignment(align_token, align_defs)

        while stack and stack[-1][0] > depth:
            d, n, a, s, start = stack.pop()
            intervals.append(BufferInterval(n, d, start, parsed_count, str(a), a, s))

        stack.append((depth, name, alignment, size, parsed_count))
        parsed_count += 1

    while stack:
        d, n, a, s, start = stack.pop()
        intervals.append(BufferInterval(n, d, start, parsed_count, str(a), a, s))

    intervals.sort(key=lambda x: (x.start, x.depth))
    return intervals


class RegionAllocator:
    """
    Fast greedy region allocator.

    This makes exactly one irrevocable pass over the buffers, in start
    order, and commits each one to whichever compatible region looks
    cheapest right now. That is fast (roughly O(n * regions)) and often
    good, but it is a heuristic, not an optimizer: an early "cheap" choice
    can permanently block a much better pairing that would only become
    visible later, so the total footprint it reports is not guaranteed to
    be the smallest one possible. See OptimalRegionAllocator below for an
    allocator that searches for the true minimum instead of taking the
    first locally-good option.
    """

    def allocate(self, intervals: List[BufferInterval]) -> List[Region]:
        intervals = sorted(intervals, key=lambda x: (x.start, x.depth))
        regions: List[Region] = []

        for buffer in intervals:
            chosen: Optional[Region] = None
            candidates = [r for r in regions if r.can_accept(buffer)]
            if candidates:
                # Prefer the region that grows the least, breaking ties
                # toward smaller / lower-alignment regions, all evaluated
                # only against the buffers seen so far, not the whole input.
                candidates.sort(key=lambda r: (
                    max(0, buffer.size - r.size),
                    r.size,
                    max(0, buffer.alignment - r.alignment)
                ))
                chosen = candidates[0]

            if chosen is None:
                # No existing region can host this buffer without a
                # conflict, so a brand-new region has to be opened for it.
                chosen = Region(id=len(regions), alignment=buffer.alignment, size=buffer.size)
                regions.append(chosen)

            chosen.add(buffer)

        return regions


class _SearchRegion:
    """
    Lightweight, mutable region used only inside OptimalRegionAllocator's
    search. Kept separate from the public Region dataclass so branch-and-
    bound can cheaply mutate and backtrack state millions of times without
    touching the "real" Region objects returned to callers.
    """

    __slots__ = ("members", "size", "alignment")

    def __init__(self) -> None:
        self.members: List[BufferInterval] = []
        self.size = 0
        self.alignment = 0

    def can_accept(self, buf: BufferInterval) -> bool:
        # Same rule as Region.can_accept: no member may overlap in time.
        for m in self.members:
            if buffers_conflict(buf, m):
                return False
        return True

    def cost_after_adding(self, buf: BufferInterval) -> int:
        # How much the total footprint would grow if buf moved in here;
        # zero if buf already fits under this region's current peak size.
        return max(0, buf.size - self.size)


class OptimalRegionAllocator:
    """
    Exact region allocator that searches for the assignment with the
    smallest possible total memory footprint, instead of committing to the
    first locally-cheap choice the way RegionAllocator does.

    Approach: branch-and-bound over "which region does each buffer go
    into", trying the largest buffers first (they dominate the cost, so
    placing them early tightens the bound fastest), and pruning any
    partial assignment whose cost has already reached or exceeded the
    best complete assignment found so far. Region size only ever grows as
    buffers are added to it, never shrinks, so a partial assignment's cost
    is always a valid lower bound on the cost of any completion of it,
    which is what makes this pruning rule correct.

    The search is seeded with RegionAllocator's fast greedy result, so:
      * the very first "best known" answer is already a decent one, giving
        strong pruning power from node one, and.
      * if the search is cut off early by the time/node budget below, the
        answer returned is never worse than plain greedy would have given.

    This is worst-case exponential (the underlying problem is a weighted
    graph-coloring variant, which is NP-hard in general), so it is meant
    for realistic buffer counts, not huge inputs. The time_limit_seconds
    and node_limit knobs cap the search so it always terminates and always
    returns a usable answer: exact if it finished in time, best-effort
    otherwise.
    """

    def __init__(self, time_limit_seconds: float = 5.0, node_limit: int = 300_000):
        # Safety valves so pathological inputs degrade gracefully instead
        # of making the tool hang.
        self.time_limit_seconds = time_limit_seconds
        self.node_limit = node_limit
        self.exact = True  # Flips to False if the search is cut off before finishing.
        self.stop_reason: Optional[str] = None  # "time" or "nodes", set only if cut off early.

    def allocate(self, intervals: List[BufferInterval]) -> List[Region]:
        if not intervals:
            return []

        # Seed with the greedy solution, both as the initial "best found"
        # bound and as a guaranteed fallback answer if the budget runs out.
        greedy_regions = RegionAllocator().allocate(intervals)
        self._best_cost = sum(r.size for r in greedy_regions)
        self._best_assignment = self._regions_to_assignment(greedy_regions, intervals)

        # Visit the biggest buffers first, since they contribute the most
        # to the final cost and therefore prune the search tree hardest.
        order = sorted(range(len(intervals)), key=lambda i: (-intervals[i].size, intervals[i].start))

        state: List[_SearchRegion] = []
        assignment = [-1] * len(intervals)  # assignment[buffer_index] -> region_index.

        self._deadline = time.monotonic() + self.time_limit_seconds
        self._nodes = 0

        self._search(intervals, order, 0, state, assignment, current_cost=0)

        return self._assignment_to_regions(intervals, self._best_assignment)

    def _search(
        self,
        intervals: List[BufferInterval],
        order: List[int],
        pos: int,
        regions: List[_SearchRegion],
        assignment: List[int],
        current_cost: int,
    ) -> None:
        # Enforce the time / node budget so the search always terminates,
        # simply keeping whatever best answer has already been found. The
        # two limits are tracked separately so callers can report the real
        # reason instead of always blaming the clock, since the node cap
        # can be hit almost instantly on a highly-branching input while
        # wall-clock time stays low.
        self._nodes += 1
        if self._nodes > self.node_limit:
            self.exact = False
            self.stop_reason = "nodes"
            return
        if time.monotonic() > self._deadline:
            self.exact = False
            self.stop_reason = "time"
            return

        # All buffers placed: this is a complete, valid assignment, so
        # check whether it beats the best one seen so far.
        if pos == len(order):
            if current_cost < self._best_cost:
                self._best_cost = current_cost
                self._best_assignment = assignment.copy()
            return

        # Core pruning rule: region sizes are monotonically non-decreasing
        # as buffers are added, so current_cost is a valid lower bound on
        # the cost of any completion of this partial assignment. If it has
        # already caught up to the best known total, finishing this branch
        # cannot possibly improve on it.
        if current_cost >= self._best_cost:
            return

        buf_idx = order[pos]
        buf = intervals[buf_idx]

        # Option A: reuse an existing, compatible region. Try the cheapest
        # (least footprint growth) option first, so good solutions, and
        # therefore tighter bounds, are found as early as possible.
        candidates = [r for r in regions if r.can_accept(buf)]
        candidates.sort(key=lambda r: r.cost_after_adding(buf))

        for r in candidates:
            growth = r.cost_after_adding(buf)
            old_size, old_alignment = r.size, r.alignment
            region_idx = regions.index(r)

            r.members.append(buf)
            r.size = max(r.size, buf.size)
            r.alignment = max(r.alignment, buf.alignment)
            assignment[buf_idx] = region_idx

            self._search(intervals, order, pos + 1, regions, assignment, current_cost + growth)

            # Undo the tentative placement so sibling branches see the
            # region exactly as it was before this attempt.
            r.members.pop()
            r.size, r.alignment = old_size, old_alignment

        # Option B: open a brand-new region for this buffer. Symmetry
        # breaking: only ever branch on a single fresh empty region per
        # node, because any two never-yet-used regions are interchangeable
        # and exploring more than one just wastes search time.
        new_region = _SearchRegion()
        new_region.members.append(buf)
        new_region.size = buf.size
        new_region.alignment = buf.alignment
        regions.append(new_region)
        assignment[buf_idx] = len(regions) - 1

        self._search(intervals, order, pos + 1, regions, assignment, current_cost + buf.size)

        regions.pop()  # Backtrack: remove the region entirely, not just its contents.

    @staticmethod
    def _regions_to_assignment(regions: List[Region], intervals: List[BufferInterval]) -> List[int]:
        # Converts RegionAllocator's output into a buffer-index -> region-id
        # mapping, used purely to seed the search's "best known" state.
        index_of = {id(b): i for i, b in enumerate(intervals)}
        assignment = [-1] * len(intervals)
        for r in regions:
            for b in r.members:
                assignment[index_of[id(b)]] = r.id
        return assignment

    @staticmethod
    def _assignment_to_regions(intervals: List[BufferInterval], assignment: List[int]) -> List[Region]:
        # Rebuilds public Region objects (with freshly recomputed size and
        # alignment) from a raw buffer-index -> region-index mapping.
        by_region: Dict[int, List[BufferInterval]] = {}
        for buf_idx, region_idx in enumerate(assignment):
            by_region.setdefault(region_idx, []).append(intervals[buf_idx])

        regions: List[Region] = []
        for new_id, (_, members) in enumerate(sorted(by_region.items())):
            region = Region(id=new_id, alignment=0)
            for b in members:
                region.add(b)
            regions.append(region)
        return regions


def print_regions(regions: List[Region], intervals: List[BufferInterval]) -> None:
    # 1. Calculate raw total (before combining)
    raw_memory = sum(b.size for b in intervals)
    
    # 2. Calculate optimized total (after combining)
    optimized_memory = sum(r.size for r in regions)
    
    # 3. Calculate ratio
    percentage = (optimized_memory / raw_memory * 100) if raw_memory > 0 else 0

    print("\nRegion assignment:")
    for region in regions:
        # How much memory this specific region saved: the sum of what its
        # members would have needed individually, minus what the region
        # actually costs (its single largest member's size), since they
        # now take turns sharing one slot instead of each getting their own.
        raw_region_memory = sum(b.size for b in region.members)
        region_savings = raw_region_memory - region.size
        print(f"  Region {region.id}  effective_align={region.alignment}  savings={region_savings:,} bytes")
        for b in sorted(region.members, key=lambda x: x.start):
            print(f"    {b.name:30s}  align={b.alignment}  size={b.size}")
    
    print("\n" + "="*40)
    print("Memory Footprint Report:")
    print(f"  Raw Memory (No reuse):   {raw_memory:,} bytes")
    print(f"  Optimized Memory:        {optimized_memory:,} bytes")
    print(f"  Reduction:               {raw_memory - optimized_memory:,} bytes")
    print(f"  Efficiency:              {percentage:.2f}% of original size")
    print("="*40)


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse custom indentation scopes and allocate reusable regions.")
    parser.add_argument("file", type=Path, help="Input text file")
    # Default to the exact search; --fast-greedy skips straight to the old
    # heuristic for very large inputs where the search budget below would
    # not be worth spending.
    parser.add_argument(
        "--fast-greedy",
        action="store_true",
        help="Skip the optimal search and use the fast greedy heuristic only.",
    )
    parser.add_argument(
        "--time-limit",
        type=float,
        default=5.0,
        help="Seconds to spend searching for the optimal assignment before falling back to the best found so far.",
    )
    args = parser.parse_args()

    text = args.file.read_text(encoding="utf-8")
    lines = text.splitlines()

    align_defs = parse_alignment_definitions(lines)
    intervals = parse_usage_lines(lines, align_defs)

    if args.fast_greedy:
        regions = RegionAllocator().allocate(intervals)
    else:
        # Search for the true minimum footprint, bounded by a time budget
        # so the tool always finishes; it can never do worse than the
        # greedy heuristic alone, since that heuristic seeds the search.
        allocator = OptimalRegionAllocator(time_limit_seconds=args.time_limit, node_limit = 3_000_000)
        regions = allocator.allocate(intervals)
        if not allocator.exact:
            if allocator.stop_reason == "time":
                reason = f"time budget ({args.time_limit:.1f}s) exceeded"
            else:
                reason = f"node budget ({allocator.node_limit:,} search states) exceeded"
            print(f"\nNote: {reason}; result is the best found, not proven optimal.")

    print_regions(regions, intervals)


if __name__ == "__main__":
    main()