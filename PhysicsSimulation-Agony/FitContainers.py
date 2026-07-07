from __future__ import annotations

import argparse
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

ALIGN_DEF_RE = re.compile(r"^\s*align\(([^)]+)\)\s*is\s*:(.*)$", re.IGNORECASE)
USAGE_RE = re.compile(r"^\s*(.*?)\s+align\((.+?)\)\s*$", re.IGNORECASE)
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


@dataclass
class Region:
    id: int
    alignment: int
    members: List[BufferInterval] = field(default_factory=list)

    def can_accept(self, buffer: BufferInterval) -> bool:
        # Regions can dynamically upgrade their alignment, so we only check for lifetime overlaps
        for b in self.members:
            if buffer.end > b.start and buffer.start < b.end:
                return False
        return True

    def add(self, buffer: BufferInterval) -> None:
        self.members.append(buffer)
        self.alignment = max(self.alignment, buffer.alignment)


def is_power_of_two(x: int) -> bool:
    return x > 0 and (x & (x - 1)) == 0


def parse_alignment_definitions(lines: Iterable[str]) -> Dict[str, List[int]]:
    """Collect all alignment definitions from the file."""
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
    """Parse usage lines and convert them into block-scoped lifetimes."""
    intervals: List[BufferInterval] = []
    stack: List[Tuple[int, str, int, int]] = []  # depth, name, alignment, start
    parsed_count = 0

    for line in lines:
        if ALIGN_DEF_RE.match(line):
            continue

        # Empty lines or comments act as explicit boundaries to close active lifetimes
        if not line.strip() or COMMENT_RE.match(line):
            while stack:
                d, n, a, start = stack.pop()
                intervals.append(
                    BufferInterval(
                        name=n,
                        depth=d,
                        start=start,
                        end=parsed_count,
                        alignment_name=str(a),
                        alignment=a,
                    )
                )
            continue

        m = USAGE_RE.match(line)
        if not m:
            continue

        content = m.group(1)
        align_token = m.group(2).strip()

        # Visual layout column depth is determined by the total leading formatting prefix width
        leading_len = len(content) - len(content.lstrip(" |"))
        depth = leading_len
        name = content[leading_len:].strip()

        if not name:
            raise ValueError(f"Missing buffer name in line: {line!r}")

        alignment = resolve_alignment(align_token, align_defs)

        # Scopes close strictly when the column indentation level decreases
        while stack and stack[-1][0] > depth:
            d, n, a, start = stack.pop()
            intervals.append(
                BufferInterval(
                    name=n,
                    depth=d,
                    start=start,
                    end=parsed_count,
                    alignment_name=str(a),
                    alignment=a,
                )
            )

        stack.append((depth, name, alignment, parsed_count))
        parsed_count += 1

    # Flush out remaining items at EOF
    while stack:
        d, n, a, start = stack.pop()
        intervals.append(
            BufferInterval(
                name=n,
                depth=d,
                start=start,
                end=parsed_count,
                alignment_name=str(a),
                alignment=a,
            )
        )

    intervals.sort(key=lambda x: (x.start, x.depth))
    return intervals


class RegionAllocator:
    """Assign buffers to the smallest number of reusable regions, supporting alignment upgrades."""

    def allocate(self, intervals: List[BufferInterval]) -> List[Region]:
        intervals = sorted(intervals, key=lambda x: (x.start, x.depth))
        regions: List[Region] = []

        for buffer in intervals:
            chosen: Optional[Region] = None
            candidates = [r for r in regions if r.can_accept(buffer)]
            if candidates:
                candidates.sort(key=lambda r: (
                    max(0, buffer.alignment - r.alignment)
                ))
                chosen = candidates[0]

            if chosen is None:
                chosen = Region(id=len(regions), alignment=buffer.alignment)
                regions.append(chosen)

            chosen.add(buffer)

        return regions


def print_intervals(intervals: List[BufferInterval]) -> None:
    print("Parsed intervals:")
    for b in intervals:
        print(
            f"  {b.name:32s}  start={b.start:3d}  end={b.end:3d}  "
            f"indent={b.depth:2d}  align={b.alignment:3d}"
        )


def print_regions(regions: List[Region]) -> None:
    print("\nRegion assignment:")
    for region in regions:
        print(f"  Region {region.id}  effective_align={region.alignment}")
        for b in sorted(region.members, key=lambda x: (x.start, x.end, x.depth, x.name)):
            print(
                f"    {b.name:30s}  live=[{b.start}, {b.end})  align={b.alignment}  depth={b.depth}"
            )
    print(f"\nTotal regions required: {len(regions)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse custom indentation scopes and allocate reusable regions.")
    parser.add_argument("file", type=Path, help="Input text file")
    args = parser.parse_args()

    text = args.file.read_text(encoding="utf-8")
    lines = text.splitlines()

    align_defs = parse_alignment_definitions(lines)
    intervals = parse_usage_lines(lines, align_defs)

    print("Alignment dictionary:")
    for name, vals in align_defs.items():
        print(f"  {name}: {vals} (effective={max(vals)})")

    print_intervals(intervals)

    allocator = RegionAllocator()
    regions = allocator.allocate(intervals)
    print_regions(regions)


if __name__ == "__main__":
    main()