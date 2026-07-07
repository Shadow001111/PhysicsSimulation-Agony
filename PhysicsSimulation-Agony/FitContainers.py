from __future__ import annotations

import argparse
import re
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


@dataclass
class Region:
    id: int
    alignment: int
    size: int = 0
    members: List[BufferInterval] = field(default_factory=list)

    def can_accept(self, buffer: BufferInterval) -> bool:
        for b in self.members:
            if buffer.end > b.start and buffer.start < b.end:
                return False
        return True

    def add(self, buffer: BufferInterval) -> None:
        self.members.append(buffer)
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
    def allocate(self, intervals: List[BufferInterval]) -> List[Region]:
        intervals = sorted(intervals, key=lambda x: (x.start, x.depth))
        regions: List[Region] = []

        for buffer in intervals:
            chosen: Optional[Region] = None
            candidates = [r for r in regions if r.can_accept(buffer)]
            if candidates:
                candidates.sort(key=lambda r: (
                    max(0, buffer.size - r.size),
                    r.size,
                    max(0, buffer.alignment - r.alignment)
                ))
                chosen = candidates[0]

            if chosen is None:
                chosen = Region(id=len(regions), alignment=buffer.alignment, size=buffer.size)
                regions.append(chosen)

            chosen.add(buffer)

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
        print(f"  Region {region.id}  effective_align={region.alignment}  effective_size={region.size}")
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
    args = parser.parse_args()

    text = args.file.read_text(encoding="utf-8")
    lines = text.splitlines()

    align_defs = parse_alignment_definitions(lines)
    intervals = parse_usage_lines(lines, align_defs)

    allocator = RegionAllocator()
    regions = allocator.allocate(intervals)
    
    print_regions(regions, intervals)


if __name__ == "__main__":
    main()