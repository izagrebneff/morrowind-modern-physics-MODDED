#!/usr/bin/env python3
"""Fast conservative OpenMW integration preflight.

Catch absent relative includes and cross-fork interface mismatches before
running expensive native C++ builds.  This is NOT a complete C++ parser.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OPENMW = ROOT / "apps" / "openmw"
CONTRACTS = {
    "world": OPENMW / "mwbase" / "world.hpp",
    "physics": OPENMW / "mwphysics" / "physicssystem.hpp",
    "rendering": OPENMW / "mwrender" / "renderingmanager.hpp",
}
CALLS = {
    "world": re.compile(r"\bgetWorld\s*\(\s*\)\s*->\s*(\w+)\s*\("),
    "physics": re.compile(r"\bmPhysics\s*->\s*(\w+)\s*\("),
    "rendering": re.compile(r"\bmRendering\s*->\s*(\w+)\s*\("),
}
COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.DOTALL)
NAME = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
INCLUDE = re.compile(r'^\s*#\s*include\s*"(\.\.?/[^"]+)"', re.MULTILINE)


def without_comments(source: str) -> str:
    return COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), source)


def line_number(source: str, offset: int) -> int:
    return source.count("\n", 0, offset) + 1


def main() -> int:
    errors: list[str] = []
    declarations: dict[str, set[str]] = {}
    for name, path in CONTRACTS.items():
        if not path.is_file():
            errors.append(f"{path.relative_to(ROOT)}: missing interface header")
            continue
        declarations[name] = set(NAME.findall(without_comments(path.read_text(encoding="utf-8"))))

    for path in sorted(OPENMW.rglob("*")):
        if path.suffix not in (".hpp", ".h", ".cpp", ".cc") or not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        source = without_comments(text)
        relpath = path.relative_to(ROOT)

        for match in INCLUDE.finditer(source):
            include = match.group(1)
            if not (path.parent / include).is_file():
                errors.append(f"{relpath}:{line_number(source, match.start())}: missing include {include}")

        # Only unambiguous receivers: generic variables named 'world' are skipped.
        owners: list[str] = ["world"]
        if relpath.as_posix() in ("apps/openmw/mwworld/worldimp.cpp",
                                  "apps/openmw/mwworld/scene.cpp"):
            owners += ["physics", "rendering"]
        for owner in owners:
            if owner not in declarations:
                continue
            for match in CALLS[owner].finditer(source):
                method = match.group(1)
                if method not in declarations[owner]:
                    errors.append(f"{relpath}:{line_number(source, match.start())}: "
                                  f"{owner} method {method}() absent from "
                                  f"{CONTRACTS[owner].relative_to(ROOT)}")

    if errors:
        print(f"OpenMW port preflight: {len(errors)} possible incompatibility(s):")
        for error in errors[:120]:
            print("  " + error)
        if len(errors) > 120:
            print(f"  ... and {len(errors) - 120} more")
        print("Heuristic check: review false positives before changing code.")
        return 1
    print("OpenMW port preflight OK: checked includes and public interfaces agree.")
    print("This does NOT guarantee C++ compilation or correct runtime physics.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
