#!/usr/bin/env python3
"""Generate C++ message structs from Topic/msg/*.msg definitions."""

import argparse
import re
from pathlib import Path


TYPE_MAP = {
    "bool": "bool",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
}
IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
FIELD = re.compile(r"([a-z][a-z0-9]*)(?:\[([1-9][0-9]*)\])?\s+([a-z][a-z0-9_]*)\Z")


def parse_message(path: Path):
    struct_name = None
    fields = []
    field_names = set()

    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            if line.startswith("# @struct "):
                name = line[len("# @struct "):].strip()
                if struct_name is not None or not IDENTIFIER.fullmatch(name):
                    raise ValueError(f"{path}:{line_number}: invalid or duplicate struct name")
                struct_name = name
            elif line.startswith("# @"):
                raise ValueError(f"{path}:{line_number}: unknown directive: {line}")
            continue

        match = FIELD.fullmatch(line)
        if match is None or match.group(1) not in TYPE_MAP:
            raise ValueError(f"{path}:{line_number}: invalid field: {line}")
        source_type, count, name = match.groups()
        if name in field_names:
            raise ValueError(f"{path}:{line_number}: duplicate field: {name}")
        field_names.add(name)
        fields.append((TYPE_MAP[source_type], count, name))

    if struct_name is None or not fields:
        raise ValueError(f"{path}: requires '# @struct Name' and at least one field")
    return struct_name, fields


def render_header(source: Path, struct_name: str, fields) -> str:
    lines = [
        f"// Generated from {source.name}. Do not edit.",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        f"struct {struct_name} {{",
    ]
    for cpp_type, count, name in fields:
        suffix = f"[{count}]" if count else ""
        lines.append(f"    {cpp_type} {name}{suffix};")
    lines.extend(("};", ""))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("files", nargs="+", type=Path)
    args = parser.parse_args()

    messages = []
    names = set()
    stems = set()
    try:
        for path in args.files:
            struct_name, fields = parse_message(path)
            if struct_name in names or path.stem in stems:
                raise ValueError(f"{path}: duplicate message name or filename")
            names.add(struct_name)
            stems.add(path.stem)
            messages.append((path, struct_name, fields))
    except (OSError, ValueError) as error:
        parser.error(str(error))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for path, struct_name, fields in messages:
        output = args.output_dir / f"{path.stem}.hpp"
        content = render_header(path, struct_name, fields)
        if not output.exists() or output.read_text(encoding="utf-8") != content:
            output.write_text(content, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
