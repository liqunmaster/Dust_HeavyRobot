#!/usr/bin/env python3
"""生成静态定长的 Zephyr 主题存储和 PX4 风格的主题头文件。

支持数值和布尔字段、定长数组、嵌套消息、标量默认值、常量以及 # TOPICS。
ORB_MAX_INSTANCES 是本工程扩展，不依赖 PX4 的 EmPy/ROS 构建依赖。
"""
import argparse
import re
from pathlib import Path

TYPES = {"bool": "bool", "float32": "float", "float64": "double"}
TYPES.update({f"{s}{n}": f"{s}{n}_t" for s in ("int", "uint") for n in (8, 16, 32, 64)})
FIELD = re.compile(r"([A-Za-z][A-Za-z0-9_]*)(?:\[([1-9][0-9]*)\])?\s+([A-Za-z_][A-Za-z0-9_]*)(?:\s*=\s*(.+))?$")


def snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def literal(kind, value):
    if kind == "bool":
        if value not in ("true", "false"):
            raise ValueError("bool default must be true or false")
        return value
    if kind.startswith(("uint", "int")):
        number = int(value, 0)
        bits = int(re.search(r"\d+", kind)[0])
        minimum = 0 if kind.startswith("uint") else -(1 << (bits - 1))
        maximum = (1 << bits) - 1 if minimum == 0 else (1 << (bits - 1)) - 1
        if not minimum <= number <= maximum:
            raise ValueError("integer default out of range")
        return str(number)
    if kind in ("float32", "float64"):
        import math
        number = float(value)
        if not math.isfinite(number) or (kind == "float32" and abs(number) > 3.402823466e38):
            raise ValueError("non-finite or overflowing default")
        return repr(number) + ("f" if kind == "float32" else "")
    raise ValueError("nested message/array defaults are not supported")


def generate(files):
    messages = {}
    for path in files:
        if not re.fullmatch(r"[A-Z][A-Za-z0-9]*", path.stem) or path.stem in messages:
            raise ValueError(f"{path}: invalid or duplicate message name")
        fields, topics, names = [], [], set()
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if raw.strip().startswith("# TOPICS "):
                topics.extend(raw.strip()[9:].split())
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            match = FIELD.fullmatch(line)
            if not match:
                raise ValueError(f"{path}:{lineno}: invalid field: {line}")
            kind, count, name, default = match.groups()
            if name in names or not (name.isupper() or re.fullmatch(r"[a-z][a-z0-9_]*", name)):
                raise ValueError(f"{path}:{lineno}: duplicate or invalid field name")
            names.add(name)
            if default is not None:
                if count:
                    raise ValueError(f"{path}:{lineno}: array defaults are unsupported")
                default = literal(kind, default)
            if name.isupper() and default is None:
                raise ValueError(f"{path}:{lineno}: constant needs value")
            fields.append((kind, count, name, default))
        if not fields or not any(f[2] == "timestamp" and f[0] == "uint64" and not f[1] for f in fields):
            raise ValueError(f"{path}: requires uint64 timestamp")
        messages[path.stem] = (fields, topics or [snake(path.stem)])

    ordered, visiting = [], set()

    def visit(name):
        if name in visiting:
            raise ValueError(f"cyclic dependency: {name}")
        if name in ordered:
            return
        if name not in messages:
            raise ValueError(f"unknown message type: {name}")
        visiting.add(name)
        for kind, *_ in messages[name][0]:
            if kind not in TYPES:
                visit(kind)
        visiting.remove(name)
        ordered.append(name)

    for name in messages:
        visit(name)
    outputs, all_topics = {}, []
    for name in ordered:
        fields, topics = messages[name]
        basename = snake(name)
        lines = ["// 由 msg/" + name + ".msg 生成，请勿直接编辑。", "#pragma once", "#include <stdint.h>", '#include "uORB.h"']
        for kind in sorted({f[0] for f in fields if f[0] not in TYPES}):
            lines.append(f"#include <uORB/topics/{snake(kind)}.h>")
        lines += ["", f"struct {basename}_s {{"]
        constants = {}
        for kind, count, field, default in fields:
            cpp = TYPES.get(kind, snake(kind) + "_s")
            if field.isupper():
                constants[field] = int(default, 0) if kind.startswith(("int", "uint")) else default
                lines.append(f"\tstatic constexpr {cpp} {field} = {default};")
            else:
                lines.append(f"\t{cpp} {field}" + (f"[{count}]" if count else "") + "{" + (default or "") + "};")
        lines += ["};", ""]
        queue = constants.get("ORB_QUEUE_LENGTH", 1)
        instances = constants.get("ORB_MAX_INSTANCES", 1)
        if not isinstance(queue, int) or queue < 1 or queue > 32 or queue & (queue - 1):
            raise ValueError(f"{name}: queue length must be power of two in [1,32]")
        if not isinstance(instances, int) or not 1 <= instances <= 32:
            raise ValueError(f"{name}: instance count must be in [1,32]")
        for topic in topics:
            if not re.fullmatch(r"[a-z][a-z0-9_]*", topic) or any(t[0] == topic for t in all_topics):
                raise ValueError(f"duplicate or invalid topic: {topic}")
            all_topics.append((topic, basename, queue, instances))
            lines.append(f"extern const orb_metadata __orb_{topic};")
        outputs[f"uORB/topics/{basename}.h"] = "\n".join(lines) + "\n"
    topics_h = ["#pragma once", '#include "uORB.h"', "enum class ORB_ID : uint16_t {"]
    topics_h += [f"\t{t[0]}," for t in all_topics]
    topics_h += ["\tCOUNT,", "\tINVALID = 65535", "};", "const orb_metadata *get_orb_meta(ORB_ID id);"]
    outputs["uORB/topics/uORBTopics.hpp"] = "\n".join(topics_h) + "\n"
    cpp = ["// 自动生成的主题元数据与静态定长存储。", '#include "uORBManager.hpp"']
    cpp += [f"#include <uORB/topics/{snake(n)}.h>" for n in ordered]
    for index, (topic, basename, queue, instances) in enumerate(all_topics):
        cpp += [f'const orb_metadata __orb_{topic}{{"{topic}", sizeof({basename}_s), {index}, {queue}, {instances}}};',
                f"static uint8_t {topic}_data[{instances}][{queue} * sizeof({basename}_s)]{{}};",
                f"static uORB::Node {topic}_nodes[{instances}]{{"]
        cpp += [f"\t{{&__orb_{topic}, {topic}_data[{i}]}}," for i in range(instances)]
        cpp += ["};"]
    cpp += ["namespace uORB {", "TopicStorage topic_storage[] = {"]
    cpp += [f"\t{{&__orb_{t[0]}, {t[0]}_nodes}}," for t in all_topics]
    cpp += ["};", "}", "const orb_metadata *get_orb_meta(ORB_ID id)", "{",
            "\tconst auto index = static_cast<unsigned>(id);",
            "\treturn index < static_cast<unsigned>(ORB_ID::COUNT) ? uORB::topic_storage[index].meta : nullptr;", "}"]
    outputs["uORBTopics.cpp"] = "\n".join(cpp) + "\n"
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("files", type=Path, nargs="+")
    args = parser.parse_args()
    try:
        outputs = generate(args.files)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    for name, content in outputs.items():
        path = args.output_dir / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists() or path.read_text(encoding="utf-8") != content:
            path.write_text(content, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
