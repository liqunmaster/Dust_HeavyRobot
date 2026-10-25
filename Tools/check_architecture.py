#!/usr/bin/env python3
"""检查直接头文件依赖、组件源码注册和低耦合分层边界。"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLS = {"chassis_control", "yaw_control", "pitch_control", "shoot_control", "lift_control", "booster_control"}


def main():
    errors, checked = [], 0
    headers = {}
    for folder in ("src", "platforms", "boards"):
        for header in (ROOT / folder).rglob("*"):
            if header.suffix not in (".h", ".hpp"):
                continue
            if header.name in headers:
                errors.append(f"duplicate header basename: {header.relative_to(ROOT)} and {headers[header.name]}")
            headers[header.name] = header.relative_to(ROOT).as_posix()
    for folder in ("src", "platforms", "boards"):
        for path in (ROOT / folder).rglob("*"):
            if path.suffix not in (".cpp", ".hpp", ".c", ".h"):
                continue
            checked += 1
            relative = path.relative_to(ROOT).as_posix()
            is_lower = relative.startswith(("src/lib/", "src/device/", "platforms/"))
            module = path.relative_to(ROOT / "src/modules").parts[0] if relative.startswith("src/modules/") else None
            for delimiter, include in re.findall(r'^\s*#include\s*([<"])([^>"]+)', path.read_text(encoding="utf-8"), re.M):
                resolved = headers.get(Path(include).name, "")
                if resolved and (delimiter != '"' or "/" in include or "\\" in include):
                    errors.append(f"{relative}: project header must use its short name and quotes: {include}")
                if is_lower and resolved.startswith(("src/modules/", "boards/")):
                    errors.append(f"{relative}: lower layer includes application {include}")
                if module in CONTROLS:
                    if resolved.startswith("src/modules/") and resolved.split("/")[2] != module:
                        errors.append(f"{relative}: control module depends on {include}")
                    if resolved.startswith(("src/device/", "platforms/drivers/dvc_", "boards/", "src/lib/drivers/motor_port/", "src/lib/drivers/canfd_bus/")):
                        errors.append(f"{relative}: control task bypasses physical device service: {include}")
                    if path.name.endswith("_controller.hpp") and (include.startswith(("uORB/", "zephyr/")) or
                            resolved.startswith(("platforms/", "src/lib/drivers/"))):
                        errors.append(f"{relative}: pure controller includes runtime {include}")
                if include.startswith(("modules/", "lib/", "device/")) and not resolved:
                    errors.append(f"{relative}: unresolved project include {include}")
            if not (path.parent / "CMakeLists.txt").is_file():
                errors.append(f"{relative}: source/header directory has no CMakeLists.txt")
    for path in (ROOT / "src").rglob("*.cpp"):
        cmake = path.parent / "CMakeLists.txt"
        if not cmake.exists() or path.name not in cmake.read_text(encoding="utf-8"):
            errors.append(f"{path.relative_to(ROOT)}: source not registered in component CMake")
    print(f"Checked {checked} production files; {len(errors)} dependency errors")
    if errors:
        print("\n".join(errors))
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
