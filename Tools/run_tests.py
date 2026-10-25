#!/usr/bin/env python3
"""使用本机 C++17 编译器构建并运行架构回归测试。"""
import argparse
from contextlib import nullcontext
import importlib.util
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=shutil.which("g++") or "D:/mingw64/bin/g++.exe")
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("messages", ROOT / "Tools/msg/generate_messages.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    build = ROOT / "build/host_tests"
    build.mkdir(parents=True, exist_ok=True)
    schema_tests = build / "schema_tests"
    schema_tests.mkdir(exist_ok=True)
    probe = build / "EventProbe.msg"
    probe.write_text("uint64 timestamp\nuint32 value\nuint8 ORB_QUEUE_LENGTH = 4\n", encoding="utf-8")
    files = sorted((ROOT / "msg").glob("*.msg")) + [probe]
    outputs = module.generate(files)
    for name, data in outputs.items():
        path = build / "generated" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data, encoding="utf-8")
    # 嵌套类型和默认值的校验不依赖输入文件顺序。
    with nullcontext(schema_tests) as temporary:
        child = Path(temporary) / "Child.msg"
        parent = Path(temporary) / "Parent.msg"
        child.write_text("uint64 timestamp\nbool valid = true\n", encoding="utf-8")
        parent.write_text("uint64 timestamp\nChild[2] samples\n", encoding="utf-8")
        assert "uORB/topics/parent.h" in module.generate([parent, child])
        child.write_text("uint64 timestamp\nParent parent\n", encoding="utf-8")
        try:
            module.generate([parent, child])
        except ValueError:
            pass
        else:
            raise AssertionError("cyclic schema accepted")
    # 定义错误必须在写入任何生成文件之前报告。
    for body in ("uint64 timestamp\nuint8 value\nuint8 value\n", "uint64 timestamp\nUnknown child\n",
                 "uint64 timestamp\nuint8 value = 256\n", "uint64 timestamp\nuint8 ORB_QUEUE_LENGTH = 3\n"):
        with nullcontext(schema_tests) as temporary:
            path = Path(temporary) / "Invalid.msg"
            path.write_text(body, encoding="utf-8")
            try:
                module.generate([path])
            except ValueError:
                pass
            else:
                raise AssertionError("invalid schema accepted")
    sources = ["platforms/common/uORB/Subscription.cpp", "platforms/common/uORB/uORBManager.cpp",
               "src/modules/manual_control/manual_control.cpp", "src/modules/chassis_control/controller.cpp",
               "src/modules/booster_control/controller.cpp", "src/lib/control/pid.cpp", "src/lib/control/control.cpp",
               "src/modules/mcu_bridge/codec/mcu.cpp", "test/architecture_regression.cpp"]
    executable = build / "architecture_regression.exe"
    command = [args.cxx, "-std=c++17", "-O2", "-pthread", "-static", "-Wall", "-Wextra"]
    include_directories = [ROOT / "test/host", ROOT / "build/host_tests/generated"]
    include_directories += sorted({path.parent for folder in ("src", "platforms", "boards")
                                   for path in (ROOT / folder).rglob("*") if path.suffix in (".h", ".hpp")})
    for include in include_directories:
        command += ["-I", str(include)]
    command += [str(ROOT / source) for source in sources]
    command += [str(build / "generated/uORBTopics.cpp"), "-o", str(executable)]
    subprocess.run(command, check=True, cwd=ROOT)
    subprocess.run([str(executable)], check=True, cwd=ROOT)
    for header in sorted((build / "generated/uORB/topics").glob("*.h")):
        translation_unit = build / "header.cpp"
        translation_unit.write_text(f'#include <uORB/topics/{header.name}>\n', encoding="utf-8")
        subprocess.run([args.cxx, "-std=c++17", "-fsyntax-only", "-I", str(ROOT / "platforms/common/uORB"),
                        "-I", str(build / "generated"), str(translation_unit)], check=True, cwd=ROOT)
    print("PASS: generated headers compile independently")


if __name__ == "__main__":
    main()
