#!/usr/bin/env python3
"""使用 PX4 固定的 AStyle 3.1 规则格式化工程自有 C/C++ 代码。"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--astyle", default=shutil.which("astyle") or str(ROOT / "build/tools/astyle.exe"))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    version = subprocess.check_output([args.astyle, "--version"], text=True, stderr=subprocess.STDOUT)
    if "Version 3.1" not in version or "Version 3.1." in version:
        parser.error("AStyle 3.1 is required; see Tools/astyle/README.md")
    files = sorted(p for folder in ("src", "platforms", "boards", "test") for p in (ROOT / folder).rglob("*")
                   if p.suffix in (".c", ".cpp", ".h", ".hpp"))
    changed = []
    for path in files:
        original = path.read_bytes()
        result = subprocess.run([args.astyle, f"--options={ROOT / '.astylerc'}"], input=original,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout
        result = result.replace(b"\r\n", b"\n").rstrip(b"\r\n") + b"\n"
        if result != original:
            changed.append(path.relative_to(ROOT).as_posix())
            if not args.check:
                path.write_bytes(result)
    print(f"{len(files)} files checked; {len(changed)} {'need formatting' if args.check else 'formatted'}")
    if args.check and changed:
        print("\n".join(changed))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
