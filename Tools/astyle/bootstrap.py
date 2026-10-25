#!/usr/bin/env python3
"""在工程内构建固定版本的 AStyle，无需全局安装。"""
import argparse
import hashlib
import shutil
import subprocess
import tarfile
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
URL = "https://deb.debian.org/debian/pool/main/a/astyle/astyle_3.1.orig.tar.gz"
SHA256 = "cbcc4cf996294534bb56f025d6f199ebfde81aa4c271ccbd5ee1c1a3192745d7"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=shutil.which("g++") or "D:/mingw64/bin/g++.exe")
    args = parser.parse_args()
    build = ROOT / "build/tools"
    build.mkdir(parents=True, exist_ok=True)
    archive = build / "astyle_3.1.orig.tar.gz"
    if not archive.exists():
        with urlopen(URL, timeout=30) as response:
            archive.write_bytes(response.read())
    if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA256:
        raise ValueError("AStyle source checksum mismatch")
    with tarfile.open(archive) as source:
        source.extractall(build, filter="data")
    sources = sorted((build / "astyle/src").glob("*.cpp"))
    executable = build / "astyle.exe"
    subprocess.run([args.cxx, "-O2", "-std=c++11", "-static", *map(str, sources), "-o", str(executable)], check=True)
    subprocess.run([str(executable), "--version"], check=True)


if __name__ == "__main__":
    main()
