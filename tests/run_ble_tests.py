"""Run real BLE framing/scheduler code on the host with asynchronous fake GAP."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / ".pio" / "host-ble-tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    args = parser.parse_args()
    cxx = shutil.which(args.cxx)
    if not cxx:
        parser.error("G++ must be on PATH, or specify --cxx.")
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("ble_legacy_tests.exe" if os.name == "nt" else "ble_legacy_tests")
    sources = ["RidBleBroadcaster.cpp", "RidLocationAgeMonitor.cpp", "RidBasicIdEncoder.cpp", "RidOperatorIdEncoder.cpp",
               "RidLocationMapper.cpp", "RidLocationEncoder.cpp"]
    subprocess.run([cxx, "-std=c++11", "-O2", "-Wall", "-Wextra", "-pedantic",
                    "-I", str(ROOT / "include"),
                    str(ROOT / "tests" / "ble_legacy_tests.cpp"),
                    *[str(ROOT / "src" / name) for name in sources],
                    "-o", str(exe)], check=True, timeout=120)
    result = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True, timeout=30)
    output = result.stdout + result.stderr
    (BUILD / "results.txt").write_text(output, encoding="utf-8")
    print(output, end="")
    return result.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: host compilation/execution: {error}", file=sys.stderr)
        sys.exit(1)
