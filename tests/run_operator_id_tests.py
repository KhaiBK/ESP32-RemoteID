"""Test the supplied Operator ID encoder against the pinned OpenDroneID Core C."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / ".pio" / "host-operator-id-tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("CC", "gcc"))
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    args = parser.parse_args()
    cc, cxx = shutil.which(args.cc), shutil.which(args.cxx)
    if not cc or not cxx:
        parser.error("GCC/G++ must be on PATH, or specify --cc and --cxx.")
    BUILD.mkdir(parents=True, exist_ok=True)
    reference = ROOT / "tests" / "reference" / "opendroneid"
    obj = BUILD / "opendroneid.o"
    exe = BUILD / ("operator_id_tests.exe" if os.name == "nt" else "operator_id_tests")
    subprocess.run([cc, "-std=c11", "-O2", "-DODID_DISABLE_PRINTF", "-I", str(reference),
                    "-c", str(reference / "opendroneid.c"), "-o", str(obj)], check=True, timeout=120)
    subprocess.run([cxx, "-std=c++11", "-O2", "-Wall", "-Wextra", "-pedantic",
                    "-I", str(ROOT / "include"), "-I", str(reference),
                    str(ROOT / "tests" / "operator_id_tests.cpp"),
                    str(ROOT / "src" / "RidOperatorIdEncoder.cpp"), str(obj),
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
