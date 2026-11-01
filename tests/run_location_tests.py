"""Compile and run host tests; no firmware build, serial access, or network calls."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / ".pio" / "host-location-tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--cc", default=os.environ.get("CC", "gcc"))
    parser.add_argument("--mavlink", type=Path)
    args = parser.parse_args()
    cxx, cc = shutil.which(args.cxx), shutil.which(args.cc)
    if not cxx or not cc:
        parser.error("GCC/G++ must be on PATH, or specify --cc and --cxx.")
    mavlink = args.mavlink
    if mavlink is None:
        candidates = list((ROOT / ".pio" / "libdeps").glob("*/c_library_v2"))
        if not candidates:
            parser.error("Specify --mavlink pointing to the MAVLink C headers.")
        mavlink = candidates[0]
    if not (mavlink / "ardupilotmega" / "mavlink.h").is_file():
        parser.error("MAVLink ardupilotmega/mavlink.h not found.")

    BUILD.mkdir(parents=True, exist_ok=True)
    reference = ROOT / "tests" / "reference" / "opendroneid"
    obj = BUILD / "opendroneid.o"
    exe = BUILD / ("location_vector_tests.exe" if os.name == "nt" else "location_vector_tests")
    subprocess.run([cc, "-std=c11", "-O2", "-DODID_DISABLE_PRINTF",
                    "-I", str(reference), "-c", str(reference / "opendroneid.c"),
                    "-o", str(obj)], check=True, timeout=120)
    subprocess.run([cxx, "-std=c++11", "-O2", "-Wall", "-Wextra",
                    "-Wno-address-of-packed-member", "-I", str(ROOT / "include"),
                    "-I", str(reference), "-I", str(mavlink),
                    str(ROOT / "tests" / "location_vector_tests.cpp"),
                    str(ROOT / "src" / "RidLocationMapper.cpp"),
                    str(ROOT / "src" / "RidLocationEncoder.cpp"),
                    str(ROOT / "src" / "MavlinkAdapter.cpp"), str(obj),
                    "-o", str(exe)], check=True, timeout=120)

    # A separate process makes a direction-normalization hang a bounded failure.
    results = []
    failed = False
    for command, timeout in [([str(exe)], 30), ([str(exe), "--extreme-direction"], 3)]:
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=timeout, cwd=ROOT)
            output = result.stdout + result.stderr
            failed |= result.returncode != 0
        except subprocess.TimeoutExpired:
            output = "FAIL: extreme finite direction did not finish within 3 seconds.\n"
            failed = True
        results.append(output)
        print(output, end="")
    (BUILD / "results.txt").write_text("".join(results), encoding="utf-8")
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: host compilation/execution: {error}", file=sys.stderr)
        sys.exit(1)
