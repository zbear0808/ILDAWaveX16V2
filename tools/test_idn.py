#!/usr/bin/env python3
"""Host-side IDN tests: the Python sender tests, then every native C test target.

  python tools/test_idn.py                  # everything
  python tools/test_idn.py --no-native      # Python tests only

Each tests/idn/native/<name>/CMakeLists.txt is configured, built and run with
ctest in build/idn-tests/<name> (Ninja when it is installed; set --generator,
or CC / CFLAGS such as -fsanitize=address,undefined, to change the build).
Nothing here touches the network or hardware, and passing it does not show
that the ESP32-S3 / RP2354 firmware builds (tools/build_all.py) or works on a
board (tools/hwtest.py). The exit status is non-zero if anything fails.
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests", "idn")


def python_tests() -> bool:
    result = unittest.TextTestRunner().run(unittest.defaultTestLoader.discover(TESTS, top_level_dir=TESTS))
    return result.wasSuccessful() and result.testsRun > 0


def run(cmd, cwd=None) -> bool:
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode:
        print(f"$ {' '.join(cmd)}\n{r.stdout}{r.stderr}")
    return r.returncode == 0


def native_test(src: str, build_root: str, generator) -> bool:
    build = os.path.join(build_root, os.path.basename(src))
    # Run ctest in the build directory: before CMake 3.20, ctest ignores --test-dir and passes with no tests.
    return (run(["cmake", "-S", src, "-B", build] + (["-G", generator] if generator else []))
            and run(["cmake", "--build", build])
            and run(["ctest", "--output-on-failure", "--no-tests=error"], cwd=build))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-native", action="store_true", help="run only the Python tests")
    ap.add_argument("--build-dir", default=os.path.join(ROOT, "build", "idn-tests"))
    ap.add_argument("--generator", default="Ninja" if shutil.which("ninja") else None,
                    help='CMake generator, e.g. "MinGW Makefiles" (default: Ninja if installed)')
    args = ap.parse_args()

    results = [("python", python_tests())]
    if not args.no_native:
        for cmake_lists in sorted(glob.glob(os.path.join(TESTS, "native", "*", "CMakeLists.txt"))):
            src = os.path.dirname(cmake_lists)
            results.append((f"native/{os.path.basename(src)}", native_test(src, args.build_dir, args.generator)))
    for name, ok in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
