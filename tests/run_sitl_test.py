#!/usr/bin/env python3
"""
CLI runner for transect.lua SITL autotests.

Usage:
    python3 tests/run_sitl_test.py [--speedup 10.0]
"""

import argparse
import os
import sys


def setup_autotest_path():
    """Ensure Tools/autotest is on sys.path and return ARDUPILOT_HOME."""
    home = os.environ.get("ARDUPILOT_HOME")
    if not home:
        sys.exit(
            "Error: ARDUPILOT_HOME environment variable must be set.\n"
            "Please export ARDUPILOT_HOME=/path/to/ardupilot (or 'workon guided_above_terrain')."
        )
    home = os.path.abspath(home)
    autotest_dir = os.path.join(home, "Tools", "autotest")
    if not os.path.isdir(autotest_dir):
        sys.exit(f"Error: ArduPilot autotest directory not found at: {autotest_dir}")
    if autotest_dir not in sys.path:
        sys.path.insert(0, autotest_dir)
    return home


# Ensure Tools/autotest is on sys.path before importing autotest or test modules
ARDUPILOT_HOME = setup_autotest_path()

from test_transect import AutoTestTransect
from vehicle_test_suite import Test


def get_ardusub_binary(ardupilot_home):
    """Resolve the ardusub SITL executable path inside ARDUPILOT_HOME."""
    binary = os.path.join(ardupilot_home, "build", "sitl", "bin", "ardusub")
    if not os.path.isfile(binary):
        sys.exit(f"Error: ArduSub SITL binary not found at: {binary}")
    if not os.access(binary, os.X_OK):
        sys.exit(f"Error: ArduSub SITL binary is not executable: {binary}")
    return binary


def main():
    parser = argparse.ArgumentParser(description="Run transect.lua automated SITL tests against ArduSub.")
    parser.add_argument("--speedup", type=float, default=10.0, help="SITL simulation speedup (default: %(default)s)")
    args = parser.parse_args()

    binary = get_ardusub_binary(ARDUPILOT_HOME)

    print(f"Using ArduPilot root: {ARDUPILOT_HOME}")
    print(f"Using ArduSub binary: {binary}")
    print(f"Simulation speedup:   {args.speedup}")

    # AutoTest expects cwd to be the ArduPilot root
    old_cwd = os.getcwd()
    os.chdir(ARDUPILOT_HOME)

    try:
        tester = AutoTestTransect(binary, speedup=args.speedup, build_opts={})
        test_instance = Test(tester.StickyTarget)
        success = tester.autotest(tests=[test_instance], allow_skips=False)

        if not success:
            print(f"\n>>>> SITL TEST FAILED ({len(tester.fail_list)} failure(s)) <<<<")
            sys.exit(1)
        else:
            print("\n>>>> SITL TEST PASSED <<<<")
            sys.exit(0)
    finally:
        os.chdir(old_cwd)


if __name__ == "__main__":
    main()
