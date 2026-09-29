#!/usr/bin/env python3
"""
CLI runner for transect.lua SITL autotests.

Usage:
    python3 tests/run_sitl_test.py [--speedup 10.0] [--bundle 1] [--test <pattern>] [--list]
"""

import argparse
import fnmatch
import os
import sys


def setup_autotest_path():
    """Ensure Tools/autotest is on sys.path and return ARDUPILOT_HOME."""
    home = os.environ.get("ARDUPILOT_HOME")
    if not home:
        sys.exit("Error: ARDUPILOT_HOME environment variable must be set" )
    home = os.path.abspath(home)
    autotest_dir = os.path.join(home, "Tools", "autotest")
    if not os.path.isdir(autotest_dir):
        sys.exit(f"Error: ArduPilot autotest directory not found at: {autotest_dir}")
    if autotest_dir not in sys.path:
        sys.path.insert(0, autotest_dir)
    return home


def get_ardusub_binary(ardupilot_home):
    """Resolve the ardusub SITL executable path inside ARDUPILOT_HOME."""
    binary = os.path.join(ardupilot_home, "build", "sitl", "bin", "ardusub")
    if not os.path.isfile(binary):
        sys.exit(f"Error: ArduSub SITL binary not found at: {binary}")
    if not os.access(binary, os.X_OK):
        sys.exit(f"Error: ArduSub SITL binary is not executable: {binary}")
    return binary


def main():
    parser = argparse.ArgumentParser(description="Run transect.lua automated SITL tests against ArduSub")
    parser.add_argument("--speedup", type=float, default=10.0, help="SITL simulation speedup (default: %(default)s)")
    parser.add_argument("--bundle", "-b", type=int, default=1, help="Seafloor bundle (default: %(default)s)")
    parser.add_argument("--test", "-t", type=str, default=None, help=("Run specific test(s)"))
    parser.add_argument("--list", "-l", action="store_true", help="List available tests")
    args = parser.parse_args()

    # Ensure Tools/autotest is on sys.path before importing autotest or test modules
    ardupilot_home = setup_autotest_path()
    from test_transect import AutoTestTransect
    from vehicle_test_suite import Test

    binary = get_ardusub_binary(ardupilot_home)

    # Instantiate tester to inspect available tests
    tester = AutoTestTransect(binary, speedup=args.speedup, bundle=args.bundle, build_opts={})
    available_tests = [t if isinstance(t, Test) else Test(t) for t in tester.tests()]

    if args.list:
        print("Available transect autotests:")
        for t in available_tests:
            desc = t.description or "No description"
            print(f"  - {t.name:<18} : {desc}")
        sys.exit(0)

    if args.test:
        pattern = args.test.lower()
        selected_tests = [
            t
            for t in available_tests
            if fnmatch.fnmatch(t.name.lower(), pattern) or pattern in t.name.lower()
        ]
        if not selected_tests:
            print(f"Error: No tests match pattern '{args.test}'")
            print("Available tests:")
            for t in available_tests:
                print(f"  - {t.name}")
            sys.exit(1)
    else:
        selected_tests = available_tests

    print(f"Using ArduPilot root: {ardupilot_home}")
    print(f"Using ArduSub binary: {binary}")
    print(f"Simulation speedup:   {args.speedup}")
    print(f"Seafloor bundle:      {args.bundle}")
    print(f"Running {len(selected_tests)} test(s): {[t.name for t in selected_tests]}")

    # AutoTest expects cwd to be the ArduPilot root
    old_cwd = os.getcwd()
    os.chdir(ardupilot_home)

    try:
        success = tester.autotest(tests=selected_tests, allow_skips=False)

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
