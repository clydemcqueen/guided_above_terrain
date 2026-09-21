#!/usr/bin/env python3
"""
Automated SITL tests for transect.lua against ArduSub.
"""

import os
import re

# The test harness calls chdir to live in the ArduPilot directory, so these imports should work
import ardusub
from vehicle_test_suite import NotAchievedException

# Grab some paths relative to this file.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TRANSECT_LUA_PATH = os.path.join(REPO_ROOT, "lua", "transect.lua")
TRANSECT_TEST_PARAMS_PATH = os.path.join(REPO_ROOT, "params", "transect_test.params")


def load_param_file(filepath):
    """Load parameter dictionary from a .param or .params file; supports comments."""
    params = {}
    with open(filepath, "r") as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 2:
                param_name = parts[0]
                val_str = parts[1]
                if val_str.lower().startswith("0x"):
                    val = int(val_str, 16)
                else:
                    val = float(val_str)
                params[param_name] = val
    return params


class AutoTestTransect(ardusub.AutoTestSub):
    def pulse_button(self, btn_bit, desc, shift=False):
        """Pulse a joystick button via MANUAL_CONTROL."""
        mask = 1 << btn_bit
        if shift:
            mask |= 1 << 0  # BTN0_FUNCTION is shift
        self.progress(f"Sending {desc} (buttons=0x{mask:x})...")
        self.context_clear_collection("STATUSTEXT")
        self.mav.mav.manual_control_send(self.mav.target_system, 0, 0, 500, 0, mask)
        self.delay_sim_time(0.2, f"{desc} down")
        self.mav.mav.manual_control_send(self.mav.target_system, 0, 0, 500, 0, 0)
        self.delay_sim_time(0.1, f"{desc} up")

    def pulse_button_and_wait(self, btn_bit, desc, expected_text, shift=False, timeout=10):
        """Pulse button and wait for expected statustext."""
        self.pulse_button(btn_bit, desc, shift=shift)
        return self.wait_text(expected_text, timeout=timeout, check_context=True)

    def StickyTarget(self):
        """Test sticky rangefinder target and terrain following."""
        self.context_collect("STATUSTEXT")

        seafloor_depth = 15
        match_distance = 2.0  # Target HAGL in meters
        speed = 0.5  # Forward speed in m/s
        xy_margin = 4.0  # Allowed horizontal position error in meters
        z_margin = 1.5  # Allowed vertical error in meters

        try:
            # Install Lua scripts
            self.progress(f"Installing {TRANSECT_LUA_PATH}...")
            self.install_script(TRANSECT_LUA_PATH, "transect.lua")
            self.install_example_script_context("sub_test_synthetic_seafloor.lua")

            # Configure parameters before reboot
            self.progress(f"Loading parameters from {TRANSECT_TEST_PARAMS_PATH}...")
            params = load_param_file(TRANSECT_TEST_PARAMS_PATH)
            self.set_parameters(params)

            # Reboot SITL to load Lua scripts and parameters
            self.reboot_sitl()
            self.set_rc_default()
            self.wait_ready_to_arm()

            # Set custom Lua script parameters once transect.lua is loaded
            gat_params = {"GAT_SPD": speed}
            self.progress(f"Setting Lua script parameters: {gat_params}...")
            self.set_parameters(gat_params)

            # Arm and dive to target depth (-seafloor_depth + match_distance)
            target_depth = -seafloor_depth + match_distance
            self.progress(f"Diving to {target_depth}m in ALT_HOLD...")
            self.dive(target_depth, mode="ALT_HOLD")

            # Engage SURFTRAK (mode 21)
            self.progress("Start SURFTRAK...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode(21)
            self.wait_text("transect.lua: set rangefinder target to", timeout=10, check_context=True)

            # Switch to GUIDED mode
            self.progress("Start GUIDED mode...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode("GUIDED")
            guided_msg = self.wait_text("transect.lua: GUIDED active", timeout=10, check_context=True)
            match = re.search(r"target HAGL ([\d.]+)m", guided_msg)
            if not match:
                raise NotAchievedException(f"Failed to parse active target from: {guided_msg}")
            current_target = float(match.group(1))
            self.progress(f"PVA GUIDED controller active with target: {current_target:.2f}m")

            # Test joystick button adjustments in GUIDED mode
            # Inc rf_target via BTN11 (shift-dpad-up -> script_1)
            inc_target = round(current_target + 0.1, 2)
            self.pulse_button_and_wait(
                11,
                "BTN11 (shift-dpad-up: inc rf_target)",
                f"transect.lua: set rangefinder target to {inc_target:.2f} m",
                shift=True,
            )

            # Dec rf_target back via BTN12 (shift-dpad-down -> script_2)
            self.pulse_button_and_wait(
                12,
                "BTN12 (shift-dpad-down: dec rf_target)",
                f"transect.lua: set rangefinder target to {current_target:.2f} m",
                shift=True,
            )

            # Inc speed via BTN14 (shift-dpad-right -> script_4, GAT_SPD_INC is 0.05 in transect.lua)
            inc_speed = round(speed + 0.05, 2)
            self.pulse_button_and_wait(
                14,
                "BTN14 (shift-dpad-right: inc speed)",
                f"transect.lua: change GAT_SPD to {inc_speed:.2f} ms",
                shift=True,
            )

            # Dec speed back via BTN13 (shift-dpad-left -> script_3)
            self.pulse_button_and_wait(
                13,
                "BTN13 (shift-dpad-left: dec speed)",
                f"transect.lua: change GAT_SPD to {speed:.2f} ms",
                shift=True,
            )

            # Verify sub maintains target distance over synthetic terrain in GUIDED mode
            start_loc = self.get_location()
            timeout = 20
            self.progress(f"Verifying terrain following for {timeout}s...")
            self.watch_true_distance_maintained(current_target, delta=z_margin, timeout=timeout)

            # Verify forward distance travelled
            end_loc = self.get_location()
            distance_travelled = self.get_distance(start_loc, end_loc)
            expected_distance = speed * timeout
            self.progress(f"Expected forward distance: ~{expected_distance:.1f}m, achieved: {distance_travelled:.2f}m")
            if abs(distance_travelled - expected_distance) > xy_margin:
                raise NotAchievedException(
                    f"Transect failed distance check: expected ~{expected_distance:.1f}m, got {distance_travelled:.2f}m"
                )

            # Switch back to SURFTRAK
            self.progress("Switching back to SURFTRAK...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode(21)
            self.wait_text("transect.lua: GUIDED not active", timeout=10, check_context=True)

            # Verify sub maintains target distance over synthetic terrain in SURFTRAK mode
            start_loc = self.get_location()
            timeout = 20
            self.progress(f"Verifying terrain following for {timeout}s...")
            self.watch_true_distance_maintained(current_target, delta=z_margin, timeout=timeout)

            # Disarm vehicle and verify target reset
            self.progress("Disarming to test target reset...")
            self.context_clear_collection("STATUSTEXT")
            self.disarm_vehicle()
            self.wait_text("transect.lua: forget rangefinder target", timeout=10, check_context=True)

            self.progress("StickyTarget test PASSED!")

        finally:
            if self.armed():
                self.disarm_vehicle()
