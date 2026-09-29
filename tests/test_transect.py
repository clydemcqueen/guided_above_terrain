#!/usr/bin/env python3
"""
Automated SITL tests for transect.lua against ArduSub.
"""

import math
import os
import re

# The test harness calls chdir to live in the ArduPilot directory, so these imports should work
import ardusub
from vehicle_test_suite import NotAchievedException, Test

# Grab some paths relative to this file.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TRANSECT_LUA_PATH = os.path.join(REPO_ROOT, "lua", "transect4.lua")
TRANSECT_TEST_PARAMS_PATH = os.path.join(REPO_ROOT, "params", "transect_test.params")

# Extract true range from STATUSTEXT messages sent by sub_test_synthetic_seafloor.lua
RE_TR_SEARCH = re.compile(r'#TR#\s*([-+]?\d*\.?\d+)')


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
    def __init__(self, *args, bundle=1, **kwargs):
        super().__init__(*args, **kwargs)
        self.bundle = bundle

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

    def setup_transect_test(self, speed=0.5, bundle=None):
        """Install Lua scripts, load test parameters, reboot SITL, and set initial GAT_SPD."""
        if bundle is None:
            bundle = getattr(self, "bundle", 1)
        self.context_collect("STATUSTEXT")

        # Install Lua scripts
        self.progress(f"Installing {TRANSECT_LUA_PATH}...")
        self.install_script(TRANSECT_LUA_PATH, "transect.lua")
        self.context_get().installed_scripts.append("transect.lua")
        self.install_example_script_context("sub_test_synthetic_seafloor.lua")

        # Configure parameters before reboot
        self.progress(f"Loading parameters from {TRANSECT_TEST_PARAMS_PATH}...")
        params = load_param_file(TRANSECT_TEST_PARAMS_PATH)
        params["SCR_USER1"] = bundle
        self.set_parameters(params)

        # Reboot SITL to load Lua scripts and parameters
        self.reboot_sitl()
        self.set_rc_default()
        self.wait_ready_to_arm()

        # Set custom Lua script parameters once transect script is loaded
        gat_params = {"GAT_SPD": speed}
        self.progress(f"Setting Lua script parameters: {gat_params}...")
        self.set_parameters(gat_params)

    def start_transect_guided(self, speed=0.5, seafloor_depth=15, match_distance=2.0, enter_via_surftrak=True):
        """Helper to dive to match distance and engage GUIDED transect mode."""
        target_depth = -seafloor_depth + match_distance
        self.progress(f"Diving to {target_depth}m in ALT_HOLD...")
        self.dive(target_depth, mode="ALT_HOLD")

        if enter_via_surftrak:
            self.progress("Engaging SURFTRAK to capture target...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode(21)
            self.wait_text("transect: SURFTRAK active, target", timeout=10, check_context=True)

        self.progress("Starting GUIDED mode...")
        self.context_clear_collection("STATUSTEXT")
        self.change_mode("GUIDED")
        guided_msg = self.wait_text("transect: GUIDED active", timeout=10, check_context=True)
        match = re.search(r"target ([\d.]+)m", guided_msg)
        if not match:
            raise NotAchievedException(f"Failed to parse active target from: {guided_msg}")
        active_target = float(match.group(1))
        self.progress(f"PVA GUIDED controller active with target: {active_target:.2f}m")
        return active_target

    def wait_reach_true_distance(self, target_distance, delta=0.5, timeout=15):
        """Wait until simulated rangefinder reading reaches target_distance +/- delta."""
        tstart = self.get_sim_time_cached()
        self.progress(f"Waiting to reach distance {target_distance:.2f}m (+/- {delta:.2f}m)...")
        while self.get_sim_time_cached() - tstart < timeout:
            m_true = self.assert_receive_message("STATUSTEXT", timeout=3.0)
            match = RE_TR_SEARCH.search(m_true.text)
            if not match:
                continue
            dist = float(match.group(1))
            if abs(dist - target_distance) <= delta:
                self.progress(f"Reached target distance: {dist:.2f}m")
                return dist
        raise NotAchievedException(f"Failed to reach true distance {target_distance:.2f}m within {timeout}s")

    def StickyTarget(self):
        """Test sticky rangefinder target, joystick adjustments, and terrain following."""
        seafloor_depth = 15
        match_distance = 2.0  # Target HAGL in meters
        speed = 0.5  # Forward speed in m/s
        xy_margin = 0.5  # Allowed horizontal position error in meters

        # Allowed vertical error in meters
        # bundle=1: 0.5 is great
        # bundle=2: 0.7 works most of the time, but it can still occasionally fail
        z_margin = 0.5

        try:
            self.setup_transect_test(speed=speed)
            current_target = self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Test joystick button adjustments in GUIDED mode
            # Inc rf_target via BTN11 (shift-dpad-up -> script_1)
            inc_target = round(current_target + 0.1, 2)
            self.pulse_button_and_wait(
                11,
                "BTN11 (shift-dpad-up: inc rf_target)",
                f"transect: set rangefinder target to {inc_target:.2f} m",
                shift=True,
            )

            # Dec rf_target back via BTN12 (shift-dpad-down -> script_2)
            self.pulse_button_and_wait(
                12,
                "BTN12 (shift-dpad-down: dec rf_target)",
                f"transect: set rangefinder target to {current_target:.2f} m",
                shift=True,
            )

            # Inc speed via BTN14 (shift-dpad-right -> script_4, GAT_SPD_INC is 0.05)
            inc_speed = round(speed + 0.05, 2)
            self.pulse_button_and_wait(
                14,
                "BTN14 (shift-dpad-right: inc speed)",
                f"transect: change GAT_SPD to {inc_speed:.2f} m/s",
                shift=True,
            )

            # Dec speed back via BTN13 (shift-dpad-left -> script_3)
            self.pulse_button_and_wait(
                13,
                "BTN13 (shift-dpad-left: dec speed)",
                f"transect: change GAT_SPD to {speed:.2f} m/s",
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
            self.wait_text("transect: SURFTRAK active, target", timeout=10, check_context=True)

            # Verify sub maintains target distance over synthetic terrain in SURFTRAK mode
            start_loc = self.get_location()
            timeout = 20
            self.progress(f"Verifying terrain following for {timeout}s...")
            self.watch_true_distance_maintained(current_target, delta=z_margin, timeout=timeout)

            # Disarm vehicle and verify target reset
            self.progress("Disarming to test target reset...")
            self.context_clear_collection("STATUSTEXT")
            self.disarm_vehicle()
            self.wait_text("transect: forget rangefinder target", timeout=10, check_context=True)

            self.progress("StickyTarget test PASSED!")

        finally:
            if self.armed():
                self.disarm_vehicle()

    def YawSteering(self):
        """Test pilot yaw steering and heading tracking in GUIDED mode."""
        speed = 0.5
        seafloor_depth = 15
        match_distance = 2.0
        z_margin = 0.5

        try:
            self.setup_transect_test(speed=speed)
            active_target = self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Run along initial heading for 5s
            self.progress("Running along initial heading for 5s...")
            self.delay_sim_time(5, reason="initial heading run")
            initial_heading = self.get_heading()
            self.progress(f"Heading before turn: {initial_heading:.1f} deg")

            # Pilot commands a 90-degree right turn (toward West, ~275 deg)
            target_heading = (initial_heading + 90) % 360
            self.progress(f"Pilot commanding yaw turn to {target_heading:.1f} deg...")
            self.reach_heading_manual(target_heading)

            current_heading = self.get_heading()
            self.progress(f"Turn complete. Current heading: {current_heading:.1f} deg")

            # Record location right after turn
            loc_after_turn = self.get_location()

            # Track transect along new heading for 15s
            run_time = 15
            expected_dist = speed * run_time
            self.progress(f"Tracking transect along new heading for {run_time}s...")
            self.watch_true_distance_maintained(active_target, delta=z_margin, timeout=run_time)

            end_loc = self.get_location()
            dist_travelled = self.get_distance(loc_after_turn, end_loc)
            bearing_travelled = self.get_bearing(loc_after_turn, end_loc)

            self.progress(
                f"Leg after turn: distance {dist_travelled:.2f}m (expected ~{expected_dist:.1f}m), "
                f"bearing {bearing_travelled:.1f} deg (target {target_heading:.1f} deg)"
            )

            if abs(dist_travelled - expected_dist) > 3.0:
                raise NotAchievedException(
                    f"Distance check failed after turn: expected ~{expected_dist:.1f}m, got {dist_travelled:.2f}m"
                )

            bearing_diff = (bearing_travelled - target_heading + 180) % 360 - 180
            if abs(bearing_diff) > 30:
                raise NotAchievedException(
                    f"Bearing check failed: expected ~{target_heading:.1f} deg, "
                    f"traveled along {bearing_travelled:.1f} deg (diff: {bearing_diff:.1f} deg)"
                )

            self.progress("YawSteering test PASSED!")

        finally:
            if self.armed():
                self.disarm_vehicle()

    def CurrentRejection(self):
        """Test strafing current resistance in GUIDED mode."""
        speed = 0.5
        seafloor_depth = 15
        match_distance = 2.0
        z_margin = 0.5
        drift_margin = 0.5  # Allowed cross-track drift in meters
        current_spd = 0.3  # m/s cross-current

        try:
            self.setup_transect_test(speed=speed)

            # Apply cross-current: 0.3 m/s from 90 deg (East), pushing sub West
            self.progress(f"Injecting cross-current: {current_spd} m/s from 90 deg (East)...")
            self.set_parameters({
                "SIM_WIND_SPD": current_spd,
                "SIM_WIND_DIR": 90.0,
                "SIM_WIND_T": 1,
            })

            active_target = self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Allow vehicle to settle into GUIDED mode and wind up integrator under cross-current
            settle_time = 3
            self.progress(f"Settling in GUIDED mode for {settle_time}s under cross-current...")
            self.delay_sim_time(settle_time, reason="settle into GUIDED mode")

            start_loc = self.get_location()
            att = self.assert_receive_message("ATTITUDE", timeout=5)
            planned_heading = (math.degrees(att.yaw) + 360.0) % 360.0
            timeout = 20
            self.progress(
                f"Verifying current rejection during transect for {timeout}s (heading: {planned_heading:.1f} deg)..."
            )
            self.watch_true_distance_maintained(active_target, delta=z_margin, timeout=timeout)

            end_loc = self.get_location()
            total_dist = self.get_distance(start_loc, end_loc)
            bearing_travelled = self.get_bearing(start_loc, end_loc)
            expected_dist = speed * timeout

            # Decompose movement into along-track and cross-track relative to planned heading
            track_error_angle_rad = math.radians((bearing_travelled - planned_heading + 180) % 360 - 180)
            cross_track_drift = abs(total_dist * math.sin(track_error_angle_rad))
            along_track_dist = total_dist * math.cos(track_error_angle_rad)

            self.progress(
                f"Transect under cross-current: along-track={along_track_dist:.2f}m "
                f"(expected ~{expected_dist:.1f}m), cross-track drift={cross_track_drift:.2f}m "
                f"(uncontrolled drift would be ~{current_spd * timeout:.1f}m)"
            )

            # In 20s with 0.3 m/s current, uncontrolled drift would be 6.0 meters.
            # PVA controller must resist lateral drift to within drift_margin.
            if cross_track_drift > drift_margin:
                raise NotAchievedException(
                    f"Current rejection failed: cross-track drift {cross_track_drift:.2f}m exceeded {drift_margin:.1f}m limit"
                )

            if abs(along_track_dist - expected_dist) > 3.0:
                raise NotAchievedException(
                    f"Along-track distance check failed: expected ~{expected_dist:.1f}m, got {along_track_dist:.2f}m"
                )

            self.progress("CurrentRejection test PASSED!")

        finally:
            self.set_parameters({"SIM_WIND_SPD": 0.0})
            if self.armed():
                self.disarm_vehicle()

    def ModeSwitching(self):
        """Test sticky target persistence across STABILIZE and ALT_HOLD."""
        speed = 0.5
        seafloor_depth = 15
        match_distance = 2.0

        try:
            self.setup_transect_test(speed=speed)
            initial_target = self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Switch to STABILIZE (mode 0) for 3 seconds
            self.progress("Switching to STABILIZE to simulate obstacle negotiation...")
            self.change_mode(0)  # STABILIZE
            self.delay_sim_time(3, reason="stabilize check")

            # Switch to ALT_HOLD (mode 2) for 3 seconds
            self.progress("Switching to ALT_HOLD...")
            self.change_mode(2)  # ALT_HOLD
            self.delay_sim_time(3, reason="alt_hold check")

            # Return to GUIDED mode - verify sticky target is preserved
            self.progress("Returning to GUIDED mode...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode("GUIDED")
            guided_msg = self.wait_text("transect: GUIDED active", timeout=10, check_context=True)
            match = re.search(r"target ([\d.]+)m", guided_msg)
            if not match:
                raise NotAchievedException(f"Failed to parse active target from: {guided_msg}")
            target_after_switch = float(match.group(1))
            self.progress(f"GUIDED resumed with target: {target_after_switch:.2f}m")

            if abs(target_after_switch - initial_target) > 0.01:
                raise NotAchievedException(
                    f"Sticky target lost after mode switch! Expected {initial_target:.2f}m, "
                    f"got {target_after_switch:.2f}m"
                )

            # Switch to SURFTRAK - verify target also preserved in ArduSub SURFTRAK
            self.progress("Switching to SURFTRAK...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode(21)
            surftrak_msg = self.wait_text("transect: SURFTRAK active, target", timeout=10, check_context=True)
            match = re.search(r"target ([\d.]+)m", surftrak_msg)
            if not match:
                raise NotAchievedException(f"Failed to parse target from: {surftrak_msg}")
            st_target = float(match.group(1))
            if abs(st_target - initial_target) > 0.01:
                raise NotAchievedException(
                    f"SURFTRAK target lost after switch! Expected {initial_target:.2f}m, got {st_target:.2f}m"
                )

            # Disarm vehicle and verify target reset
            self.progress("Disarming to test target reset...")
            self.context_clear_collection("STATUSTEXT")
            self.disarm_vehicle()
            self.wait_text("transect: forget rangefinder target", timeout=10, check_context=True)

            self.progress("ModeSwitching test PASSED!")

        finally:
            if self.armed():
                self.disarm_vehicle()

    def FloorClearance(self):
        """Test floor proximity collision prevention, speed scaling, and recovery."""
        speed = 0.5
        seafloor_depth = 15
        match_distance = 2.0
        z_margin = 0.5

        try:
            self.setup_transect_test(speed=speed)
            self.set_parameters({"GAT_CLR_MIN": 0.40, "GAT_CLR_SLOW": 0.65})

            active_target = self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Test 1: Low clearance entry and automatic climb recovery
            self.progress("Switching to ALT_HOLD to descend close to floor (~0.35m clearance)...")
            self.change_mode(2)  # ALT_HOLD
            low_depth = -seafloor_depth + 0.35
            self.dive(low_depth, mode="ALT_HOLD")

            self.progress("Re-engaging GUIDED mode at close floor proximity (< GAT_CLR_MIN)...")
            self.context_clear_collection("STATUSTEXT")
            self.change_mode("GUIDED")

            # Verify low clearance warning is triggered
            self.wait_text("transect: low clearance", timeout=10, check_context=True)
            self.progress("Low clearance warning verified!")

            # Wait for vehicle to climb out of low-clearance zone and reach target altitude
            self.wait_reach_true_distance(active_target, delta=z_margin, timeout=15)

            # Verify vehicle maintains target altitude after recovery
            self.progress(f"Verifying vehicle maintains {active_target:.1f}m target altitude...")
            self.watch_true_distance_maintained(active_target, delta=z_margin, timeout=10)

            # Test 2: Verify forward halting when HAGL <= GAT_CLR_MIN
            self.progress("Testing forward halt by setting GAT_CLR_MIN > current HAGL...")
            self.context_clear_collection("STATUSTEXT")
            self.set_parameters({"GAT_CLR_MIN": 2.50, "GAT_CLR_SLOW": 2.80})
            self.wait_text("transect: low clearance", timeout=10, check_context=True)

            halt_start_loc = self.get_location()
            self.delay_sim_time(5, reason="verify forward motion stopped")
            halt_end_loc = self.get_location()
            halt_dist = self.get_distance(halt_start_loc, halt_end_loc)
            self.progress(f"Distance travelled while low clearance active: {halt_dist:.2f}m (limit 0.50m)")
            if halt_dist > 0.50:
                raise NotAchievedException(
                    f"Forward motion was not halted below GAT_CLR_MIN! Travelled {halt_dist:.2f}m in 5s"
                )

            # Test 3: Restore GAT_CLR_MIN and verify forward motion resumes
            self.progress("Restoring GAT_CLR_MIN to 0.40m and verifying forward resumption...")
            self.set_parameters({"GAT_CLR_MIN": 0.40, "GAT_CLR_SLOW": 0.65})
            resume_start_loc = self.get_location()
            self.delay_sim_time(10, reason="verify forward cruise resumed")
            resume_end_loc = self.get_location()
            resumed_dist = self.get_distance(resume_start_loc, resume_end_loc)
            self.progress(f"Distance travelled after restoring GAT_CLR_MIN: {resumed_dist:.2f}m (expected ~5.0m)")
            if resumed_dist < 2.5:
                raise NotAchievedException(
                    f"Forward motion did not properly resume! Travelled only {resumed_dist:.2f}m in 10s"
                )

            self.progress("FloorClearance test PASSED!")

        finally:
            self.set_parameters({"GAT_CLR_MIN": 0.40, "GAT_CLR_SLOW": 0.65})
            if self.armed():
                self.disarm_vehicle()

    def RangefinderDropout(self):
        """Test rangefinder dropout failsafe, station-keeping in GUIDED mode, and recovery."""
        speed = 0.5
        seafloor_depth = 15
        match_distance = 2.0

        try:
            self.setup_transect_test(speed=speed)

            self.start_transect_guided(
                speed=speed,
                seafloor_depth=seafloor_depth,
                match_distance=match_distance,
                enter_via_surftrak=True,
            )

            # Wait a few seconds to establish forward transect
            self.delay_sim_time(3, reason="establish forward transect")

            # Simulate rangefinder dropout by disabling RNGFND1_TYPE
            self.progress("Simulating rangefinder dropout via RNGFND1_TYPE = 0...")
            self.context_clear_collection("STATUSTEXT")
            self.set_parameters({"RNGFND1_TYPE": 0})

            # Verify statustext warning: "transect: rangefinder lost, forward stopped"
            self.wait_text("transect: rangefinder lost, forward stopped", timeout=10, check_context=True)
            self.progress("Rangefinder lost warning verified!")

            # Verify vehicle remains in GUIDED mode
            self.wait_mode("GUIDED")

            # Check that forward motion has halted and station is held
            halt_start_loc = self.get_location()
            self.delay_sim_time(5, reason="verify forward motion stopped and station held")
            halt_end_loc = self.get_location()
            halt_dist = self.get_distance(halt_start_loc, halt_end_loc)
            self.progress(f"Distance travelled while rangefinder lost: {halt_dist:.2f}m (limit 0.40m)")
            if halt_dist > 0.40:
                raise NotAchievedException(
                    f"Forward motion was not halted on rangefinder dropout! Travelled {halt_dist:.2f}m in 5s"
                )

            # Verify vehicle is still in GUIDED mode
            self.wait_mode("GUIDED")

            # Restore rangefinder
            self.progress("Restoring rangefinder via RNGFND1_TYPE = 36...")
            self.context_clear_collection("STATUSTEXT")
            self.set_parameters({"RNGFND1_TYPE": 36})

            # Verify recovery statustext
            self.wait_text("transect: rangefinder recovered, forward resumed", timeout=10, check_context=True)
            self.progress("Rangefinder recovery message verified!")

            # Verify forward transect resumed
            resume_start_loc = self.get_location()
            self.delay_sim_time(10, reason="verify forward cruise resumed")
            resume_end_loc = self.get_location()
            resumed_dist = self.get_distance(resume_start_loc, resume_end_loc)
            self.progress(f"Distance travelled after rangefinder recovery: {resumed_dist:.2f}m (expected ~5.0m)")
            if resumed_dist < 2.5:
                raise NotAchievedException(
                    f"Forward motion did not properly resume after recovery! Travelled only {resumed_dist:.2f}m in 10s"
                )

            self.progress("RangefinderDropout test PASSED!")

        finally:
            self.set_parameters({"RNGFND1_TYPE": 36})
            if self.armed():
                self.disarm_vehicle()

    def tests(self):
        """Return list of all transect autotests."""
        return [
            Test(self.StickyTarget),
            Test(self.YawSteering),
            Test(self.CurrentRejection),
            Test(self.ModeSwitching),
            Test(self.FloorClearance),
            Test(self.RangefinderDropout),
        ]

