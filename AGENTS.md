# guided_above_terrain

## Overview

`guided_above_terrain` implements custom autonomous transect survey modes and terrain-following capabilities for ArduSub ROVs.
It uses downward-facing rangefinders (or synthetic seafloor simulation in SITL) and sets 3D kinematic setpoints (`pos`, `vel`, `accel`) in ArduSub GUIDED PVA mode.

- **Primary Lua Script:** [`lua/transect4.lua`](lua/transect4.lua)
- **SITL Synthetic Seafloor Script:** `libraries/AP_Scripting/examples/sub_test_synthetic_seafloor.lua` (in ArduPilot)
- **Parameters:** [`params/transect_test.params`](params/transect_test.params) and [`params/transect_sitl.params`](params/transect_sitl.params)
- **Test Suite:** [`tests/test_transect.py`](tests/test_transect.py) and runner [`tests/run_sitl_test.py`](tests/run_sitl_test.py)

---

## Python Virtual Environment (.venv)

Autotests require a Python virtual environment configured with `pymavlink` and ArduPilot autotest dependencies (often available or symlinked as `.venv` in the repository root).

- **CRITICAL RULE:** **NEVER run bare system `python3` or `python`**. The system Python typically lacks `pymavlink` and ArduPilot autotest dependencies.
- **Interpreter Path:**
  - Use the virtual environment interpreter: `./.venv/bin/python` (or activate your virtual environment).
- **Environment Variable:** Autotests require `ARDUPILOT_HOME` to point to your ArduPilot root directory (e.g. `ARDUPILOT_HOME=/path/to/ardupilot`).

---

## Lua Quality & Pre-Commit Hooks

Git pre-commit hooks enforce `luacheck` on all Lua scripts in `lua/`:

- **Run Luacheck:**
  ```bash
  luacheck lua/transect4.lua
  ```
- **Configuration:** [`.luacheckrc`](.luacheckrc) defines ArduPilot global bindings (`ahrs`, `sub`, `poscontrol`, `vehicle`, `param`, `Parameter`, `Vector3f`, `Vector2f`, `logger`, `gcs`, etc.).
- **Common Luacheck Pitfalls to Avoid:**
  - **Unused variables:** Clean up obsolete constants (e.g. `MAX_VEL_Z`).
  - **Upvalue shadowing:** Never declare a local variable inside a function (e.g. `local init_spd = ...`) that shares a name with a file-scoped local declared earlier.
  - **Heap allocations:** Minimize instantiating objects (like `Vector2f()` or `Vector3f()`) in 20 Hz inner loops to prevent garbage collection spikes on vehicle autopilots.

---

## SITL Autotests

Tests are executed with [`tests/run_sitl_test.py`](tests/run_sitl_test.py), which interfaces with ArduSub SITL:

### Running Tests
```bash
# Run a specific test with speedup (recommended: 100x):
ARDUPILOT_HOME=/path/to/ardupilot ./.venv/bin/python tests/run_sitl_test.py --speedup 100 --test <TestName>

# Run full test suite:
ARDUPILOT_HOME=/path/to/ardupilot ./.venv/bin/python tests/run_sitl_test.py --speedup 100

# List available autotests:
ARDUPILOT_HOME=/path/to/ardupilot ./.venv/bin/python tests/run_sitl_test.py --list
```

### Test Development Guidelines
- **`watch_true_distance_maintained` is an altitude hold check, NOT a wait-for-target loop:**
  In `ardusub.py`, `watch_true_distance_maintained` continuously asserts that every received reading is within `target +/- delta`. If the ROV is in the middle of a climb/descent transition (e.g. recovering from low clearance), call `self.wait_reach_true_distance(target, delta)` first to allow the vehicle to reach altitude before asserting distance maintenance.
- **Target Parameter Consistency:**
  `params/transect_test.params` sets `SCR_USER4 = 2.0` (which `sub_test_synthetic_seafloor.lua` assigns to SURFTRAK mode upon entry). Tests entering via SURFTRAK should match this target (`match_distance = 2.0`) unless explicitly testing custom setpoint changes.
- **SITL Reboot Behavior:**
  During `reboot_sitl()`, ArduSub executes `execv` and immediately re-binds TCP port `5760`. Avoid running SITL in environments with rigid sandbox network constraints that linger TCP sockets in `TIME_WAIT`.

---

## Control & Architecture Patterns

1. **Custom GAT Parameters (`PARAM_TABLE_KEY = 96`):**
   - `GAT_SPD` (Param 1): Cruise speed (m/s).
   - `GAT_SPD_INC` (Param 2): Joystick speed increment step (m/s).
   - `GAT_CLR_MIN` (Param 3): Hard stop clearance deck (m). Below this HAGL, forward velocity is zeroed.
   - `GAT_CLR_SLOW` (Param 4): Clearance slowdown threshold (m). Forward velocity scales proportionally between `GAT_CLR_SLOW` and `GAT_CLR_MIN`.
2. **Defensive Parameter Coupling:**
   - Safety checks (`rf_reading <= clr_min`) must never be nested inside or conditional on `clr_slow > clr_min`. If parameters are tuned independently in QGC, the hard stop must always engage.
   - Always clamp `clr_slow = math.max(clr_slow, clr_min)`.
3. **Vertical Velocity Limits (Preventing Windup):**
   - Always bound vertical velocity targets dynamically using `WP_SPD_UP` and `WP_SPD_DN` parameters rather than hardcoded constants. This prevents `pos_target:z` from integrating far ahead of the vehicle's physical thruster capacity.
