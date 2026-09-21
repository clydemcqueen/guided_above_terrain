# guided_above_terrain

## Overview

The [Seattle Aquarium CCR team](https://github.com/Seattle-Aquarium/Coastal_Climate_Resilience) uses ROVs to gather 
images of the seafloor for scientific analysis. The ROVs are run in relatively shallow water in long transects, where 
the goal is to move at a constant speed 1 meter above the seafloor. This requires careful piloting when the seafloor is
sloping or the transect runs through a kelp forest.

ArduSub [SURFTRAK](https://github.com/clydemcqueen/ardusub_surftrak) mode uses down-facing sonar to maintain a constant
distance above the seafloor. This partial automation makes it easier to get good, consistent imagery.

Even with SURFTRAK the pilot needs to move forward at a constant speed, which can consume a lot of attention.
Strafing currents can make it even more challenging.

The next step toward automation is to use a DVL like the [A50](https://www.waterlinked.com/shop/dvl-a50-1248) and
ArduSub's GUIDED mode to move the ROV forward at a constant speed, while maintaining a constant distance above the
seafloor. This also neatly solves the problem of strafing currents.

There are two tasks:
* Extend ArduSub GUIDED mode to include acceleration targets, and expose the ArduSub position controller API to Lua scripts.
This is addressed in [this ArduPilot PR](https://github.com/ArduPilot/ardupilot/pull/33609).
* Write a Lua script to implement terrain following in GUIDED mode, this is addressed in [transect.lua](lua/transect.lua).

## transect.lua

The `transect.lua` script implements a "sticky" rangefinder target that works across both SURFTRAK and GUIDED modes, using ArduSub's PosVelAccel and position controller Lua bindings.

Typical operation looks like this:
* Use STABILIZE or DEPTH_HOLD to motor to the transect starting point, and dive to the target depth.
* Engage SURFTRAK mode (mode 21). The script captures the rangefinder reading (or current SURFTRAK target) rounded to the nearest 10 cm and stores it as the sticky target.
* Adjust the rangefinder target up/down in 10 cm increments using joystick buttons mapped to script functions 1 and 2 (`k_script_1` / `k_script_2`).
* Adjust cruise speed up/down in 0.1 m/s increments using joystick buttons mapped to script functions 3 and 4 (`k_script_3` / `k_script_4`).
* Point in the direction of travel and engage GUIDED mode (mode 4).
* In GUIDED mode, `transect.lua` executes a 20 Hz PosVelAccel control loop:
  - Ramps horizontal velocity along heading at `ACC_XY = 0.5 m/s²` up to `GAT_SPD` and feeds position, velocity, and acceleration targets to `vehicle:set_target_posvelaccel_NED()`.
  - Computes vertical error relative to the sticky rangefinder target and applies vertical position and velocity offsets to `poscontrol:set_posvelaccel_offset()`.
  - Supports pilot yaw stick steering during the transect.
  - Logs transect state to the DataFlash `GUIT` log table (`RFTarg`, `RFRead`, `Head`, `TargZ`, `OffZ`, `Spd`).
* Switch to SURFTRAK, DEPTH_HOLD, or STABILIZE to manually negotiate obstacles. The sticky target is preserved and reapplied when returning to SURFTRAK or GUIDED.
* The rangefinder target is forgotten and reset when the ROV is disarmed.

### Joystick Button Mapping

Assign the following button functions in your ArduSub parameter configuration (or via QGroundControl / BlueOS):
* **Button -> Function 108 (`k_script_1`)**: Increment rangefinder target (+0.1 m)
* **Button -> Function 109 (`k_script_2`)**: Decrement rangefinder target (-0.1 m)
* **Button -> Function 110 (`k_script_3`)**: Decrement forward speed (`GAT_SPD`, -0.05 m/s)
* **Button -> Function 111 (`k_script_4`)**: Increment forward speed (`GAT_SPD`, +0.05 m/s)

## Automated SITL Testing

An automated test suite is provided in the `tests/` directory to verify `transect.lua` against ArduSub SITL without checking any test code into the upstream ArduPilot repository.

### Prerequisites

* ArduSub SITL binary compiled with Lua scripting support (from PR #33609 or master):
  ```bash
  cd /path/to/ardupilot
  ./waf configure --board sitl --enable-scripting
  ./waf sub
  ```
* Python 3 with `pymavlink` and ArduPilot autotest dependencies installed (or use the Python virtual environment located in your ArduPilot directory).

### Running the Test

Set `ARDUPILOT_HOME` to point to your ArduPilot build directory:

```bash
export ARDUPILOT_HOME=/path/to/ardupilot
python3 tests/run_sitl_test.py
```

The runner automatically locates `ardusub` at `$ARDUPILOT_HOME/build/sitl/bin/ardusub`.

Optional argument:
* `--speedup <multiplier>`: Run simulation faster than real-time (default: `10.0`).
