# guided_above_terrain

Use a Lua script and ArduSub GUIDED mode to implement a custom flight mode that moves the ROV forward at a constant speed while maintaining a constant distance above the seafloor. Requires ArduSub 4.7.2.

## Overview

The [Seattle Aquarium CCR team](https://github.com/Seattle-Aquarium/Coastal_Climate_Resilience) uses ROVs to gather images of the seafloor for scientific analysis. The ROVs are run in long transects, where the goal is to move at a slow, constant speed 1 meter above the seafloor to allow the camera(s) to capture iamges at fixed rates. This requires careful piloting when the seafloor is sloping or the transect runs through a kelp forest.

ArduSub SURFTRAK mode uses down-facing sonar to maintain a constant distance above the seafloor. This partial automation makes it easier to get good, consistent imagery.

However, with SURFTRAK the pilot still has to maintain a constant forward speed during the transect, which can consume a lot of attention. Strafing currents can make it even more difficult to maintain a good track.

The next step toward automation is to use ArduSub's GUIDED mode to:
* maintain a constant distance above the seafloor,
* move the ROV forward at a constant speed automatically,
* resist strafing currents,
* and still allow the pilot to control the heading.

We need 3 things to provide this automation:
1. A sensor that can provided precise local positioning, such as the [Water Linked A50 DVL](https://www.waterlinked.com/shop/dvl-a50-1248).
2. Support for PVA control (position, velocity, acceleration) in ArduSub GUIDED mode.
3. Lua bindings and a Lua script that uses GUIDED PVA mode to implement this new, custom mode.

ArduSub 4.7.2 adds [GUIDED PVA support, Lua bindings and an example script](https://github.com/ArduPilot/ardupilot/pull/33609).

This repo introduces `transect.lua`, a custom Lua script tuned for Seattle Aquarium operations.

## transect.lua

The `transect.lua` script extends the ArduSub example script by implemeting a "sticky" rangefinder target. The sticky target works in both SURFTRAK and GUIDED modes and is maintained while using other modes, such as ALT_HOLD and STABILIZE. This allows the pilot to set the sticky target once and use it for the entire transect, switching modes as necessary to maintain course and avoid obstacles. The stick rangefinder target is reset when the ROV is disarmed.

### Typical Operation

* Use STABILIZE to dive to the transect starting point.
* Engage SURFTRAK mode. The `transect.lua` script captures the current rangefinder reading, rounds it to the nearest 10 cm, and stores it as the sticky target.
* Adjust the sticky target up/down in 10 cm increments using joystick buttons.
* Adjust cruise speed up/down in 5 cm/s increments using joystick buttons.
* Point in the direction of travel and engage GUIDED mode.
* Use the yaw stick to follow the target track or steer around obstables.
* Switch to SURFTRAK, DEPTH_HOLD, or STABILIZE mode as necessary to manually negotiate obstacles. The sticky target is preserved and reapplied when returning to SURFTRAK or GUIDED.
* Disarm the vehicle to forget (reset) the sticky target.

### SURFTRAK Behavior

Note that `transect.lua` changes how SURFTRAK works:
* Default SURFTRAK behavior is based on ALT_HOLD. The pilot can use the joystick to ascend or descend, and when the pilot lets go of the stick the current *depth* is maintained and the rangefinder target is adjusted if needed.
* With `transect.lua` the pilot can still use the joystick to ascend or descend while in SURFTRAK, but when the pilot lets go of the stick the **sticky target is re-applied**, and the ROV may start ascending or descending back to the target distance.

This makes SURFTRAK more useful for long transects where the primary goal is to maintain the target distance.

### GUIDED Behavior

#### Floor Clearance

If the terrain rises faster than the ROV can clear it, the ROV will slow down and allow the vertical controller to catch up.

If the ROV falls below a minimum height, the ROV will stop.

### Key Parameters

* **GAT_SPD**: forward speed in GUIDED mode, defaults to 0.2 m/s -- **custom parameter**
* **GAT_SPD_INC**: forward speed increment, defaults to 0.05 m/s -- **custom parameter**
* **GAT_RF_INC**: rangefinder target increment, defaults to 0.05 m -- **custom parameter**
* **GAT_CLR_SLOW**: minimum clearance required for full speed, defaults to 0.65 m -- **custom parameter**
* **GAT_CLR_MIN**: minimum clearance for forward motion, defaults to 0.40 m -- **custom parameter**
* **WP_SPD**: maximum forward speed in GUIDED mode, defaults to 1.0 m/s
* **WP_SPD_DN**: maximum downward speed in GUIDED mode, defaults to 1.5 m/s
* **WP_SPD_UP**: maximum upward speed in GUIDED mode, defaults to 2.5 m/s
* **PILOT_SPEED_DN**: maximum downward speed in SURFTRAK mode, defaults to 100 cm/s
* **PILOT_SPEED_UP**: maximum upward speed in SURFTRAK mode, defaults to 100 cm/s

> The GAT custom parameters are registered by `transect.lua`, so you can't modify them until *after* the script starts running. Any changes you make will persist across dives.

> Be sure to check the `WP_SPD_*` and `PILOT_SPEED_*` values on your ROV! The defaults are very aggressive.

### Joystick Button Mapping

Typical joystick assignments:

* Map shift-dpad-up to function 108 (`k_script_1`): Increment rangefinder target (+0.05 m)
* Map shift-dpad-down to function 109 (`k_script_2`): Decrement rangefinder target (-0.05 m)
* Map shift-dpad-left to function 110 (`k_script_3`): Decrement forward speed (-0.05 m/s)
* Map shift-dpad-right to function 111 (`k_script_4`): Increment forward speed (+0.05 m/s)

## SITL Testing

There are 3 parameter files for 3 different SITL environments:

* `transect_test.params` is used by the automated testing framework, see below.
* `transect_sitl.params` can be used with the ArduSub `sim_vehicle.py` wrapper for manual tests.
* `transect_gz.params` can be used with [OSRF Gazebo](https://gazebosim.org/docs/harmonic/getstarted/) and the [bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) worlds and models.

### Automated SITL Testing

An automated test suite is provided in the `tests/` directory to verify `transect.lua` against ArduSub SITL without checking any test code into the upstream ArduPilot repository.

Compile the ArduSub SITL binary compiled with Lua scripting support:

```bash
cd /path/to/ardupilot
source .venv/bin/activate
./waf configure --board sitl --enable-scripting
./waf sub
```

Run the test locally:

```bash
export ARDUPILOT_HOME=/path/to/ardupilot
python tests/run_sitl_test.py
```

The runner automatically locates `ardusub` at `$ARDUPILOT_HOME/build/sitl/bin/ardusub`.

Optional arguments:
* `--speedup <multiplier>`: Run the simulation faster than real-time (default: `10.0`).
* `--test <pattern>`: Run specific test(s) matching substring or glob pattern (e.g. `YawSteering`, `CurrentRejection`, `*yaw*`).
* `--list`: List all available autotests and their descriptions.