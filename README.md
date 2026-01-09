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
* Add support for the ABOVE_TERRAIN frame to GUIDED mode in ArduSub, this is addressed in [this ArduPilot PR](https://github.com/ArduPilot/ardupilot/pull/27767)
* Write a Lua script to use GUIDED to control the forward velocity, this is addressed in [transect3.lua](lua/transect3.lua)

## transect3.lua

As the name suggests, the transect3.lua script is our third iteration of the transect control script.
This version implements a "sticky" rangefinder target that works across both SURFTRAK and GUIDED modes.
Typical operation looks like this:

* Use STABILIZE or DEPTH_HOLD to motor to the transect starting point, and dive to the target depth
* Engage SURFTRAK mode. This will set the rangefinder target to the current rangefinder reading, rounded to the nearest 10 cm
* Adjust the rangefinder target up/down using the joystick
* Point in the direction of travel and start the transect by engaging GUIDED mode
* The ROV will move forward at a constant speed. Adjust the heading to move around obstacles as needed
* Switch to SURFTRAK, DEPTH_HOLD, or STABILIZE mode to move up and over a tricky obstacle. Select GUIDED mode to resume the transect
* The rangefinder target is maintained across all mode transitions. It is reset when the ROV is disarmed

## Test Results

We ran the following tests:
* There is an autotest in the PR
* We ran SITL tests with [these parameters](params/gat_sitl.params)
* We Gazebo tests using [this model](https://github.com/clydemcqueen/bluerov2_gz/blob/main/models/bluerov2_ping/BlueROV2Ping.md) and [these parameters](params/gat_gz_sitl.params)

We also ran several live tests with a BlueROV2 off Pier 59 in Seattle, using these parameters:
~~~
PSC_JERK_Z 8.0          # force KPa = 1.6, KPv = 0.8
PILOT_ACCEL_Z 500
WPNAV_ACCEL_Z 500
SURFTRAK_DEPTH -100     # min surftrak depth 1m
LOG_DISARMED 1
LOG_BITMASK 180222      # Also log PSCx for debugging
WPNAV_SPEED     20      # move forward at 20 cms
WPNAV_SPEED_UP  30      # ascend at 30 cms
WPNAV_SPEED_DN  30      # descend at 30 cms
BTN11_SFUNCTION  108 # inc rf target by 10 cm (up)
BTN12_SFUNCTION  109 # dec rf target by 10 cm (down)
BTN13_SFUNCTION  111 # inc fwd speed by 10 cms (right)
BTN14_SFUNCTION  110 # dec fwd speed by 10 cms (left)
~~~

A trimmed dataflash log for the 7-Jan-2026 test is available [here](logs/00000029_filtered.BIN).

The upshot is that GUIDED mode with the ABOVE_TERRAIN frame is an improvement over SURFTRAK mode.