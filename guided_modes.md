# GUIDED Sub-Modes for ArduSub and ArduCopter

| Sub mode | Copter mode | Intended use | Implementation | Rapid target updates? | Terrain following? |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`Guided_WP`** | **`WP`** | **Single waypoint:** External controller sends a single destination. | `WP_Nav` | No. Rapid updates will lead to jerky behavior. | Yes (1) |
| **`Guided_Velocity`** | **`VelAccel`** | **Velocity control:** External controller provides velocity (and accel in Copter) targets. | `AC_PosControl` | Yes | N/A |
| **`Guided_PosVelAccel`** | **`PosVelAccel`** | **Trajectory following:** External controller provides simultaneous position, velocity and acceleration targets for precise tracking. (2) | `AC_PosControl` | Yes | Possible via Lua (3) |
| **`Guided_Angle`** | **`Angle`** | **Attitude control:** External controller bypasses position controllers entirely to dictate the vehicle's attitude and thrust or climb rate. | `AC_PosControl` | Yes | N/A |
| *(None)* | **`Pos`** | **Continuous position or single waypoint:** Behavior depends on `GUID_OPTIONS.WPNavUsedForPosControl`. | `WP_Nav` or `AC_PosControl` | **Continuous**: Yes<br>**WP**: No | **Copter**: Yes. Horizontal and vertical controllers are coupled. |
| *(None)* | **`Accel`** | **Acceleration control:** External controller rapidly commands vehicle accelerations. | `AC_PosControl` | Yes | N/A |
| *(None)* | **`TakeOff`** | **Autonomous takeoff:** Used to command the vehicle to rise to a set altitude before handing over control. | `_AutoTakeoff` | No | **Copter**: Yes |

## Notes

1. Terrain following in Sub via `SET_POSITION_TARGET_GLOBAL_INT` added in [33412](https://github.com/ArduPilot/ardupilot/pull/33412).
2. Acceleration targets in Sub added in [33609](https://github.com/ArduPilot/ardupilot/pull/33609).
3. [Example Lua script](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/guided_above_terrain_posvelaccel_sub.lua)