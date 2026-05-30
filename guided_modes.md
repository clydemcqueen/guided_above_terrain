# GUIDED Sub-Modes for ArduSub and ArduCopter

| Sub mode | Copter mode | Intended use | Implementation | Rapid target updates? | Terrain following? |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`Guided_WP`** | **`WP`** | **Single waypoint:** External controller sends a single destination. | `WP_Nav` | No. Rapid updates will lead to jerky behavior. | **Sub**: No<br>**Copter**: Yes |
| **`Guided_Velocity`** | **`VelAccel`** | **Velocity control:** External controller provides velocity (and accel in Copter) targets. | `AC_PosControl` | Yes | N/A |
| **`Guided_PosVel`** | **`PosVelAccel`** | **Trajectory following:** External controller provides simultaneous position and velocity (and accel in Copter) targets for precise tracking. | `AC_PosControl` | Yes | **Sub**: Proposed, with independent horizontal and vertical controllers.<br>**Copter**: No |
| **`Guided_Angle`** | **`Angle`** | **Attitude control:** External controller bypasses position controllers entirely to dictate the vehicle's attitude and thrust or climb rate. | `AC_PosControl` | Yes | N/A |
| *(None)* | **`Pos`** | **Continuous position or single waypoint:** Behavior depends on `GUID_OPTIONS.WPNavUsedForPosControl`. | `WP_Nav` or `AC_PosControl` | **Continuous**: Yes<br>**WP**: No | **Copter**: Yes. Horizontal and vertical controllers are coupled. |
| *(None)* | **`Accel`** | **Acceleration control:** External controller rapidly commands vehicle accelerations. | `AC_PosControl` | Yes | N/A |
| *(None)* | **`TakeOff`** | **Autonomous takeoff:** Used to command the vehicle to rise to a set altitude before handing over control. | `_AutoTakeoff` | No | **Copter**: Yes |
