--[[
    Use GUIDED mode to move forward at a constant speed while following the terrain.
    Maintain a sticky rangefinder target across all modes.

    The rangefinder target...
    * starts as nil
    * gets set the first time you enter GUIDED or SURFTRAK mode
    * is rounded to the nearest 10 cm
    * can be adjusted up/down in 10 cm increments via joystick buttons
    * is sticky across all modes
    * is reset to nil if the sub is disarmed

    Pilot controls:
    * Switch modes as necessary
    * Change heading using the joystick yaw
    * Increment / decrement altitude using joystick buttons
    * Increment / decrement speed using joystick buttons

    Example joystick button settings:
    param set BTN0_FUNCTION    1.0         # shift
    param set BTN2_SFUNCTION   11          # shift-X            GUIDED mode
    param set BTN3_SFUNCTION   13          # shift-Y            SURFTRAK mode
    param set BTN11_SFUNCTION  108         # shift-dpad-up      script_1: increment rf_target by 10 cm
    param set BTN12_SFUNCTION  109         # shift-dpad-down    script_2: decrement rf_target by 10 cm
    param set BTN13_SFUNCTION  110         # shift-dpad-left    script_3: decrement speed by 5 cm/s
    param set BTN14_SFUNCTION  111         # shift-dpad-right   script_4: increment speed by 5 cm/s

    There are several custom GAT parameters:
    * GAT_SPD sets the forward speed during GUIDED mode
    * GAT_SPD_INC sets the speed increment

    Custom parameters are a bit different than normal parameters:
    * This script sets the default values; these default values are _not_ stored in the eeprom.
    * This script uses "set()" (vs "set_and_save()") to modify the values in memory; these changes are not stored.
    * The pilot can set these in the GCS to specific values; these changes _will_ be stored in the eeprom.
    * If you use "--wipe" in sim_vehicle.py, you can't use "--add-param-file" to set these parameters.
]]--

local PARAM_TABLE_KEY = 96
local PARAM_TABLE_PREFIX = "GAT_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), "transect.lua: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "SPD", 0.2), "transect.lua: could not add SPD param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "SPD_INC", 0.05), "transect.lua: could not add SPD_INC param")

local surftrak_depth_p = Parameter("SURFTRAK_DEPTH")
local gat_spd_p = Parameter("GAT_SPD")
local gat_spd_inc_p = Parameter("GAT_SPD_INC")
local wp_spd_p = Parameter("WP_SPD")

if not surftrak_depth_p or not gat_spd_p or not gat_spd_inc_p or not wp_spd_p then
  gcs:send_text(3, "transect.lua: parameters missing, exit")
  return
end

local RUN_HZ = 20               -- update frequency (20 Hz)
local MAX_DT = 0.5              -- cap dt to avoid large integration step on stall

local GUIDED_MODE_NUM = 4       -- sub GUIDED mode number
local SURFTRAK_MODE_NUM = 21    -- sub SURFTRAK mode number
local ROTATION_PITCH_270 = 25   -- downward-facing rangefinder orientation

-- Configuration constants for PosVelAccel controller
local ACC_XY = 0.5              -- horizontal acceleration limit in m/s^2
local P_GAIN_Z = 1.0            -- proportional gain for vertical terrain following
local MAX_VEL_Z = 2.0           -- maximum vertical velocity correction in m/s

-- Pilot controls
local SPEED_INC_MS = 0.05       -- increment/decrement GAT_SPD by 0.05 m/s
local SPEED_MIN_MS = 0.1
local SPEED_MAX_MS = 1.0        -- fallback upper limit if WP_SPD is unavailable

local RF_TARGET_INC = 0.1       -- increment/decrement rf_target by 0.1 m
local RF_TARGET_MIN = 0.5
local RF_TARGET_MAX = 50.0

local BTN_INC_RF_TARGET = 1     -- Script button 1 (Function 108)
local BTN_DEC_RF_TARGET = 2     -- Script button 2 (Function 109)
local BTN_DEC_SPEED = 3         -- Script button 3 (Function 110)
local BTN_INC_SPEED = 4         -- Script button 4 (Function 111)

-- Controller state variables
local prev_mode                 -- track vehicle mode transitions
local rf_target                 -- sticky rangefinder target in meters
local guided_active = false
local last_time_ms = millis()
local yaw_target_rad = 0
local pos_target = Vector3f()   -- 3D position target in m (NED)
local vel_target = Vector3f()   -- 3D velocity target in m/s (NED)
local acc_target = Vector3f()   -- 3D acceleration target in m/s^2 (NED)

local function clamp(val, min_val, max_val)
  if val < min_val then return min_val end
  if val > max_val then return max_val end
  return val
end

local function get_max_speed()
  return (wp_spd_p and wp_spd_p:get()) or SPEED_MAX_MS
end

-- Clamp initial GAT_SPD if it exceeds WP_SPD
if gat_spd_p:get() > get_max_speed() then
  gat_spd_p:set(get_max_speed())
end

-- TODO why use both data sources? can't we trust ahrs?
local function above_surftrak_depth()
  -- If sub is shallower than SURFTRAK_DEPTH (default -50 cm, i.e. 0.5m depth), return true
  local st_depth = surftrak_depth_p:get()
  local min_depth_m = math.abs(st_depth) * 0.01

  local pos = ahrs:get_relative_position_NED_origin()
  if pos then
    -- pos:z() is positive down (depth in meters)
    return pos:z() < min_depth_m
  end

  local alt_m = baro:get_altitude()
  if alt_m then
    -- alt_m is negative underwater
    return -alt_m < min_depth_m
  end

  return false
end

-- Set rangefinder target, rounded to nearest 10cm
local function set_rf_target(proposed_rf_target)
  rf_target = clamp(math.floor(proposed_rf_target * 10.0 + 0.5) / 10.0, RF_TARGET_MIN, RF_TARGET_MAX)
  gcs:send_text(6, string.format("transect.lua: set rangefinder target to %.2f m", rf_target))
  gcs:send_named_float("RFTarget", rf_target)
end

local function respond_to_joystick_buttons()
  local count = {}
  for i = 1, 4 do
    count[i] = sub:get_and_clear_button_count(i)
  end

  -- Increment or decrement GAT_SPD (clamped to WP_SPD)
  local net_speed_inc = count[BTN_INC_SPEED] - count[BTN_DEC_SPEED]
  if net_speed_inc ~= 0 then
    local inc = (gat_spd_inc_p and gat_spd_inc_p:get()) or SPEED_INC_MS
    local max_spd = get_max_speed()
    local new_spd = clamp(gat_spd_p:get() + net_speed_inc * inc, SPEED_MIN_MS, max_spd)
    gat_spd_p:set(new_spd)
    gcs:send_text(6, string.format("transect.lua: change GAT_SPD to %.2f ms", gat_spd_p:get()))
  end

  -- Increment or decrement rf_target
  local net_rf_inc = count[BTN_INC_RF_TARGET] - count[BTN_DEC_RF_TARGET]
  if net_rf_inc ~= 0 and rf_target ~= nil then
    set_rf_target(rf_target + net_rf_inc * RF_TARGET_INC)
    if vehicle:get_mode() == SURFTRAK_MODE_NUM then
      sub:set_rangefinder_target_cm(rf_target * 100.0)
    end
  end
end

local function reset_guided_controller()
  guided_active = false
  -- Clear targets and ensure AC_PosControl offset is zeroed
  pos_target:x(0)
  pos_target:y(0)
  pos_target:z(0)
  vel_target:x(0)
  vel_target:y(0)
  vel_target:z(0)
  acc_target:x(0)
  acc_target:y(0)
  acc_target:z(0)
  poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
end

local function update_guided_mode(rf_reading, dt)
  local current_pos = ahrs:get_relative_position_NED_origin()
  local current_vel = ahrs:get_velocity_NED()

  if not guided_active then
    -- Check prerequisites
    if not current_pos or not current_vel then
      if prev_mode ~= GUIDED_MODE_NUM then
        gcs:send_text(4, "transect.lua: waiting for EKF relative position")
      end
      return
    end

    if rf_reading == nil then
      if prev_mode ~= GUIDED_MODE_NUM then
        gcs:send_text(4, "transect.lua: waiting for rangefinder reading")
      end
      return
    end

    -- TODO make sure we aren't spamming the pilot
    if above_surftrak_depth() then
      if prev_mode ~= GUIDED_MODE_NUM then
        gcs:send_text(4, "transect.lua: dive below SURFTRAK depth to engage script")
      end
      return
    end

    -- Set sticky target if not set yet
    if rf_target == nil then
      set_rf_target(rf_reading)
    end

    -- Initialize PVA controller targets
    yaw_target_rad = ahrs:get_yaw_rad()

    pos_target:x(current_pos:x())
    pos_target:y(current_pos:y())
    pos_target:z(current_pos:z())

    vel_target:x(current_vel:x())
    vel_target:y(current_vel:y())
    vel_target:z(current_vel:z())

    acc_target:x(0)
    acc_target:y(0)
    acc_target:z(0)

    -- Ensure any previous offset in poscontrol is cleared
    poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())

    guided_active = true
    -- TODO also show speed
    gcs:send_text(6, string.format("transect.lua: GUIDED active, target HAGL %.2fm", rf_target))
  end

  -- Pilot yaw steering: inspect yaw channel input
  local yaw_channel_num = param:get('RCMAP_YAW') or 4
  local yaw_chan = rc:get_channel(yaw_channel_num)
  local yaw_input = yaw_chan and yaw_chan:norm_input_dz() or 0

  local gyro = ahrs:get_gyro()
  local yaw_rate_rads = gyro and gyro:z() or 0

  -- If the yaw stick is deflected or sub is actively turning, follow heading
  if yaw_input ~= 0 or math.abs(yaw_rate_rads) >= 0.05 then
    yaw_target_rad = ahrs:get_yaw_rad()
  end

  -- Desired forward speed (clamped to WP_SPD)
  local speed_ms = clamp(gat_spd_p:get(), SPEED_MIN_MS, get_max_speed())

  -- Desired horizontal velocity in NE frame
  local vel_desired_ne = Vector2f()
  vel_desired_ne:x(math.cos(yaw_target_rad) * speed_ms)
  vel_desired_ne:y(math.sin(yaw_target_rad) * speed_ms)

  -- Ramp horizontal velocity according to ACC_XY
  local vel_diff_ne = Vector2f()
  vel_diff_ne:x(vel_desired_ne:x() - vel_target:x())
  vel_diff_ne:y(vel_desired_ne:y() - vel_target:y())

  local vel_diff_len = vel_diff_ne:length()
  local step_max = ACC_XY * dt
  if vel_diff_len > step_max then
    local scale = step_max / vel_diff_len
    vel_diff_ne:x(vel_diff_ne:x() * scale)
    vel_diff_ne:y(vel_diff_ne:y() * scale)
  end

  local vel_target_old_ne = Vector2f()
  vel_target_old_ne:x(vel_target:x())
  vel_target_old_ne:y(vel_target:y())

  -- Update horizontal targets
  vel_target:x(vel_target:x() + vel_diff_ne:x())
  vel_target:y(vel_target:y() + vel_diff_ne:y())

  acc_target:x(vel_diff_ne:x() / dt)
  acc_target:y(vel_diff_ne:y() / dt)

  pos_target:x(pos_target:x() + (vel_target_old_ne:x() + vel_target:x()) * 0.5 * dt)
  pos_target:y(pos_target:y() + (vel_target_old_ne:y() + vel_target:y()) * 0.5 * dt)

  -- Vertical terrain following:
  local hagl_current = rf_reading or rf_target
  local hagl_error = rf_target - hagl_current

  -- In NED frame, negative Z is upwards (shallower depth).
  -- If hagl_current < rf_target (too close to bottom) -> hagl_error > 0 -> vel_desired_z < 0 (up)
  local vel_desired_z = clamp(-1.0 * (hagl_error * P_GAIN_Z), -MAX_VEL_Z, MAX_VEL_Z)
  local vel_target_old_z = vel_target:z()
  vel_target:z(vel_desired_z)
  acc_target:z((vel_target:z() - vel_target_old_z) / dt)
  pos_target:z(pos_target:z() + (vel_target_old_z + vel_target:z()) * 0.5 * dt)

  -- Enforce SURFTRAK_DEPTH minimum depth ceiling
  local st_depth = surftrak_depth_p:get()
  local min_depth_m = math.abs(st_depth) * 0.01
  if pos_target:z() < min_depth_m then
    pos_target:z(min_depth_m)
    if vel_target:z() < 0 then
      vel_target:z(0)
    end
  end

  -- Send targets to vehicle position controllers
  vehicle:set_target_posvelaccel_NED(pos_target, vel_target, acc_target, false, 0, false, 0, false)

  gcs:send_named_float("RFTarget", rf_target)
end

local function update_surftrak_mode(rf_reading)
  if above_surftrak_depth() then
    return
  end

  -- Get current SURFTRAK target
  local sub_target_cm = sub:get_rangefinder_target_cm()

  if rf_target == nil then
    if sub_target_cm and sub_target_cm > 0 then
      set_rf_target(sub_target_cm * 0.01)
    elseif rf_reading then
      set_rf_target(rf_reading)
    end
  else
    -- If we just switched into SURFTRAK from another mode (e.g. GUIDED),
    -- push our sticky target into ArduSub's SURFTRAK controller.
    if prev_mode ~= SURFTRAK_MODE_NUM then
      local target_cm = rf_target * 100.0
      if sub_target_cm == nil or math.abs(target_cm - sub_target_cm) > 0.5 then
        sub:set_rangefinder_target_cm(target_cm)
      end
    elseif sub_target_cm and sub_target_cm > 0 then
      -- In steady SURFTRAK, if pilot changed the target via throttle stick,
      -- sync our sticky target so it carries over to GUIDED mode.
      local current_target_m = math.floor(sub_target_cm * 0.01 * 10.0 + 0.5) / 10.0
      if math.abs(current_target_m - rf_target) >= 0.09 then
        set_rf_target(current_target_m)
      end
    end
  end

  gcs:send_named_float("RFTarget", rf_target or 0)
end

local function log_transect_state(current_mode, rf_reading)
  local current_pos = ahrs:get_relative_position_NED_origin()
  local depth_m = current_pos and current_pos:z() or 0
  local sub_target_cm = sub:get_rangefinder_target_cm()
  local st_target_m = (sub_target_cm and sub_target_cm > 0) and (sub_target_cm * 0.01) or -1
  local speed_ms = (gat_spd_p and gat_spd_p:get()) or 0
  local targ_z = guided_active and pos_target:z() or -1

  -- TODO bug: there is no rf_target here; rename st_target (sticky target?) to rf_target
  -- TODO: throughout code, use unit suffixes consistently
  logger:write("TRNS",
    "Mode,RFTarg,RFRead,STTarg,Depth,TargZ,Head,Spd",
    "Bfffffff",
    current_mode,
    rf_target or -1,
    rf_reading or -1,
    st_target_m,
    depth_m,
    targ_z,
    ahrs:get_yaw_rad() or 0,
    speed_ms
  )
end

local function update()
  -- Calculate loop delta time
  local tnow = millis()
  local dt = (tnow - last_time_ms):tofloat() / 1000.0
  if dt <= 0 then
    return update, 1000 / RUN_HZ
  end
  if dt > MAX_DT then
    dt = MAX_DT
  end
  last_time_ms = tnow

  respond_to_joystick_buttons()

  -- Handle disarmed state
  if not arming:is_armed() then
    if rf_target ~= nil then
      rf_target = nil
      gcs:send_text(6, "transect.lua: forget rangefinder target")
    end
    if guided_active then
      reset_guided_controller()
      gcs:send_text(6, "transect.lua: GUIDED not active")
    end
    return update, 1000 / RUN_HZ
  end

  -- Read rangefinder
  local rf_reading
  if rangefinder:has_data_orient(ROTATION_PITCH_270)
      and rangefinder:status_orient(ROTATION_PITCH_270) == 4 then
    rf_reading = rangefinder:distance_orient(ROTATION_PITCH_270)
  elseif sub:rangefinder_alt_ok() then
    rf_reading = rangefinder:distance_orient(ROTATION_PITCH_270)
  end

  local current_mode = vehicle:get_mode()

  if current_mode == GUIDED_MODE_NUM then
    update_guided_mode(rf_reading, dt)
  else
    if guided_active then
      reset_guided_controller()
      gcs:send_text(6, "transect.lua: GUIDED not active")
    end

    if current_mode == SURFTRAK_MODE_NUM then
      update_surftrak_mode(rf_reading)
    end
  end

  log_transect_state(current_mode, rf_reading)

  prev_mode = current_mode
  return update, 1000 / RUN_HZ
end

gcs:send_text(6, "transect.lua loaded")
return update, 1000
