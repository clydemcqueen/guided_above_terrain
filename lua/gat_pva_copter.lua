-- gat_pva_copter.lua
-- Implements "guided mode above terrain with cruise control"
-- Uses pos-vel-accel bindings to control XY based on yaw.
-- Uses poscontrol:set_posvelaccel_offset to control Z for terrain following.

local RUN_HZ = 20
local DT = 1.0 / RUN_HZ
local GUIDED_MODE = 4 -- Copter GUIDED mode number

-- Configuration
local P_GAIN_Z = 1.0 -- Proportional gain for vertical correction
local MAX_VEL_Z = 2.0 -- max vertical velocity correction in m/s

-- State variables
local active = false
local target_pos_ne = Vector2f()
local target_pos_z = 0 -- Base Z target (EKF origin frame)
local target_hagl_m = 0 -- Target height above ground level
local offset_z = 0 -- Integrated vertical offset

local function update()
    local current_mode = vehicle:get_mode()
    -- Check if we are in GUIDED mode and the activation switch is HIGH
    -- We use Scripting1 (Option 300) to activate the script's behavior
    local script_switch = rc:get_aux_cached(300)

    if current_mode ~= GUIDED_MODE or script_switch ~= 2 then
        if active then
            gcs:send_text(6, "GAT: Deactivated")
        end
        active = false
        return update, 1000 / RUN_HZ
    end

    -- Transition to GUIDED mode
    if not active then
        -- Initialize base targets
        local current_pos = ahrs:get_relative_position_NED_origin()
        if not current_pos then
            gcs:send_text(4, "GAT: Waiting for EKF origin")
            return update, 1000 / RUN_HZ
        end

        -- Check rangefinder
        if not rangefinder:has_data_orient(25) or rangefinder:status_orient(25) ~= 4 then -- 4 is RangeFinder_Good
            gcs:send_text(4, "GAT: Downward rangefinder not healthy")
            return update, 1000 / RUN_HZ
        end

        local current_hagl_m = rangefinder:distance_orient(25)

        target_pos_ne:x(current_pos:x())
        target_pos_ne:y(current_pos:y())
        target_pos_z = current_pos:z()
        target_hagl_m = current_hagl_m
        offset_z = 0

        active = true
        gcs:send_text(6, string.format("GAT: Activated. Target HAGL: %.1fm", target_hagl_m))
    end

    -- Get forward speed
    local wp_spd_cms = param:get("WP_SPD")
    if not wp_spd_cms then
        wp_spd_cms = 500 -- Default to 5m/s
    end
    local cruise_speed_ms = wp_spd_cms * 0.01

    -- XY control
    local ahrs_yaw = ahrs:get_yaw_rad()
    local vel_target_ne = Vector2f()
    vel_target_ne:x(math.cos(ahrs_yaw) * cruise_speed_ms)
    vel_target_ne:y(math.sin(ahrs_yaw) * cruise_speed_ms)

    local accel_target_ne = Vector2f()
    -- AC_PosControl handles velocity smoothing natively, zero accel target is fine

    -- Integrate XY position forward based on velocity target
    target_pos_ne:x(target_pos_ne:x() + vel_target_ne:x() * DT)
    target_pos_ne:y(target_pos_ne:y() + vel_target_ne:y() * DT)

    -- Z control
    local current_hagl_m
    if rangefinder:has_data_orient(25) and rangefinder:status_orient(25) == 4 then
        current_hagl_m = rangefinder:distance_orient(25)
    else
        -- Fallback if rangefinder lost: maintain current offset
        current_hagl_m = target_hagl_m
    end

    local hagl_error = target_hagl_m - current_hagl_m

    -- If error is positive (target > current), we need to go UP (negative Z velocity in NED)
    local vel_correction_z = -1.0 * (hagl_error * P_GAIN_Z)

    -- Limit vertical velocity correction
    if vel_correction_z > MAX_VEL_Z then
        vel_correction_z = MAX_VEL_Z
    elseif vel_correction_z < -MAX_VEL_Z then
        vel_correction_z = -MAX_VEL_Z
    end

    -- Integrate vertical offset
    offset_z = offset_z + vel_correction_z * DT

    -- Send Z offset to position controller
    -- poscontrol:set_posvelaccel_offset expects Vector3f targets
    local pos_offset = Vector3f()
    pos_offset:z(offset_z)
    local vel_offset = Vector3f()
    vel_offset:z(vel_correction_z)
    local accel_offset = Vector3f()

    if poscontrol and poscontrol.set_posvelaccel_offset then
        poscontrol:set_posvelaccel_offset(pos_offset, vel_offset, accel_offset)
    end

    -- Send base PVA target
    local pos_target = Vector3f()
    pos_target:x(target_pos_ne:x())
    pos_target:y(target_pos_ne:y())
    pos_target:z(target_pos_z)

    local vel_target = Vector3f()
    vel_target:x(vel_target_ne:x())
    vel_target:y(vel_target_ne:y())
    vel_target:z(0)

    local accel_target = Vector3f()
    accel_target:x(accel_target_ne:x())
    accel_target:y(accel_target_ne:y())
    accel_target:z(0)

    -- use_yaw is false, so user can control yaw with RC sticks (subject to GUIDED_OPTIONS)
    vehicle:set_target_posvelaccel_NED(pos_target, vel_target, accel_target, false, 0, false, 0, false)

    return update, 1000 / RUN_HZ
end

gcs:send_text(6, "GAT: Script loaded")
return update, 1000
