-- gat_pva_sub.lua
-- Cruise control with terrain-following
-- Use pos-vel-accel to control XY based on pilot yaw
-- Use poscontrol:set_posvelaccel_offset to control Z for terrain following

local RUN_HZ = 20
local DT = 1.0 / RUN_HZ
local GUIDED_MODE = 4

-- Configuration
local P_GAIN_Z = 1.0 -- Proportional gain for vertical correction
local MAX_VEL_Z = 2.0 -- Max vertical velocity correction in m/s

-- State variables
local active = false
local target_pos_ne = Vector2f()
local target_pos_z = 0 -- Base Z target (EKF origin frame)
local target_hagl_m = 0 -- Target height above ground level
local offset_z = 0 -- Vertical offset
local target_yaw_rad = nil -- Locked yaw target for velocity vector

local function update()
    local current_mode = vehicle:get_mode()
    if current_mode ~= GUIDED_MODE then
        if active then
            gcs:send_text(6, "GAT: not active")
        end
        active = false
        return update, 1000 / RUN_HZ
    end

    -- Transition to GUIDED mode
    if not active then
        -- Initialize base targets
        local current_pos = ahrs:get_relative_position_NED_origin()
        if not current_pos then
            gcs:send_text(4, "GAT: waiting for EKF origin")
            return update, 1000 / RUN_HZ
        end

        -- Check rangefinder
        if not rangefinder:has_data_orient(25) or rangefinder:status_orient(25) ~= 4 then -- 4 is RangeFinder_Good
            gcs:send_text(4, "GAT: downward rangefinder not healthy")
            return update, 1000 / RUN_HZ
        end

        target_pos_ne:x(current_pos:x())
        target_pos_ne:y(current_pos:y())
        target_pos_z = current_pos:z()
        target_hagl_m = rangefinder:distance_orient(25)
        offset_z = 0

        active = true
        gcs:send_text(6, string.format("GAT: active, target HAGL %.1fm", target_hagl_m))
    end

    -- Initialize target_yaw_rad if not set
    local ahrs_yaw = ahrs:get_yaw_rad()
    if not target_yaw_rad then
        target_yaw_rad = ahrs_yaw
    end

    -- Get pilot intent for yaw to prevent wobble
    local yaw_chan = rc:get_channel(4)
    local yaw_input = yaw_chan and yaw_chan:norm_input_dz() or 0
    local gyro = ahrs:get_gyro()
    local yaw_rate_rads = gyro and gyro:z() or 0

    -- If stick is in deadband and rotation has stopped, lock the yaw
    if yaw_input == 0 and math.abs(yaw_rate_rads) < 0.05 then
        -- Keep target_yaw_rad locked
    else
        target_yaw_rad = ahrs_yaw
    end

    local speed_ms = param:get("WP_SPD")
    local vel_target_ne = Vector2f()
    vel_target_ne:x(math.cos(target_yaw_rad) * speed_ms)
    vel_target_ne:y(math.sin(target_yaw_rad) * speed_ms)

    local accel_target_ne = Vector2f()
    -- Calculate feed-forward centripetal acceleration to improve turn tracking
    local gyro = ahrs:get_gyro()
    if gyro then
        local yaw_rate_rads = gyro:z()
        accel_target_ne:x(-vel_target_ne:y() * yaw_rate_rads)
        accel_target_ne:y(vel_target_ne:x() * yaw_rate_rads)
    end

    -- Integrate XY position forward based on velocity target
    target_pos_ne:x(target_pos_ne:x() + vel_target_ne:x() * DT)
    target_pos_ne:y(target_pos_ne:y() + vel_target_ne:y() * DT)

    -- Safety check: stop if we get caught on something
    local current_pos = ahrs:get_relative_position_NED_origin()
    if current_pos then
        local distance_m = (target_pos_ne - current_pos:xy()):length()
        if distance_m > 1.0 then
            gcs:send_text(4, string.format("GAT: leash exceeded (%.2fm), switching to MANUAL", distance_m))
            vehicle:set_mode(19) -- MANUAL mode
            active = false
            return update, 1000 / RUN_HZ
        end
    end

    -- Z control (terrain following)
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

    -- bool set_target_posvelaccel_NED(Vector3f pos, Vector3f vel, Vector3f accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
    -- use_yaw is false, so user can control yaw with RC sticks (subject to GUIDED_OPTIONS)
    vehicle:set_target_posvelaccel_NED(pos_target, vel_target, accel_target, false, 0, false, 0, false)

    -- Log key vertical data for analysis
    logger:write('GATD', 'HAGL,Offset,TargZ,YInp,YRat,TYaw', 'ffffff', 'mmm-rr', '------', current_hagl_m, offset_z, target_pos_z, yaw_input, yaw_rate_rads, target_yaw_rad)

    return update, 1000 / RUN_HZ
end

gcs:send_text(6, "GAT: Script loaded")
return update, 1000
