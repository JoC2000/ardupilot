--[[
 Repeatable stick excitation test

 Runs in LOITER so position hold stays active. On each LOW->HIGH flip of the
 RC_OVERRIDE_ENABLE switch (RCx_OPTION = 46) it overrides ONE stick channel with
 a fixed doublet profile, then releases it. Flipping that switch LOW at any time
 makes ArduPilot drop all overrides immediately and the pilot gets the stick back,
 independently of this script.

 Profile on the selected axis (A = TST_AMP stick fraction, T = TST_SEG_T):
   +A (T), -A (T), 0 (HOLD_S), -A (T), +A (T), 0 (HOLD_S)
 LOITER only starts braking LOIT_BRK_DELAY after the stick is centred, so the
 neutral hold lets it stop completely. Both commands then start from rest and the
 mirrored second one brings the vehicle back toward the start point.

 Required vehicle params:
   SCR_ENABLE = 1
   RCx_OPTION = 46 on a spare 2-position switch
   RC_OVERRIDE_TIME between 0.1 and 1.0 s 

 Log message TSTX, every loop while running:
   St  1 running, 2 finished, 3 aborted
   Ax  axis (1 roll, 2 pitch, 3 yaw)
   Sg  profile segment (1..6)
   U   commanded stick, normalized as ArduPilot reads it (after RCx_REVERSED)
   PWM value written to the channel
   CC  CUSTOM_CONTROLLER switch position (0 low, 1 mid, 2 high, 255 not assigned)
   D   horizontal distance from the start point (m)
--]]

local PARAM_TABLE_KEY = 60
local MODE_LOITER = 5
local AUX_RC_OVERRIDE_ENABLE = 46
local AUX_CUSTOM_CONTROLLER = 109
local SWITCH_HIGH = 2
local MAX_DIST_M = 5.0      -- abort if further than this from the start point
local MAX_LEAN_DEG = 45      -- abort on excessive roll or pitch
local HOLD_S = 3.0           -- neutral hold, long enough for LOITER to brake to a stop
local LOOP_MS = 10
local SEV_WARNING = 4
local SEV_INFO = 6

local RCMAP_PARAMS = { "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_YAW" }

assert(param:add_table(PARAM_TABLE_KEY, "TST_", 3), "TST: could not add param table")

local function bind_param(idx, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "TST: could not add param " .. name)
    return Parameter("TST_" .. name)
end

--[[
  // @Param: TST_AXIS
  // @DisplayName: Excitation axis
  // @Description: Stick axis excited by the test profile
  // @Values: 1:Roll,2:Pitch,3:Yaw
  // @User: Standard
--]]
local TST_AXIS = bind_param(1, "AXIS", 1)

--[[
  // @Param: TST_AMP
  // @DisplayName: Excitation amplitude
  // @Description: Doublet amplitude as a fraction of full stick
  // @Range: 0.05 1.0
  // @User: Standard
--]]
local TST_AMP = bind_param(2, "AMP", 0.5)

--[[
  // @Param: TST_SEG_T
  // @DisplayName: Excitation segment time
  // @Description: Duration T of each doublet half
  // @Range: 0.1 3.0
  // @Units: s
  // @User: Standard
--]]
local TST_SEG_T = bind_param(3, "SEG_T", 0.5)

local sw_enable = rc:find_channel_for_option(AUX_RC_OVERRIDE_ENABLE)
local sw_cc = rc:find_channel_for_option(AUX_CUSTOM_CONTROLLER)

if not sw_enable then
    gcs:send_text(SEV_WARNING, "TST: no RCx_OPTION=46 switch, script disabled")
    return
end

local running = false
local prev_sw = sw_enable and sw_enable:get_aux_switch_pos() or 0
local t0_s, start_pos, axis, chan, amp, seg_t, profile
local trim, pwm_min, pwm_max, reversed

local function now_s()
    return millis():tofloat() * 0.001
end

local function cc_pos()
    if sw_cc then
        return sw_cc:get_aux_switch_pos()
    end
    return 255
end

local function stick_to_pwm(u)
    if reversed then
        u = -u
    end
    if u >= 0 then
        return math.floor(trim + u * (pwm_max - trim) + 0.5)
    end
    return math.floor(trim + u * (trim - pwm_min) + 0.5)
end

local function horiz_dist()
    local p = ahrs:get_relative_position_NED_origin()
    if not p then
        return nil
    end
    local dx = p:x() - start_pos:x()
    local dy = p:y() - start_pos:y()
    return math.sqrt(dx * dx + dy * dy)
end

local function log(state, seg, u, pwm, dist)
    logger:write('TSTX', 'St,Ax,Sg,U,PWM,CC,D', 'BBBfHBf', state, axis, seg, u, pwm, cc_pos(), dist)
end

local function stop(state, seg, dist, msg)
    chan:set_override(0)
    running = false
    log(state, seg, 0, 0, dist or -1)
    gcs:send_text(state == 3 and SEV_WARNING or SEV_INFO, msg)
end

-- returns nil on success, otherwise the reason the test was not started
local function try_start()
    if not arming:is_armed() then
        return "not armed"
    end
    if vehicle:get_mode() ~= MODE_LOITER then
        return "not in LOITER"
    end
    local ovr_t = param:get("RC_OVERRIDE_TIME")
    if not ovr_t or ovr_t < 0.1 or ovr_t > 1.0 then
        return "RC_OVERRIDE_TIME must be 0.1..1.0 s"
    end
    local a = math.floor(TST_AXIS:get())
    if a < 1 or a > 3 then
        return "TST_AXIS must be 1..3"
    end
    local a_amp = TST_AMP:get()
    local a_seg_t = TST_SEG_T:get()
    if a_amp < 0.05 or a_amp > 1.0 or a_seg_t < 0.1 or a_seg_t > 3.0 then
        return "TST_AMP or TST_SEG_T out of range"
    end
    local p = ahrs:get_relative_position_NED_origin()
    if not p then
        return "no position estimate"
    end

    local rcmap = param:get(RCMAP_PARAMS[a])
    if not rcmap then
        return "could not read " .. RCMAP_PARAMS[a]
    end

    local ch_num = math.floor(rcmap)
    local prefix = string.format("RC%d_", ch_num)
    chan = rc:get_channel(ch_num)
    trim = param:get(prefix .. "TRIM")
    pwm_min = param:get(prefix .. "MIN")
    pwm_max = param:get(prefix .. "MAX")
    reversed = param:get(prefix .. "REVERSED") == 1
    if not chan or not trim or not pwm_min or not pwm_max then
        return "could not read RC" .. ch_num .. " setup"
    end

    axis = a
    amp = a_amp
    seg_t = a_seg_t
    -- {stick sign, duration s}; each doublet starts from rest so the mirrored one cancels the drift
    profile = { {1, seg_t}, {-1, seg_t}, {0, HOLD_S}, {-1, seg_t}, {1, seg_t}, {0, HOLD_S} }
    start_pos = p
    t0_s = now_s()
    running = true
    gcs:send_text(SEV_INFO, string.format("TST: start axis %d (RC%d) amp %.2f T %.2fs CC %d",
                                          axis, ch_num, amp, seg_t, cc_pos()))
    return nil
end

local function run_step(sw)
    local dist = horiz_dist()
    if sw ~= SWITCH_HIGH then
        return stop(3, 0, dist, "TST: aborted, override switch off")
    end
    if not arming:is_armed() then
        return stop(3, 0, dist, "TST: aborted, disarmed")
    end
    if vehicle:get_mode() ~= MODE_LOITER then
        return stop(3, 0, dist, "TST: aborted, mode changed")
    end
    if not dist then
        return stop(3, 0, dist, "TST: aborted, position estimate lost")
    end
    if dist > MAX_DIST_M then
        return stop(3, 0, dist, string.format("TST: aborted, %.2f m from start", dist))
    end
    local max_lean = math.rad(MAX_LEAN_DEG)
    if math.abs(ahrs:get_roll_rad()) > max_lean or math.abs(ahrs:get_pitch_rad()) > max_lean then
        return stop(3, 0, dist, "TST: aborted, lean angle")
    end

    local t = now_s() - t0_s
    local seg_end = 0
    for i, s in ipairs(profile) do
        seg_end = seg_end + s[2]
        if t < seg_end then
            local u = s[1] * amp
            local pwm = stick_to_pwm(u)
            chan:set_override(pwm)
            log(1, i, u, pwm, dist)
            return
        end
    end
    stop(2, 0, dist, string.format("TST: done, %.2f m from start", dist))
end

local function update()
    local sw = sw_enable:get_aux_switch_pos()
    if running then
        run_step(sw)
    elseif sw == SWITCH_HIGH and prev_sw ~= SWITCH_HIGH then
        local err = try_start()
        if err then
            gcs:send_text(SEV_WARNING, "TST: not started, " .. err)
        else
            run_step(sw)
        end
    end
    prev_sw = sw
    return update, LOOP_MS
end

-- on any script error release the stick and stop for good
local function protected_update()
    local ok, ret = pcall(update)
    if ok then
        return protected_update, LOOP_MS
    end
    if chan then
        chan:set_override(0)
    end
    gcs:send_text(0, "TST: script error, stopped: " .. tostring(ret))
end

gcs:send_text(SEV_INFO, "TST: loaded, flip RC_OVERRIDE_ENABLE switch to run")
return protected_update()
