-- ArduCopter SITL circle demo. API target: Copter-4.8.0-dev (cee87141).
-- Copy to SITL's copter_files/scripts/ directory; SCR_ENABLE=1; reboot.
-- Set CDEM_* parameters, then CDEM_START=1 while disarmed.
-- Local axes: X north, Y east, Z down (metres), NOT Gazebo world axes, instead AP axis convention.
-- Mode 1: continuous circle; mode 2: sequential polygon vertices.
-- Both use GUIDED velocity commands with position-error feedback at 20 Hz.

local KEY, PREFIX = 86, "CDEM_" -- Change KEY if another Lua script uses 86.
local num_params = 11
assert(param:add_table(KEY, PREFIX, num_params), "CDEM: parameter table conflict")
local function bind(index, name, default)
    assert(param:add_param(KEY, index, name, default), "CDEM: add " .. name)
    local p = Parameter()
    assert(p:init(PREFIX .. name), "CDEM: bind " .. name)
    return p
end
local P = {
    START   = bind(1, "START", 0),
    MODE    = bind(2, "MODE", 1),
    ALT     = bind(3, "ALT", 10),
    RADIUS  = bind(4, "RADIUS", 10),
    SPEED   = bind(5, "SPEED", 2),
    NPTS    = bind(6, "NPTS", 12),
    LAPS    = bind(7, "LAPS", 1),
    DO_RTL  = bind(8, "DO_RTL", 0),
    WP_TOL  = bind(9, "WP_TOL", 0.5),
    TIMEOUT = bind(10, "TIMEOUT", 600),
    BOOT_DELAY = bind(11, "BOOT_DELAY", 10000),
}
-- Never replay a stored start command on script restart / vehicle reboot.
assert(P.START:set_and_save(0), "CDEM: cannot clear START")

local GUIDED, RTL, LAND = 4, 6, 9
local PERIOD_MS, TAU = 50, 2 * math.pi
local KP, ACCEL = 0.8, 1.0 -- Position feedback / tangential ramp (m/s^2).
local state, cfg = "IDLE", nil
local began, entered, previous, last_attempt, settled
local cx, cy, target_z, theta, circle_speed, waypoint
local last_say = 0
local boot_time_elapsed = false

local function elapsed(stamp)
    return (millis() - stamp):tofloat() * 0.001
end
local function say(text, throttle)
    -- Send text every time the function is called, optionally try to print every throttle
    throttle = throttle or 0
    if throttle > 0 then
        if elapsed(last_say) > throttle * 0.001 then 
            gcs:send_text(6, "CDEM: " .. text)
            last_say = millis()
            return 
        end
    else
        gcs:send_text(6, "CDEM: " .. text)
    end
end
local function vec(x, y, z)
    local v = Vector3f()
    v:x(x); v:y(y); v:z(z)
    return v
end
local function clamp(x, lo, hi)
    return math.max(lo, math.min(hi, x))
end
local function transition(next_state)
    state, entered, settled, last_attempt = next_state, millis(), nil, nil
    say(next_state)
end
local function release(reason)
    P.START:set_and_save(0)
    state = "IDLE"
    say(reason)
end
local function finish(abort_reason)
    if abort_reason then
        gcs:send_text(3, "CDEM: " .. abort_reason .. "; LAND")
    end
    transition(abort_reason and "ABORT" or "FINISH")
end
local function snapshot()
    local c = {}
    for name, p in pairs(P) do c[name] = p:get() end
    local function check(name, lo, hi, integer)
        local v = c[name]
        if not v or v ~= v or v < lo or v > hi or
           (integer and v ~= math.floor(v)) then
            return name .. " invalid"
        end
    end
    local err = check("MODE", 1, 2, true) or check("ALT", 2, 100) or
        check("RADIUS", 2, 100) or check("SPEED", 0.2, 10) or
        check("NPTS", 3, 100, true) or check("LAPS", 1, 20, true) or
        check("DO_RTL", 0, 1, true) or check("WP_TOL", 0.1, 2) or
        check("TIMEOUT", 30, 7200)
    if err then return nil, err end
    if c.MODE == 1 and c.SPEED * c.SPEED / c.RADIUS > 3 then
        return nil, "SPEED^2/RADIUS must be <=3 m/s^2"
    end
    if c.MODE == 2 and c.WP_TOL >= c.RADIUS * math.sin(math.pi / c.NPTS) then
        return nil, "WP_TOL too large for waypoint spacing"
    end
    local length = c.MODE == 1 and TAU * c.RADIUS or
        2 * c.NPTS * c.RADIUS * math.sin(math.pi / c.NPTS)
    if c.TIMEOUT < 60 + (c.RADIUS + c.LAPS * length) / c.SPEED then
        return nil, "increase TIMEOUT for this path/speed"
    end
    return c
end

-- Feed-forward circle velocity plus proportional position correction.
-- The extra 1 m/s of headroom permits catching up with the reference.
local function command(p, x, y, z, vx, vy, limit)
    local dx, dy, dz = x - p:x(), y - p:y(), z - p:z()
    vx, vy = vx + KP * dx, vy + KP * dy
    local speed = math.sqrt(vx * vx + vy * vy)
    if speed > limit then vx, vy = vx * limit / speed, vy * limit / speed end
    if not vehicle:set_target_velocity_NED(vec(vx, vy, clamp(KP * dz, -1, 1))) then
        finish("velocity command rejected")
        return false
    end
    return true
end
local function at_target(p, v, x, y, z)
    local dx, dy = x - p:x(), y - p:y()
    local good = dx * dx + dy * dy <= cfg.WP_TOL * cfg.WP_TOL and
        math.abs(z - p:z()) <= 0.3 and
        v:x() * v:x() + v:y() * v:y() + v:z() * v:z() <= 0.09
    if not good then settled = nil; return false end
    if not settled then settled = millis() end
    return elapsed(settled) >= 0.5
end

local function update()
    local now = millis()
    local dt = previous and elapsed(previous) or PERIOD_MS * 0.001
    previous = now

    if state == "IDLE" then
        if P.START:get() ~= 1 then return end
        if arming:is_armed() then release("start rejected: disarm first"); return end
        local err
        cfg, err = snapshot()
        if not cfg then release(err); return end
        if not ahrs:healthy() or not ahrs:home_is_set() or
           not ahrs:get_relative_position_NED_origin() then
            release("start rejected: wait for EKF/home"); return
        end
        if not vehicle:set_mode(GUIDED) then release("GUIDED rejected"); return end
        began = now
        transition("ARM")
        return
    end

    -- Respect operator / failsafe mode changes; never fight them or re-arm.
    if vehicle:get_mode() ~= GUIDED then
        release("mode changed; control released"); return
    end
    if state ~= "ARM" and not arming:is_armed() then
        release("disarmed; control released"); return
    end

    if state == "FINISH" or state == "ABORT" then
        if not arming:is_armed() then release("stopped"); return end
        -- Keep trying LAND if an RTL request is rejected. Do not disarm in air.
        local mode = state == "FINISH" and cfg.DO_RTL == 1 and RTL or LAND
        if not last_attempt or elapsed(last_attempt) >= 1 then
            last_attempt = now
            if vehicle:set_mode(mode) then
                release(mode == RTL and "RTL handed to autopilot" or "LAND handed to autopilot")
            elseif mode == RTL then
                finish("RTL rejected")
            else
                gcs:send_text(3, "CDEM: LAND rejected; retrying")
            end
        end
        return
    end

    if P.START:get() ~= 1 then finish("cancelled"); return end
    if elapsed(began) > cfg.TIMEOUT then finish("mission timeout"); return end
    if dt > 0.5 then finish("script scheduling gap"); return end
    if not ahrs:healthy() then finish("EKF unhealthy"); return end
    local p = ahrs:get_relative_position_NED_origin()
    local v = ahrs:get_velocity_NED()
    if not p or not v then finish("position/velocity unavailable"); return end

    if state == "ARM" then
        if elapsed(entered) > 30 then finish("arming timeout"); return end
        if not arming:is_armed() then
            if not last_attempt or elapsed(last_attempt) >= 2 then
                last_attempt = now
                if not arming:arm() then say("arming denied; check pre-arm messages") end
            end
            return
        end
        -- Capture AFTER arming because arming may reset home.
        cx, cy, target_z = p:x(), p:y(), p:z() - cfg.ALT
        transition("TAKEOFF_REQUEST")
        return
    end

    if state == "TAKEOFF_REQUEST" then
        if elapsed(entered) > 15 then finish("takeoff rejected/timeout"); return end
        local home_p = ahrs:get_relative_position_NED_home()
        if not home_p then finish("home position unavailable"); return end
        -- start_takeoff takes altitude ABOVE HOME, not EKF-origin altitude.
        local above_home = -home_p:z() + p:z() - target_z
        if not last_attempt or elapsed(last_attempt) >= 1 then
            last_attempt = now
            if vehicle:start_takeoff(above_home) then transition("TAKEOFF") end
        end
        return
    end

    if state == "TAKEOFF" then
        if elapsed(entered) > 120 then finish("takeoff timeout"); return end
        -- Do not stream velocities until native takeoff has reached altitude.
        if math.abs(p:z() - target_z) <= 0.3 and math.abs(v:z()) < 0.3 then
            if not settled then settled = now end
            if elapsed(settled) >= 0.5 then transition("ENTRY") end
        else settled = nil end
        return
    end

    if state == "ENTRY" or state == "CLOSE" then
        if not command(p, cx + cfg.RADIUS, cy, target_z, 0, 0, cfg.SPEED) then return end
        if at_target(p, v, cx + cfg.RADIUS, cy, target_z) then
            if state == "CLOSE" then finish(); return end
            theta, circle_speed, waypoint = 0, 0, 1
            transition(cfg.MODE == 1 and "CIRCLE" or "WAYPOINTS")
        end
        return
    end

    if state == "CIRCLE" then
        local remaining = (TAU * cfg.LAPS - theta) * cfg.RADIUS
        local desired_speed = math.min(cfg.SPEED, math.sqrt(2 * ACCEL * math.max(0, remaining)))
        circle_speed = clamp(desired_speed, math.max(0, circle_speed - ACCEL * dt),
                             circle_speed + ACCEL * dt)
        theta = math.min(TAU * cfg.LAPS, theta + circle_speed / cfg.RADIUS * dt)
        local x, y = cx + cfg.RADIUS * math.cos(theta), cy + cfg.RADIUS * math.sin(theta)
        local dx, dy = x - p:x(), y - p:y()
        if dx * dx + dy * dy > math.max(5, cfg.RADIUS * 0.5)^2 then
            finish("circle tracking error"); return
        end
        if not command(p, x, y, target_z, -circle_speed * math.sin(theta),
                       circle_speed * math.cos(theta), cfg.SPEED + 1) then return end
        if theta >= TAU * cfg.LAPS then transition("CLOSE") end
        return
    end

    if state == "WAYPOINTS" then
        -- ENTRY visits vertex 0; k=1..N includes the closing edge to vertex 0.
        local angle = TAU * (waypoint % cfg.NPTS) / cfg.NPTS
        local x, y = cx + cfg.RADIUS * math.cos(angle), cy + cfg.RADIUS * math.sin(angle)
        if not command(p, x, y, target_z, 0, 0, cfg.SPEED) then return end
        if at_target(p, v, x, y, target_z) then
            say(string.format("waypoint %d/%d", waypoint, cfg.NPTS * cfg.LAPS))
            waypoint, settled = waypoint + 1, nil
            if waypoint > cfg.NPTS * cfg.LAPS then finish() end
        end
    end
end

local function loop()
    -- Add a boot delay to ensure the system is ready before starting the main loop
    local err
    cfg, err = snapshot()
    local boot_time = millis()
    if boot_time > cfg.BOOT_DELAY then
        if not boot_time_elapsed then
            say("Boot completed")
            boot_time_elapsed = true
        end
        local ok, err = pcall(update)
        if not ok then
            gcs:send_text(3, "CDEM Lua error: " .. tostring(err):sub(1, 70))
            -- Only request LAND if this script had control; leave other modes alone.
            if state ~= "IDLE" and arming:is_armed() and vehicle:get_mode() == GUIDED then
                transition("ABORT")
            else
                release("script error; stopped")
            end
        end
        return loop, PERIOD_MS
    end
    say("booting..." .. tostring(boot_time) .. " / " .. tostring(cfg.BOOT_DELAY), 1000)
    return loop, PERIOD_MS
end

say("ready; set CDEM_START=1 while disarmed")
return loop, PERIOD_MS
