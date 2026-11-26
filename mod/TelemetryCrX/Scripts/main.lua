-- ===================== CONFIG (tune these) =====================
local GYRO_MODE         = "derived"  -- "derived" (consistent, from angle changes) or "physics" (mesh component).
                                     -- Status line prints BOTH so you can compare before switching.
local HIT_IMPULSE_MIN   = 0          -- min NormalImpulse magnitude to count a hit as a crash.
                                     -- Watch the "[TelemetryCrX] HIT" prints, then raise it above floor-scrape values.
local CRASH_COOLDOWN    = 0.5        -- seconds; one crash fires several hits, count it once
local JUMP_MIN          = 300        -- units; extra slack before a position change counts as a teleport/reset
local FALLBACK_DECEL    = 14000      -- units/s^2, used only until the first real hit is seen
local FALLBACK_MIN_SPEED = 300
local FALLBACK_ANG_SPIKE_3D = 300    -- deg/s; extra confirmation required ONLY in 3D mode (IS3D? = true),
                                     -- so hard braking there isn't a crash. Angle/acro use the plain speed-drop rule.
                                     -- Set to 0 to disable it and use the plain rule everywhere.
-- ===============================================================

-- =================== TELEMETRY OUTPUT ==========================
local TELEMETRY_DIR  = (os.getenv("TEMP") or ".") .. "\\TelemetryCrX"
local TELEMETRY_FILE = TELEMETRY_DIR .. "\\state.json"
local telemetryEnabled = true

-- Create output directory (silently succeeds if it already exists)
os.execute('mkdir "' .. TELEMETRY_DIR .. '" 2>nul')

-- Verify io is available for file writes
if not io or not io.open then
    print("[TelemetryCrX] WARNING: 'io' library not available — telemetry file output disabled (console-only mode).")
    telemetryEnabled = false
else
    print("[TelemetryCrX] Telemetry file: " .. TELEMETRY_FILE)
end
-- ===============================================================

local BP = "/Game/BASE_FPV_LOUIS/LouisFPV.LouisFPV_C:"

local lastVelocity = 0
local lastPitch, lastYaw, lastRoll = nil, nil, nil
local lastLoc = nil
local lastTime = nil
local X_rot, Y_rot, Z_rot = 0, 0, 0      -- Roll / Pitch / Yaw rates in deg/s (selected by GYRO_MODE)
local physX, physY, physZ = nil, nil, nil -- raw physics reading, for comparison only
local gyroSource = "none"
local telemetrySeq = 0

-- Event state
local crashFlag = false
local resetFlag = nil
local hookEverFired = false
local lastCrashTime = -math.huge
local crashCount, resetCount = 0, 0

local function findDrone()
    local Drones = FindAllOf("LouisFPV_C")
    if not Drones or #Drones == 0 then return nil end
    local drone = Drones[1]
    if not drone:IsValid() then return nil end
    return drone
end

local function getAngleDelta(current, last)
    local delta = current - last
    if delta > 180 then delta = delta - 360
    elseif delta < -180 then delta = delta + 360 end
    return delta
end

-- Safe read of a Blueprint property (names with spaces / '?' work via indexing)
local function readProp(obj, name)
    local ok, v = pcall(function() return obj[name] end)
    if ok then return v end
    return nil
end

-- Angular velocity from the drone's SkeletalMeshComponent (the property named "drone"),
-- NOT the actor and not necessarily RootComponent.
local function getPhysicsAngularVelocity(drone)
    local ok, ang = pcall(function()
        local mesh = drone.drone
        if not mesh or not mesh:IsValid() then return nil end
        return mesh:GetPhysicsAngularVelocityInDegrees(FName("None"))
    end)
    if ok and ang then
        return ang.X or 0, ang.Y or 0, ang.Z or 0
    end
    return nil
end

local function registerCrash(now, reason)
    if now - lastCrashTime < CRASH_COOLDOWN then return false end
    lastCrashTime = now
    crashCount = crashCount + 1
    print(string.format("[TelemetryCrX] CRASH #%d (%s)", crashCount, reason))
    return true
end

-- ---------------------------------------------------------------
-- Telemetry JSON writer — serialises all drone state into a single
-- JSON object and writes it to TELEMETRY_FILE.  The Python decoder
-- reads this file; a sequence number lets it detect stale reads.
-- ---------------------------------------------------------------
local function writeTelemetry(drone, speed, vel, loc, rot, crashEv, resetEv)
    if not telemetryEnabled then return end

    telemetrySeq = telemetrySeq + 1

    local armed = readProp(drone, "Armed?")
    local is3d  = readProp(drone, "IS3D?")
    local thr   = readProp(drone, "ThrottleAxis")
    local pit   = readProp(drone, "PitchAxis")
    local rol   = readProp(drone, "RollAxis")
    local yaw   = readProp(drone, "YawAxis")

    local json = string.format(
        '{' ..
        '"seq":%d,"t":%.6f,' ..
        '"speed":%.4f,' ..
        '"x":%.4f,"y":%.4f,"z":%.4f,' ..
        '"vx":%.4f,"vy":%.4f,"vz":%.4f,' ..
        '"pitch":%.4f,"yaw":%.4f,"roll":%.4f,' ..
        '"gyro_x":%.4f,"gyro_y":%.4f,"gyro_z":%.4f,"gyro_src":"%s",' ..
        '"crashes":%d,"resets":%d,' ..
        '"crash_ev":%s,"reset_ev":%s,' ..
        '"armed":%s,"is_3d":%s,' ..
        '"throttle":%.4f,"pitch_in":%.4f,"roll_in":%.4f,"yaw_in":%.4f' ..
        '}',
        telemetrySeq, os.clock(),
        speed,
        loc.X or 0, loc.Y or 0, loc.Z or 0,
        vel.X or 0, vel.Y or 0, vel.Z or 0,
        rot.Pitch or 0, rot.Yaw or 0, rot.Roll or 0,
        X_rot, Y_rot, Z_rot, gyroSource,
        crashCount, resetCount,
        crashEv  and "true" or "false",
        resetEv  and "true" or "false",
        armed == true and "true" or "false",
        is3d  == true and "true" or "false",
        tonumber(thr) or 0, tonumber(pit) or 0,
        tonumber(rol) or 0, tonumber(yaw) or 0
    )

    local ok, err = pcall(function()
        local f = io.open(TELEMETRY_FILE, "w")
        if f then
            f:write(json)
            f:flush()
            f:close()
        end
    end)
    if not ok then
        print("[TelemetryCrX] Telemetry write error: " .. tostring(err))
    end
end

-- ---------------------------------------------------------------
-- Hook 1: the game's own collision event (component OnHit bound event on the mesh).
-- Params: HitComponent, OtherActor, OtherComp, NormalImpulse, Hit
-- ---------------------------------------------------------------
local okHit, errHit = pcall(function()
    RegisterHook(BP .. "BndEvt__Drone_K2Node_ComponentBoundEvent_0_ComponentHitSignature__DelegateSignature",
        function(self, HitComponent, OtherActor, OtherComp, NormalImpulse, Hit)
            local ok, err = pcall(function()
                local imp = NormalImpulse:get()
                local mag = math.sqrt((imp.X or 0)^2 + (imp.Y or 0)^2 + (imp.Z or 0)^2)

                local otherName = "?"
                pcall(function()
                    local a = OtherActor:get()
                    if a and a:IsValid() then otherName = a:GetFullName() end
                end)

                hookEverFired = true
                print(string.format("[TelemetryCrX] HIT impulse=%.1f (min %.1f) other=%s", mag, HIT_IMPULSE_MIN, otherName))

                if mag > HIT_IMPULSE_MIN then
                    crashFlag = true
                end
            end)
            if not ok then print("[TelemetryCrX] Hit callback error: " .. tostring(err)) end
        end)
end)
print(okHit and "[TelemetryCrX] Hit hook registered"
      or ("[TelemetryCrX] Hit hook FAILED (fallback only): " .. tostring(errHit)))

-- ---------------------------------------------------------------
-- Hook 2: the game's reset functions (marks episode boundaries / teleports)
-- ---------------------------------------------------------------
for _, fname in ipairs({ "Reset Quad", "full reset" }) do
    local ok, err = pcall(function()
        RegisterHook(BP .. fname, function(self)
            resetFlag = fname
        end)
    end)
    print(ok and ("[TelemetryCrX] Reset hook registered: " .. fname)
          or ("[TelemetryCrX] Reset hook FAILED for '" .. fname .. "': " .. tostring(err)))
end

-- Clears history so a teleport doesn't produce huge fake angular rates / crashes
local function clearHistory()
    lastPitch, lastYaw, lastRoll = nil, nil, nil
    lastLoc = nil
    lastVelocity = 0
    lastTime = nil
    X_rot, Y_rot, Z_rot = 0, 0, 0
end

-- Detection loop: ~25ms
local function detectionLoop()
    ExecuteWithDelay(25, function()
        local drone = findDrone()
        if drone then
            local now = os.clock()

            -- Track per-tick event flags for telemetry
            local resetThisTick = false
            local crashThisTick = false

            if resetFlag then
                resetCount = resetCount + 1
                resetThisTick = true
                print(string.format("[TelemetryCrX] RESET #%d (%s)", resetCount, resetFlag))
                resetFlag = nil
                crashFlag = false
                clearHistory()
            end

            local dt = lastTime and (now - lastTime) or nil
            lastTime = now

            local vel = drone:GetVelocity()
            local currentSpeed = math.sqrt(vel.X^2 + vel.Y^2 + vel.Z^2)

            -- Teleport guard (catches resets the hooks missed)
            local loc = drone:K2_GetActorLocation()
            if lastLoc and dt and dt > 0 then
                local dx, dy, dz = loc.X - lastLoc.X, loc.Y - lastLoc.Y, loc.Z - lastLoc.Z
                local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > currentSpeed * dt * 3 + JUMP_MIN then
                    print(string.format("[TelemetryCrX] TELEPORT detected (%.0f units in %.0f ms)", dist, dt * 1000))
                    clearHistory()
                    lastLoc = { X = loc.X, Y = loc.Y, Z = loc.Z }
                    detectionLoop()
                    return
                end
            end
            lastLoc = { X = loc.X, Y = loc.Y, Z = loc.Z }

            -- Orientation
            local VectorRotate = drone:K2_GetActorRotation()
            local Pitch = VectorRotate.Pitch or 0
            local Yaw = VectorRotate.Yaw or 0
            local Roll = VectorRotate.Roll or 0

            -- Derived rates using real elapsed time
            local dRoll, dPitch, dYaw = nil, nil, nil
            if lastPitch and lastYaw and lastRoll and dt and dt > 0 then
                dRoll  = getAngleDelta(Roll,  lastRoll)  / dt
                dPitch = getAngleDelta(Pitch, lastPitch) / dt
                dYaw   = getAngleDelta(Yaw,   lastYaw)   / dt
            end

            -- Physics rates (always read, for comparison)
            physX, physY, physZ = getPhysicsAngularVelocity(drone)

            if GYRO_MODE == "physics" and physX then
                X_rot, Y_rot, Z_rot = physX, physY, physZ
                gyroSource = "physics"
            elseif dRoll then
                X_rot, Y_rot, Z_rot = dRoll, dPitch, dYaw
                gyroSource = "derived"
            end

            -- ---- Crash detection ----
            if crashFlag then
                crashFlag = false
                crashThisTick = registerCrash(now, "collision hit")
            elseif not hookEverFired and dt and dt > 0 then
                local decelRate = (lastVelocity - currentSpeed) / dt
                local angSpike = math.max(math.abs(X_rot), math.abs(Y_rot), math.abs(Z_rot))
                local is3D = readProp(drone, "IS3D?") == true
                local confirmed = (not is3D) or angSpike > FALLBACK_ANG_SPIKE_3D
                if lastVelocity > FALLBACK_MIN_SPEED
                   and decelRate > FALLBACK_DECEL
                   and confirmed then
                    crashThisTick = registerCrash(now, string.format("fallback%s: %.0f -> %.0f, decel %.0f/s^2, ang %.0f deg/s",
                        is3D and " 3D" or "", lastVelocity, currentSpeed, decelRate, angSpike))
                end
            end

            -- Write telemetry state to JSON file for the Python decoder
            writeTelemetry(drone, currentSpeed, vel, loc, VectorRotate, crashThisTick, resetThisTick)

            lastVelocity = currentSpeed
            lastPitch, lastYaw, lastRoll = Pitch, Yaw, Roll
        end
        detectionLoop()
    end)
end

-- Status print every second
local function statusLoop()
    ExecuteWithDelay(1000, function()
        local drone = findDrone()
        if drone then
            local vel = drone:GetVelocity()
            local speed = math.sqrt(vel.X^2 + vel.Y^2 + vel.Z^2)

            local L = drone:K2_GetActorLocation()
            local R = drone:K2_GetActorRotation()

            print(string.format("[TelemetryCrX] Speed: %.2f | LastSpeed: %.2f | X: %.2f | Y: %.2f | Z: %.2f | X_rot: %.2f | Y_rot: %.2f | Z_rot: %.2f | Gyro: %s | Pitch: %.2f | Yaw: %.2f | Roll: %.2f | Crashes: %d | Resets: %d | Hook: %s",
                speed, lastVelocity, L.X or 0, L.Y or 0, L.Z or 0, X_rot, Y_rot, Z_rot, gyroSource,
                R.Pitch or 0, R.Yaw or 0, R.Roll or 0,
                crashCount, resetCount, hookEverFired and "active" or "no hits yet"))

            -- Comparison + game state (IS3D?, Armed?, stick inputs)
            print(string.format("[TelemetryCrX] physGyro: %s | IS3D: %s | Armed: %s | Sticks T/P/R/Y: %s / %s / %s / %s",
                physX and string.format("%.1f, %.1f, %.1f", physX, physY, physZ) or "n/a",
                tostring(readProp(drone, "IS3D?")), tostring(readProp(drone, "Armed?")),
                tostring(readProp(drone, "ThrottleAxis")), tostring(readProp(drone, "PitchAxis")),
                tostring(readProp(drone, "RollAxis")), tostring(readProp(drone, "YawAxis"))))
        else
            print("[TelemetryCrX] No drone")
        end
        statusLoop()
    end)
end

detectionLoop()
statusLoop()
