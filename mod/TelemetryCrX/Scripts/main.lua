local lastVelocity = 0

-- Gyro Tracking (Virtual IMU) Variables
local lastPitch, lastYaw, lastRoll = nil, nil, nil
local X_rot, Y_rot, Z_rot = 0, 0, 0

local function findDrone()
    local Drones = FindAllOf("LouisFPV_C")
    if not Drones or #Drones == 0 then return nil end
    local drone = Drones[1]
    if not drone:IsValid() then return nil end
    return drone
end

-- Helper to handle angle wrapping seamlessly when flipping past -180/180 degrees
local function getAngleDelta(current, last)
    local delta = current - last
    if delta > 180 then delta = delta - 360
    elseif delta < -180 then delta = delta + 360 end
    return delta
end

-- Detection loop: runs every 25ms (40x per second, fast enough to catch crashes)
local function detectionLoop()
    ExecuteWithDelay(25, function()
        local drone = findDrone()
        if drone then
            local vel = drone:GetVelocity()
            local currentSpeed = math.sqrt(vel.X^2 + vel.Y^2 + vel.Z^2)

            -- Use deceleration instead of absolute speed
            -- if you lost 350+ units of speed in one 25ms window, that's a crash
            local decel = lastVelocity - currentSpeed
            local isCrash = (lastVelocity > 300 and decel > 350)

            if isCrash then
                print(string.format("[TelemetryCrX] CRASH! %.2f -> %.2f (lost %.2f)",
                    lastVelocity, currentSpeed, decel))
            end

            lastVelocity = currentSpeed

            -- Angular Velocity
            local VectorRotate = drone:K2_GetActorRotation()
            local Pitch = VectorRotate.Pitch or 0
            local Yaw = VectorRotate.Yaw or 0
            local Roll = VectorRotate.Roll or 0

            -- We calculate Angular Velocity until I find a way to actually get the actual Velocity.
            if lastPitch and lastYaw and lastRoll then
                -- Multiplying the change by 40 turns degrees-per-25ms into true Degrees Per Second (DPS)
                Y_rot = getAngleDelta(Pitch, lastPitch) * 40 -- Pitch Velocity
                Z_rot = getAngleDelta(Yaw, lastYaw) * 40     -- Yaw Velocity
                X_rot = getAngleDelta(Roll, lastRoll) * 40   -- Roll Velocity
            end

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

            -- Drone Location in XYZ Coordinates
            local VectorXYZ = drone:K2_GetActorLocation()
            local X_loc = VectorXYZ.X or 0
            local Y_loc = VectorXYZ.Y or 0
            local Z_loc = VectorXYZ.Z or 0

            -- Pitch Yaw Roll 
            local VectorRotate = drone:K2_GetActorRotation()
            local Pitch = VectorRotate.Pitch or 0
            local Yaw = VectorRotate.Yaw or 0
            local Roll = VectorRotate.Roll or 0
            
            -- Note: X_rot, Y_rot, and Z_rot are now pulled directly from the live 25ms calculations above

            print(string.format("[TelemetryCrX] Speed: %.2f | LastSpeed: %.2f | X: %.2f | Y: %.2f | Z: %.2f | X_rot: %.2f | Y_rot: %.2f | Z_rot: %.2f | Pitch: %.2f | Yaw: %.2f | Roll: %.2f",
                speed, lastVelocity, X_loc, Y_loc, Z_loc, X_rot, Y_rot, Z_rot, Pitch, Yaw, Roll))
        else
            print("[TelemetryCrX] No drone")
        end
        statusLoop()
    end)
end

detectionLoop()
statusLoop()