local lastVelocity = 0


local function findDrone()
    local Drones = FindAllOf("LouisFPV_C")
    if not Drones or #Drones == 0 then return nil end
    local drone = Drones[1]
    if not drone:IsValid() then return nil end
    return drone
end

-- Detection loop: runs every 50ms (20x per second, fast enough to catch crashes)
local function detectionLoop()
    ExecuteWithDelay(25, function()
        local drone = findDrone()
        if drone then
            local vel = drone:GetVelocity()
            local currentSpeed = math.sqrt(vel.X^2 + vel.Y^2 + vel.Z^2)

            -- Use deceleration instead of absolute speed
            -- if you lost 350+ units of speed in one 50ms window, that's a crash
            local decel = lastVelocity - currentSpeed
            local isCrash = (lastVelocity > 300 and decel > 350)

            if isCrash then
                print(string.format("[TelemetryCrX] CRASH! %.2f -> %.2f (lost %.2f)",
                    lastVelocity, currentSpeed, decel))
                
            end

            lastVelocity = currentSpeed
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
            local VectorXYZ=drone:K2_GetActorLocation()
            local X_loc=VectorXYZ.X
            local Y_loc=VectorXYZ.Y
            local Z_loc=VectorXYZ.Z

            -- Angular Rotation (how fast our drone is rotating)
            local VectorXYZAng=drone:GetPhysicsAngularVelocity()
            local X_rot=VectorXYZAng.X
            local Y_rot=VectorXYZAng.Y
            local Z_rot=VectorXYZAng.Z

            -- Pitch Yaw Roll 
            local VectorRotate=drone:K2_GetActorRotation()
            local Pitch=VectorRotate.Pitch
            local Yaw=VectorRotate.Yaw
            local Roll=VectorRotate.Roll
            print(string.format("[TelemetryCrX] Speed: %.2f | LastSpeed: %.2f | X: %.2f | Y: %.2f | Z: %.2f | Pitch: %.2f | Yaw: %.2f | Roll: %.2f",
                speed, lastVelocity, X_loc, Y_loc, Z_loc, Pitch, Yaw, Roll))
        else
            print("[TelemetryCrX] No drone")
        end
        statusLoop()
    end)
end

detectionLoop()
statusLoop()