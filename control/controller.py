import time
import math

class Controller:
    def __init__(self, vehicle):
        # Vehicle class
        self.UAV = vehicle

        # Maximum controller output constraints
        self.rollConstrain  = [-4, 4]               # Deg
        self.pitchConstrain = self.rollConstrain    # Deg
        self.thrustConstrain = [-0.5, 0.5]	        # Normalized
        self.yawRateConstrain = [-10, 10]           # Deg / s

        # PID Gains: NORTH (pitch)
        self.kp_NORTH = 0.09
        self.ki_NORTH = 0.001      # Max 0.5 deg with 500 bounds
        self.kd_NORTH = 0.07

        # PID Gains: EAST (roll)
        self.kp_EAST = 0.075
        self.ki_EAST = 0.001       # Max 0.5 deg with 500 bounds
        self.kd_EAST = 0.065

        # PID Gains: DOWN (thrust)
        self.kp_DOWN = 0.002
        self.ki_DOWN = 0.00008     # Max 0.04 with 500 bounds
        self.kd_DOWN = 0.0

        # PID Gains: YAW (yaw rate)
        self.kp_YAW = 0.6
        self.ki_YAW = 0
        self.kd_YAW = 0.4

        # Gain scheduling 
        self.gainHeight = 35.0
        self.gainFactor = 2.0
        
        # Landing check 
        self.landErrorNE = 3.0
        self.landHeight  = 15.0
        self.landCount   = 0 
        self.landCountRequired = 45

        # Previous errors
        self.northPrevError = 0
        self.eastPrevError = 0
        self.downPrevError = 0
        self.yawPrevError = 0

        # Integral tracking
        self.northI = 0
        self.eastI = 0
        self.downI = 0
        self.yawI = 0

        # Integral term constraints
        self.northIcontstrain = [-500, 500]
        self.eastIcontstrain  = [-500, 500]
        self.downIcontstrain  = [-500, 500]
        self.yawIcontstrain   = [-500, 500]

        # Timing
        self.timer = None

    def startController(self):
        self.resetController()
        self.timer = time.time()

    def resetController(self):
        # Reset integral terms
        self.northI = 0
        self.eastI = 0
        self.downI = 0
        self.yawI = 0
        
        # Reset landing counter 
        self.landCount = 0

    def euler2quaternion(self, roll, pitch, yaw):
        # Convert degrees to radians 
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        # Euler angles (rad) to quaternion
        yc = math.cos(yaw * 0.5)
        ys = math.sin(yaw * 0.5)
        rc = math.cos(roll * 0.5)
        rs = math.sin(roll * 0.5)
        pc = math.cos(pitch * 0.5)
        ps = math.sin(pitch * 0.5)

        q0 = yc * rc * pc + ys * rs * ps
        q1 = yc * rs * pc - ys * rc * ps
        q2 = yc * rc * ps + ys * rs * pc
        q3 = ys * rc * pc - yc * rs * ps

        return [q0, q1, q2, q3]

    def sendAttitudeTarget(self, roll, pitch, yawRate, thrustRate):
        # Convert from degrees to radians
        yawRate = math.radians(yawRate)

        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        msg = self.UAV.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            0, # Target system
            0, # Target component
            0b00000000, # If bit is set corresponding input ignored (mappings)
            self.euler2quaternion(roll, pitch, 0), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            yawRate,   # Body yaw rate in rad/s
            thrustRate # Thrust
        )
        
        # Send the constructed message
        self.UAV.send_mavlink(msg)
       
    def constrain(self, val, minVal, maxVal):
        return max(min(maxVal, val), minVal)

    def PID(self, error, errorPrev, I, dt, kp, ki, kd, debug=False):
        # Run the PID controller
        P = error
        I = I + error * dt
        D = (error - errorPrev) / dt
        PID = (kp * P) + (ki * I) + (kd * D)

        # Debugging
        if debug is True:
            print('{:<8.0f} {:<8.0f} {:<8.0f} {:<8.2f} {:<8.2f} {:<8.2f} {:<8.2f}'.format(error, 1/dt, I, P*kp, I*ki, D*kd, (PID)))
        
        # Return values 
        return PID, I

    def positionControl(self, actual, desired):
        # Error calculations
        errorNorth = desired[0] - actual[0]
        errorEast  = desired[1] - actual[1]
        errorDown  = desired[2] - actual[2]
        errorYaw   = desired[3] - actual[3]

        # Get time delta
        dt = time.time() - self.timer
        self.timer = time.time()

        # Gain scheduling
        if actual[2] < self.gainHeight:
            tempKp = self.kp_DOWN * self.gainFactor
        else:
            tempKp = self.kp_DOWN
            
        # Calculate thrust control
        thrustControl, self.downI = self.PID(errorDown, self.downPrevError, self.downI, dt, tempKp, self.ki_DOWN, self.kd_DOWN)
                
        # Run the remainder of the control
        rollControl, self.eastI   = self.PID(errorEast, self.eastPrevError, self.eastI, dt, self.kp_EAST, self.ki_EAST, self.kd_EAST)
        pitchControl, self.northI = self.PID(errorNorth, self.northPrevError, self.northI, dt, self.kp_NORTH, self.ki_NORTH, self.kd_NORTH)
        yawControl, self.yawI     = self.PID(errorYaw, self.yawPrevError, self.yawI, dt, self.kp_YAW, self.ki_YAW, self.kd_YAW)
        
        # Constrain I terms to prevent integral windup
        self.northI = self.constrain(self.northI, self.northIcontstrain[0], self.northIcontstrain[1])
        self.eastI  = self.constrain(self.eastI, self.eastIcontstrain[0], self.eastIcontstrain[1]) 
        self.downI  = self.constrain(self.downI, self.downIcontstrain[0], self.downIcontstrain[1])
        self.yawI   = self.constrain(self.yawI, self.yawIcontstrain[0], self.yawIcontstrain[1])

        # Update previous error
        self.northPrevError = errorNorth
        self.eastPrevError  = errorEast
        self.downPrevError  = errorDown
        self.yawPrevError   = errorYaw

        # Constrain the controller values
        rollAngle  = self.constrain(rollControl, self.rollConstrain[0], self.rollConstrain[1])
        pitchAngle = self.constrain(pitchControl, self.pitchConstrain[0], self.pitchConstrain[1])
        thrust     = self.constrain(thrustControl, self.thrustConstrain[0], self.thrustConstrain[1])
        yawRate    = self.constrain(yawControl, self.yawRateConstrain[0], self.yawRateConstrain[1])

        # Inverse direction of controller if required
        rollAngle  = rollAngle
        pitchAngle = -pitchAngle
        thrust     = thrust + 0.5
        yawRate    = -yawRate

        # Mixer -> Psi: SITL (+), actual (-)
        psi = -math.radians(actual[3])
        pitchAngleTemp = pitchAngle*math.cos(psi) - rollAngle*math.sin(psi)
        rollAngleTemp = pitchAngle*math.sin(psi) + rollAngle*math.cos(psi)
        pitchAngle = pitchAngleTemp
        rollAngle = rollAngleTemp

        # Check if UAV has landed
        if ((abs(errorNorth) < self.landErrorNE) and (abs(errorEast) < self.landErrorNE) and 
                (actual[2] < self.landHeight) and (self.landCount >= self.landCountRequired)):
            landState = True
        elif ((abs(errorNorth) < self.landErrorNE) and (abs(errorEast) < self.landErrorNE) and 
                (actual[2] < self.landHeight)):
            self.landCount += 1
            landState = False
        else:
            landState = False

        # Return the values
        return rollAngle, pitchAngle, yawRate, thrust, landState
