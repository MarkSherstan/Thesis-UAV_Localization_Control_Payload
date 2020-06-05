import time
import math

class Controller:
    def __init__(self, northDesired, eastDesired, downDesired, vehicle, yawDesired=0):
        # Vehicle class
        self.UAV = vehicle
        self.msgCounter = 0 
        self.msgTimer = None
        
        # Desired position
        self.northDesired = northDesired
        self.eastDesired  = eastDesired
        self.downDesired  = downDesired
        self.yawDesired   = yawDesired

        # Constraints
        self.minValNE =   -10	# Deg
        self.maxValNE =    10   # Deg
        self.minValD =	    0	# Normalized
        self.maxValD =      1	# Normalized
        self.minYawRate = -10	# Deg/s
        self.maxYawRate =  10	# Deg/s

        # PID Gains: NORTH
        self.kp_NORTH = 0.008
        self.ki_NORTH = 0.002
        self.kd_NORTH = 0.010

        # PID Gains: EAST
        self.kp_EAST = 0.009
        self.ki_EAST = 0.002
        self.kd_EAST = 0.010

        # PID Gains: DOWN
        self.kp_DOWN = 0.0006
        self.ki_DOWN = 0.0006
        self.kd_DOWN = 0.0002

        # PID Gains: YAW
        self.kp_YAW = 0.080
        self.ki_YAW = 0.004
        self.kd_YAW = 0.040

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

        # Timing
        self.timer = None
    
    def startTimers(self):
        self.msgTimer = time.time()
        self.timer = time.time()
    
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
        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        # Set yaw to current yaw value 
        #                                       (try changing this later to just 0)
        yaw = self.UAV.attitude.yaw
        yawRate = math.radians(yawRate)

        # Create the mavlink message
        msg = self.UAV.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            0, # Target system
            0, # Target component
            0b00000000, # If bit is set corresponding input ignored (mappings)
            self.euler2quaternion(roll, pitch, yaw), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            yawRate,   # Body yaw rate in rad/s
            thrustRate # Thrust
        )
        
        # Send the constructed message
        self.UAV.send_mavlink(msg)
        
        # Log messages sent
        self.msgCounter += 1
    
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

    def positionControl(self, northActual, eastActual, downActual, yawActual):
        # Error calculations
        errorNorth = (self.northDesired - northActual)
        errorEast  = (self.eastDesired - eastActual)
        errorDown  = (self.downDesired - downActual)
        errorYaw   = (self.yawDesired - yawActual)

        # Get time delta
        dt = time.time() - self.timer
        self.timer = time.time()

        # Run some control
        rollControl, self.eastI = self.PID(errorEast, self.eastPrevError, self.eastI, dt, self.kp_EAST, self.ki_EAST, self.kd_EAST)
        pitchControl, self.northI = self.PID(errorNorth, self.northPrevError, self.northI, dt, self.kp_NORTH, self.ki_NORTH, self.kd_NORTH)
        thrustControl, self.downI = self.PID(errorDown, self.downPrevError, self.downI, dt, self.kp_DOWN, self.ki_DOWN, self.kd_DOWN)
        yawControl, self.yawI = self.PID(errorYaw, self.yawPrevError, self.yawI, dt, self.kp_YAW, self.ki_YAW, self.kd_YAW)
        
        # Constrain I terms to prevent integral windup
        # self.northI = self.constrain(self.northI, self.minValNE/2, self.maxValNE/2) 
        # self.eastI  = self.constrain(self.eastI, self.minValNE/2, self.maxValNE/2)
        # self.downI  = self.constrain(self.downI, self.min)
        # self.yawI   = self.constrain(self.yawI, )

        # Update previous error
        self.northPrevError = errorNorth
        self.eastPrevError = errorEast
        self.downPrevError = errorDown
        self.yawPrevError = yawActual

        # Set and constrain the controller values
        rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
        pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
        thrust = self.constrain(thrustControl, self.minValD, self.maxValD)
        yawRate = self.constrain(yawControl, self.minYawRate, self.maxYawRate)

        # Send the values
        self.sendAttitudeTarget(rollAngle, -pitchAngle, yawRate, thrust)

    def close(self):
        print('Message rate: ', round((self.msgCounter / (time.time() - self.msgTimer))))