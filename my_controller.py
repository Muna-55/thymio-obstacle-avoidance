from controller import Robot
import math

robot = Robot()
timeStep = int(robot.getBasicTimeStep())


# Parameters

maxMotorVelocity = 9.53
baseSpeed = 3.0
turnSpeed = 4.0
max_sensor = 4000.0

TURN_STEPS = 10
SIDE_DIFF = 0.10

FRONT_CLEAR = 0.60
FRONT_CAUTION = 0.35
SIDE_CLEAR = 0.22

PROX_FRONT_EMERGENCY = 0.70
PROX_SIDE_ALERT = 0.50


# Odometry / Goal parameters

# Tune these 
WHEEL_RADIUS = 0.021      # meters
AXLE_LENGTH = 0.095       # meters

# Start pose from your .wbt file
START_X = -1.67496
START_Y = 0.207924
START_THETA = -0.18021530717958623

# Pre-entry goal pose near upper corridor entrance
GOAL_X = 1.20#it was 1.3
GOAL_Y = 0.31
GOAL_THETA = 0.0          

# Tolerances suggested by doctor
POS_TOL = 0.06           # it was 4 cm
ANGLE_TOL = math.radians(5.0) #it was 3

# When robot gets near corridor region, switch to pose mode
SWITCH_X = 0.90

# Distance to drive straight into finish area after reaching pose
ENTER_DISTANCE = 0.30


# Helper functions

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def compass_heading():
    north = compass.getValues()
    return math.atan2(north[0], north[2])


# Motors

leftMotor = robot.getDevice("motor.left")
rightMotor = robot.getDevice("motor.right")

leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


# Encoders

leftEncoder = leftMotor.getPositionSensor()
rightEncoder = rightMotor.getPositionSensor()
leftEncoder.enable(timeStep)
rightEncoder.enable(timeStep)


# Proximity sensors

outerLeftSensor = robot.getDevice("prox.horizontal.0")
centralLeftSensor = robot.getDevice("prox.horizontal.1")
centralSensor = robot.getDevice("prox.horizontal.2")
centralRightSensor = robot.getDevice("prox.horizontal.3")
outerRightSensor = robot.getDevice("prox.horizontal.4")

sensors = [
    outerLeftSensor,
    centralLeftSensor,
    centralSensor,
    centralRightSensor,
    outerRightSensor
]

for sensor in sensors:
    sensor.enable(timeStep)


# Lidar

lidar = robot.getDevice("lidar")
lidar.enable(timeStep)


# Compass

compass = robot.getDevice("compass")
compass.enable(timeStep)


# Pose state

x = START_X
y = START_Y
theta = START_THETA

prevLeft = 0.0
prevRight = 0.0


# Turn memory

turn_timer = 0
turn_direction = None  # "left" or "right"


# High-level mode

mode = "AVOID"  # AVOID, GO_TO_POSE, ENTER_GOAL, STOP
enter_start_x = 0.0
enter_start_y = 0.0


# Wait one step so encoders are valid

robot.step(timeStep)
prevLeft = leftEncoder.getValue()
prevRight = rightEncoder.getValue()
theta = compass_heading()


# Main loop

while robot.step(timeStep) != -1:
    
    # Read proximity sensors
    
    s0 = min(outerLeftSensor.getValue() / max_sensor, 1.0)
    s1 = min(centralLeftSensor.getValue() / max_sensor, 1.0)
    s2 = min(centralSensor.getValue() / max_sensor, 1.0)
    s3 = min(centralRightSensor.getValue() / max_sensor, 1.0)
    s4 = min(outerRightSensor.getValue() / max_sensor, 1.0)

    
    # Read lidar
    
    ranges = lidar.getRangeImage()
    valid_ranges = [r for r in ranges if math.isfinite(r)]

    if len(valid_ranges) == 0:
        left_dist = float("inf")
        front_dist = float("inf")
        right_dist = float("inf")
    else:
        n = len(ranges)

        
        left_ranges = ranges[:n // 3]
        front_ranges = ranges[n // 3: 2 * n // 3]
        right_ranges = ranges[2 * n // 3:]

        left_valid = [r for r in left_ranges if math.isfinite(r)]
        front_valid = [r for r in front_ranges if math.isfinite(r)]
        right_valid = [r for r in right_ranges if math.isfinite(r)]

        left_dist = min(left_valid) if left_valid else float("inf")
        front_dist = min(front_valid) if front_valid else float("inf")
        right_dist = min(right_valid) if right_valid else float("inf")

    
    # Update pose from encoders + compass
    
    leftNow = leftEncoder.getValue()
    rightNow = rightEncoder.getValue()

    dLeft = (leftNow - prevLeft) * WHEEL_RADIUS
    dRight = (rightNow - prevRight) * WHEEL_RADIUS

    prevLeft = leftNow
    prevRight = rightNow

    dCenter = 0.5 * (dLeft + dRight)

    # compass for heading because it is more reliable
    theta = compass_heading()
    theta_deg = (theta * 180.0 / math.pi) % 360.0

    x += dCenter * math.cos(theta)
    y += dCenter * math.sin(theta)

    print("mode:", mode)
    print("pose:", round(x, 3), round(y, 3), round(theta_deg, 1))
    print("prox:", round(s0, 2), round(s1, 2), round(s2, 2), round(s3, 2), round(s4, 2))
    print("lidar:", round(left_dist, 3), round(front_dist, 3), round(right_dist, 3))

    leftSpeed = 0.0
    rightSpeed = 0.0

    
    # Mode 1: Obstacle avoidance
    
    if mode == "AVOID":
        # If already turning, keep turning
        if turn_timer > 0:
            turn_timer -= 1

            if turn_direction == "right":
                leftSpeed = turnSpeed
                rightSpeed = -turnSpeed
                print("turning right (memory)")
            elif turn_direction == "left":
                leftSpeed = -turnSpeed
                rightSpeed = turnSpeed
                print("turning left (memory)")

        else:
            # Emergency avoidance
            if front_dist < FRONT_CAUTION or s2 > PROX_FRONT_EMERGENCY:
                if left_dist > right_dist:
                    leftSpeed = -turnSpeed
                    rightSpeed = turnSpeed
                    turn_direction = "left"
                    print("emergency turn left")
                else:
                    leftSpeed = turnSpeed
                    rightSpeed = -turnSpeed
                    turn_direction = "right"
                    print("emergency turn right")

                turn_timer = TURN_STEPS

            # Side emergency from proximity
            elif s1 > PROX_SIDE_ALERT and s3 <= PROX_SIDE_ALERT:
                leftSpeed = turnSpeed
                rightSpeed = -turnSpeed
                turn_direction = "right"
                turn_timer = TURN_STEPS
                print("prox avoid right")

            elif s3 > PROX_SIDE_ALERT and s1 <= PROX_SIDE_ALERT:
                leftSpeed = -turnSpeed
                rightSpeed = turnSpeed
                turn_direction = "left"
                turn_timer = TURN_STEPS
                print("prox avoid left")

            # Clear path
            elif front_dist > FRONT_CLEAR and left_dist > SIDE_CLEAR and right_dist > SIDE_CLEAR:
                leftSpeed = baseSpeed
                rightSpeed = baseSpeed
                print("going forward")

            # Choose side with more space
            elif right_dist - left_dist > SIDE_DIFF:
                leftSpeed = turnSpeed
                rightSpeed = -turnSpeed
                turn_direction = "right"
                turn_timer = TURN_STEPS
                print("choosing right")

            elif left_dist - right_dist > SIDE_DIFF:
                leftSpeed = -turnSpeed
                rightSpeed = turnSpeed
                turn_direction = "left"
                turn_timer = TURN_STEPS
                print("choosing left")

            # Fallback
            else:
                if left_dist >= right_dist:
                    leftSpeed = -turnSpeed
                    rightSpeed = turnSpeed
                    turn_direction = "left"
                    print("fallback left")
                else:
                    leftSpeed = turnSpeed
                    rightSpeed = -turnSpeed
                    turn_direction = "right"
                    print("fallback right")

                turn_timer = TURN_STEPS

        # Switch to goal-seeking near corridor
        if x > SWITCH_X:
            mode = "GO_TO_POSE"
            turn_timer = 0
            print("Switching to GO_TO_POSE")

    
    # Mode 2: Go to chosen pose near entrance
    
    elif mode == "GO_TO_POSE":
        # Safety: if obstacle suddenly appears, temporarily avoid it
        if front_dist < FRONT_CAUTION or s2 > PROX_FRONT_EMERGENCY:
            mode = "AVOID"
            turn_timer = TURN_STEPS
            if left_dist > right_dist:
                turn_direction = "left"
            else:
                turn_direction = "right"
            print("Obstacle in GO_TO_POSE -> back to AVOID")

        else:
            dx = GOAL_X - x
            dy = GOAL_Y - y
            dist_error = math.sqrt(dx * dx + dy * dy)

            target_bearing = math.atan2(dy, dx)
            bearing_error = normalize_angle(target_bearing - theta)
            heading_error = normalize_angle(GOAL_THETA - theta)

            print("dist_error:", round(dist_error, 3),
                  "bearing_error_deg:", round(math.degrees(bearing_error), 1),
                  "heading_error_deg:", round(math.degrees(heading_error), 1))

            # First reach target position
            if dist_error > POS_TOL:
                # Rotate first if heading to target is too far off
                if abs(bearing_error) > math.radians(12):# it was 12
                    if bearing_error > 0:
                        leftSpeed = -turnSpeed
                        rightSpeed = turnSpeed
                        print("GO_TO_POSE: rotate left")
                    else:
                        leftSpeed = turnSpeed
                        rightSpeed = -turnSpeed
                        print("GO_TO_POSE: rotate right")
                else:
                    # Forward with steering correction
                    k_turn = 1.2# it was 3.0
                    forward = 2.5
                    correction = k_turn * bearing_error

                    leftSpeed = forward - correction
                    rightSpeed = forward + correction
                    print("GO_TO_POSE: drive to target")

            # Then align final orientation
            elif abs(heading_error) > ANGLE_TOL:
                if heading_error > 0:
                    leftSpeed = -turnSpeed
                    rightSpeed = turnSpeed
                    print("GO_TO_POSE: final align left")
                else:
                    leftSpeed = turnSpeed
                    rightSpeed = -turnSpeed
                    print("GO_TO_POSE: final align right")

            # Pose reached
            else:
                mode = "ENTER_GOAL"
                enter_start_x = x
                enter_start_y = y
                leftSpeed = 0.0
                rightSpeed = 0.0
                print("Pose reached -> ENTER_GOAL")

    
    # Mode 3: Drive straight into finish area
    
    elif mode == "ENTER_GOAL":
        # Safety: if something blocks the entrance, go back to avoid
        if front_dist < FRONT_CAUTION or s2 > PROX_FRONT_EMERGENCY:
            mode = "AVOID"
            turn_timer = TURN_STEPS
            if left_dist > right_dist:
                turn_direction = "left"
            else:
                turn_direction = "right"
            print("Obstacle in ENTER_GOAL -> back to AVOID")

        else:
            entered_dist = math.sqrt((x - enter_start_x) ** 2 + (y - enter_start_y) ** 2)
            heading_error = normalize_angle(GOAL_THETA - theta)

            print("entered_dist:", round(entered_dist, 3),
                  "heading_error_deg:", round(math.degrees(heading_error), 1))

            if entered_dist < ENTER_DISTANCE:
                k_heading = 2.0
                forward = 2.5
                correction = k_heading * heading_error

                leftSpeed = forward - correction
                rightSpeed = forward + correction
                print("ENTER_GOAL: driving straight")
            else:
                mode = "STOP"
                leftSpeed = 0.0
                rightSpeed = 0.0
                print("Goal entered -> STOP")

   
    # Mode 4: Stop
    
    elif mode == "STOP":
        leftSpeed = 0.0
        rightSpeed = 0.0
        print("stopped in finish area")

    
    # Clamp and apply
   
    leftSpeed = clamp(leftSpeed, -maxMotorVelocity, maxMotorVelocity)
    rightSpeed = clamp(rightSpeed, -maxMotorVelocity, maxMotorVelocity)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
