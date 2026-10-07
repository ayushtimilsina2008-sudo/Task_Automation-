"""
Webots Industrial Automation Core - UR5e Manipulator Python State Engine
"""
from controller import Robot

# Initialize system core and standard time step configuration
robot = Robot()
TIME_STEP = 32

# Track execution state modes
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

state = WAITING
counter = 0

# 1. Match the 3-finger gripper hardware strings from your C code
hand_joint_names = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
hand_motors = []
for name in hand_joint_names:
    motor = robot.getDevice(name)
    if motor:
        hand_motors.append(motor)

# 2. Match the 4 structural arm joint motor strings from your C code
ur_joint_names = ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
ur_motors = []
for name in ur_joint_names:
    motor = robot.getDevice(name)
    if motor:
        motor.setVelocity(1.0) # Sets baseline operational velocity speed
        ur_motors.append(motor)

# 3. Configure sensory hardware attachments
distance_sensor = robot.getDevice("distance sensor")
if distance_sensor:
    distance_sensor.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
if position_sensor:
    position_sensor.enable(TIME_STEP)

# 4. Fixed spatial target coordinate arrays from the operational matrix
target_positions = [-1.88, -2.14, -2.38, -1.51]

print("--------------------------------------------------")
print("  >>> UR5E PYTHON STATE ENGINE RUNTIME ACTIVE <<< ")
print("--------------------------------------------------")

# Primary real-time simulation execution loop
while robot.step(TIME_STEP) != -1:
    if counter <= 0:
        
        if state == WAITING:
            # Check proximity sensor to see if a red can has arrived on the belt (< 500 units)
            if distance_sensor and distance_sensor.getValue() < 500.0:
                print("Grasping can")
                state = GRASPING
                counter = 8
                # Clamp down the 3 industrial fingers securely onto the can
                for motor in hand_motors:
                    motor.setPosition(0.85)
                    
        elif state == GRASPING:
            print("Rotating arm")
            # Shift the 4 structural arm joints to drop vectors simultaneously
            for i in range(len(ur_motors)):
                ur_motors[i].setPosition(target_positions[i])
            state = ROTATING
            
        elif state == ROTATING:
            # Monitor wrist sensory parameters to confirm drop destination is reached (< -2.3)
            if position_sensor and position_sensor.getValue() < -2.3:
                print("Releasing can")
                counter = 8
                state = RELEASING
                # Fully open claw parameters using lowest design boundaries
                for motor in hand_motors:
                    motor.setPosition(motor.getMinPosition())
                    
        elif state == RELEASING:
            print("Rotating arm back")
            # Zero out arm configurations to align with home position (0.0)
            for motor in ur_motors:
                motor.setPosition(0.0)
            state = ROTATING_BACK
            
        elif state == ROTATING_BACK:
            # Wait until position feedback matches home boundaries (> -0.1) to run next cycle
            if position_sensor and position_sensor.getValue() > -0.1:
                print("Waiting can")
                state = WAITING

    counter -= 1
