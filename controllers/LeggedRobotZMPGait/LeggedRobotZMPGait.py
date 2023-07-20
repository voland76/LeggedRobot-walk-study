"""ik_walk_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import GPS
from math import pi, sin, atan, cos, asin

# create the Robot instance.
robot = Robot()
# Robot parametres
L = 0.14 # thigh and shin length are equal
L0 = 0.05 # from CoM to hip
dH = 0.05 # from ground to ankle motors

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
stepTime = 0.32
halfStepsize = 0.05
legLength = 2*L
theta = asin(halfStepsize/legLength) # max hip pitch angle at the start and the end of the step 

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
dofs = {}
motor = robot.getDevice('left_hip_yaw')
base_pos_sensor = robot.getDevice('gps')
com_sensor = robot.getDevice('gps2')
com_sensor.enable(32)
dofs['left_hip_yaw'] = motor
dofs['right_hip_yaw']  = robot.getDevice('right_hip_yaw')

dofs['right_hip_pitch'] = robot.getDevice('right_hip_pitch')
dofs['left_hip_pitch'] = robot.getDevice('left_hip_pitch')
dofs['right_hip_roll'] = robot.getDevice('right_hip_roll')
dofs['left_hip_roll'] = robot.getDevice('left_hip_roll')

dofs['right_knee'] = robot.getDevice('right_knee')
dofs['left_knee'] = robot.getDevice('left_knee')

dofs['right_ankle_pitch'] = robot.getDevice('right_ankle_pitch')
dofs['left_ankle_pitch'] = robot.getDevice('left_ankle_pitch')
dofs['right_ankle_roll'] = robot.getDevice('right_ankle_roll')
dofs['left_ankle_roll'] = robot.getDevice('left_ankle_roll')


base_pos_sensor.enable(timestep)



def compute_ik_for_leg(com_target, foot_target, t):
    servos_pos = {}
    
    # swinging in latheral plane
    sin_value = sin (pi*t/stepTime)
    y_CoM = 0.7*L0*sin_value 
    alpha = atan (y_CoM/(2*L))
    beta = 0.8*sin_value
    if k == 0:
        sin_value = sin (0.333*pi*t/stepTime)
        y_CoM = 0.5*L0*sin_value
        alpha = atan (y_CoM/(2*L))
        beta = 0.8*sin_value
        servos_pos['right_hip_roll'] = alpha
        servos_pos['right_ankle_roll'] = -alpha
        servos_pos['left_hip_roll'] = alpha
        servos_pos['left_ankle_roll'] = -alpha
    elif k == 1:
        if t<stepTime/2 :sin_value = sin (0.333*pi*t/stepTime+2*pi/3)
        else: sin_value = sin (pi*(t)/stepTime)
        y_CoM = 0.5*L0*sin_value
        alpha = atan (y_CoM/(2*L))
        beta = 0.8*sin_value
        servos_pos['right_hip_roll'] = alpha
        servos_pos['right_ankle_roll'] = -alpha
        servos_pos['left_hip_roll'] = alpha
        servos_pos['left_ankle_roll'] = -alpha
        servos_pos['right_knee'] = 2*beta
        servos_pos['right_hip_pitch'] = beta 
        servos_pos['right_ankle_pitch'] = beta
        print (t, sin_value)
    else:
        if leftStance:
            servos_pos['right_hip_roll'] = alpha
            servos_pos['right_ankle_roll'] = -alpha
            servos_pos['left_hip_roll'] = alpha
            servos_pos['left_ankle_roll'] = -alpha
            servos_pos['right_knee'] = 2*beta
            servos_pos['right_hip_pitch'] = beta
            servos_pos['right_ankle_pitch'] = beta
        else:
            servos_pos['right_hip_roll'] = -alpha
            servos_pos['right_ankle_roll'] = alpha
            servos_pos['left_hip_roll'] = -alpha
            servos_pos['left_ankle_roll'] = alpha
            servos_pos['left_knee'] = 2*beta
            servos_pos['left_hip_pitch'] = beta
            servos_pos['left_ankle_pitch'] = beta


    # print (leftStance)


    return servos_pos

def send_commands(servos_pos):
    for motor, pos in servos_pos.items():
        dofs[motor].setPosition(pos)

timer = -timestep
stepTimer = -timestep/1000
leftStance = True

startup_servos_pos = {}
startup_knee_ankle = 0.0
startup_servos_pos['right_hip_pitch'] = startup_knee_ankle/2
startup_servos_pos['right_knee'] = startup_knee_ankle
startup_servos_pos['right_ankle_pitch'] = startup_knee_ankle/2
startup_servos_pos['left_hip_pitch'] = startup_knee_ankle/2
startup_servos_pos['left_knee'] = startup_knee_ankle
startup_servos_pos['left_ankle_pitch'] = startup_knee_ankle/2


send_commands(startup_servos_pos)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    timer += timestep
    if timer == 640: break
    pass
timer = -timestep
k = 0
while robot.step(timestep) != -1:
    timer += timestep
    stepTimer += timestep/1000
    if stepTimer > stepTime:
        k += 1
        stepTimer = 0
        leftStance = not leftStance
        if k == 1: leftStance = not leftStance
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    base_pos = base_pos_sensor.getValues()
    com_pos = com_sensor.getValues() 
    # print(f"TIME: {timer} POS: {base_pos} {com_pos = }")
    # Process sensor data here.
    servos_pos = compute_ik_for_leg(1, 2, stepTimer)
    send_commands(servos_pos)
    # Enter here functions to send actuator commands, like:
    
    pass

# Enter here exit cleanup code.
