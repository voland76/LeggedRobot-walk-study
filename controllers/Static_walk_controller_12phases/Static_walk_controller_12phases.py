"""Static_walk_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import sin, atan , sqrt, acos, pi



# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())
phase_duration = {}
phase_duration ['phase 1'] = timestep/1000 *30 
T1 = timestep/1000 *10 
T2 = T1 + timestep/1000 * 10 
T3 = T2 + timestep/1000 * 10
T4 = T3 + timestep/1000 * 10
T5 = T4 + timestep/1000 * 10
T6 = T5 + timestep/1000 * 20 # CoM moving 
T7 = T6 + timestep/1000 * 10
T8 = T7 + timestep/1000 * 10 # leg moving
T9 = T8 + timestep/1000 * 10
T10 = T9 + timestep/1000 * 20 # CoM moving
T11 = T10 + timestep/1000 * 10
T12 = T11 + timestep/1000 * 10 # leg moving
L0 = 0.05
L = 0.14
STEP = 0.1

dofs = {}
dofs['left_hip_yaw'] = robot.getDevice('left_hip_yaw')
dofs['left_hip_pitch'] = robot.getDevice('left_hip_pitch')
dofs['left_hip_roll'] = robot.getDevice('left_hip_roll')
dofs['left_knee'] = robot.getDevice('left_knee')
dofs['left_ankle_pitch'] = robot.getDevice('left_ankle_pitch')
dofs['left_ankle_roll'] = robot.getDevice('left_ankle_roll')
dofs['right_hip_yaw']  = robot.getDevice('right_hip_yaw')
dofs['right_hip_pitch'] = robot.getDevice('right_hip_pitch')
dofs['right_hip_roll'] = robot.getDevice('right_hip_roll')
dofs['right_knee'] = robot.getDevice('right_knee')
dofs['right_ankle_pitch'] = robot.getDevice('right_ankle_pitch')
dofs['right_ankle_roll'] = robot.getDevice('right_ankle_roll')

def IK_solver(CoM, foot):
    B = sqrt((CoM[0]-foot[0])**2 +
             (CoM[2]-foot[2])**2)
    z = CoM[2] - foot[2]
    if B > 2*L: B = 2*L
    ϴ1 = ( acos( z/B ) + acos( B/(2*L) )) - pi
    if (foot [0]- CoM [0]) >= 0: ϴ1 = ( acos( z/B ) + acos( B/(2*L) )) 
    else:                        ϴ1 = (-acos( z/B ) + acos( B/(2*L) ))  
    ϴ2 = pi- acos( (L*L + L*L - B*B) / (2*L*L) )
    
    return ϴ1, Θ2

def send_commands(servos_pos):
    for motor, pos in servos_pos.items():
        dofs[motor].setPosition(pos)

def compute_positions(time):
    servos_pos = {}
    H = 2*L
    dH = 0.02

    # phase 1. making squat
    if time < T1 :
        H = H-dH*time/T1
        CoM = [0, 0, H]
        LF = [0, 0.1, 0.0]
        RF = [0, -0.1, 0.0]
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha
        servos_pos['right_hip_pitch'] =alpha
        servos_pos['right_knee'] =beta
        servos_pos['right_ankle_pitch'] = beta-alpha
    else:
        H = H - dH
        ROLL = atan (L0/H)
   
    # phase 2. moving CoM into right foot reference polygon
    if T1 <= time < T2:
        CoM = [0, -L0*time/T1, H]
        servos_pos['left_hip_roll'] = -ROLL * (time-T1)/(T2-T1)
        servos_pos['left_ankle_roll'] = ROLL * (time-T1)/(T2-T1)
        servos_pos['right_hip_roll'] = -ROLL * (time-T1)/(T2-T1)
        servos_pos['right_ankle_roll'] = ROLL * (time-T1)/(T2-T1)
    
    # phase 3. Rise left foot
    if T2 <= time < T3:
        CoM = [0, 0, H]
        LF = [0, 0.1, 0.03]
        # LF = [0, 0.1, 0.03*(time-T2)/(T3-T2)] # if you want make it slower
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha 
    
    # phase 4. move left foot to step destination
    if T3 <= time < T4:
        CoM = [0, 0, H]
        LF = [STEP * (time-T3)/(T4-T3), 0.1, 0.03 ]
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha

    # phase 5. putting left foot on the ground
    if T4 <= time < T5:
        CoM = [0, 0, H]
        LF = [STEP, 0.1, 0.03 - 0.03*(time-T4)/(T5-T4)]
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha

    # phase 6. moving CoM to left foot reference polygon
    if T5 <= time < T6:
        CoM = [STEP* (time-T5)/(T6-T5), 0, H]
        LF = [STEP, 0.1, 0.0]
        RF = [0.0, -0.1, 0.0]
        alphaLF, betaLF = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alphaLF
        servos_pos['left_knee'] = betaLF
        servos_pos['left_ankle_pitch'] = betaLF-alphaLF

        alphaRF, betaRF = IK_solver(CoM,RF)
        servos_pos['right_hip_pitch'] = alphaRF
        servos_pos['right_knee'] = betaRF
        servos_pos['right_ankle_pitch'] = betaRF-alphaRF

        servos_pos['left_ankle_roll'] = ROLL - 2*ROLL*(time-T5)/(T6-T5)
        servos_pos['left_hip_roll'] = 2*ROLL*(time-T5)/(T6-T5) - ROLL
        servos_pos['right_hip_roll'] = 2*ROLL*(time-T5)/(T6-T5) - ROLL
        servos_pos['right_ankle_roll'] = ROLL - 2*ROLL*(time-T5)/(T6-T5)
        
    # phase 7 Rise right foot
    if T6 <= time < T7:
        CoM = [STEP, 0, H]
        RF = [0, -0.1, 0.03]
        alpha, beta = IK_solver(CoM,RF)
        servos_pos['right_hip_pitch'] = alpha
        servos_pos['right_knee'] = beta
        servos_pos['right_ankle_pitch'] = beta-alpha

    # phase 8. move right foot to step destination
    if T7 <= time < T8:
        CoM = [STEP, 0, H]
        RF = [2*STEP*(time-T7)/(T8-T7), -0.1, 0.03]  # increasing x of RF slowly
        alpha, beta = IK_solver(CoM,RF)
        servos_pos['right_hip_pitch'] = alpha
        servos_pos['right_knee'] = beta
        servos_pos['right_ankle_pitch'] = beta-alpha 
        
    # phase 9. putting right foot on the ground
    if T8 <= time < T9:
        CoM = [STEP, 0, H]
        RF = [2*STEP, 0-0.1, 0.03-0.03*(time-T8)/(T9-T8)] 
        alpha, beta = IK_solver(CoM,RF)
        servos_pos['right_hip_pitch'] = alpha
        servos_pos['right_knee'] = beta
        servos_pos['right_ankle_pitch'] = beta-alpha  

    # phase 10. moving CoM to right foot reference polygon
    if T9 <= time < T10:
        CoM = [STEP+STEP * (time-T9)/(T10-T9), 0, H]
        LF = [STEP,  0.1, 0.0]
        RF = [2*STEP, -0.1, 0.0]
        alphaLF, betaLF = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alphaLF
        servos_pos['left_knee'] = betaLF
        servos_pos['left_ankle_pitch'] = betaLF-alphaLF

        alphaRF, betaRF = IK_solver(CoM,RF)
        servos_pos['right_hip_pitch'] = alphaRF
        servos_pos['right_knee'] = betaRF
        servos_pos['right_ankle_pitch'] = betaRF-alphaRF

        servos_pos['left_ankle_roll'] = 2*ROLL*(time-T9)/(T10-T9) - ROLL
        servos_pos['left_hip_roll'] = ROLL - 2*ROLL*(time-T9)/(T10-T9)
        servos_pos['right_hip_roll'] = ROLL - 2*ROLL*(time-T9)/(T10-T9)
        servos_pos['right_ankle_roll'] = 2*ROLL*(time-T9)/(T10-T9) - ROLL 

    # phase 11. Rise left foot
    if T10 <= time < T11:
        CoM = [2*STEP, 0, H]
        LF = [STEP, 0.1, 0.03]
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha

    # phase 12. move left foot to step destination
    if T11 <= time < T12:
        CoM = [2*STEP, 0, H]
        LF = [STEP + 2*STEP*(time-T11)/(T12-T11), 0.1, 0.03] # 0.1 + 0.2*(time-T11)/(T12-T11)
        alpha, beta = IK_solver(CoM,LF)
        servos_pos['left_hip_pitch'] = alpha
        servos_pos['left_knee'] = beta
        servos_pos['left_ankle_pitch'] = beta-alpha    

    # phase 12 have same posture at the end of the phase as phase 5
    # We can repeat (phase6-phase12) as a cycle it if we want to walk further  
    
    return servos_pos

time = 0

# Main loop:
while robot.step(timestep) != -1:
    time += timestep/1000
    if time > T12: time = T4
    servos_pos = compute_positions(time)
    send_commands(servos_pos)