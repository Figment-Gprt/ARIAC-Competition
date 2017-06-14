"""
    All commands that the arm will execute in the simulated world will be summarized here.


"""


import rospy
import utils
import global_vars
import math
import sys

from constants import *
from trajectory_msgs.msg import JointTrajectory
from trianglesolver import solve, degree
from copy import deepcopy

from ik_solution import solverBin, solverBelt, depositOnTray2, depositOnTray1
import gripper_actions


class SolverType:
    BIN, AGV1, AGV2, BELT = range(4)


def init():
    """
    Initialize the global vars needed for execution
    """
    global joint_trajectory_publisher
    joint_trajectory_publisher = rospy.Publisher(
        "/ariac/arm/command", JointTrajectory, queue_size=5)


def go_to_initial_position():
    """
    Move the arm to the initial position defined statically in constants
    """
    set_arm_joint_values(list_of_joint_values=STATIC_POSITIONS[
                         "initial_position"], time_to_execute_action=0.5)

def go_to_tray_position(tray_id, time=2):
    """
    Move the arm to static position in front of the tray
    tray_id - key for the wanted tray to be used in the STATIC_POSITIONS MAP.
    """

    set_arm_joint_values(STATIC_POSITIONS[tray_id], time)

    return STATIC_POSITIONS[tray_id]

def go_to_belt_start():
    """
    Move to the static positon in front of the belt.

    """    
    rest_position_start = STATIC_POSITIONS["beltStart"]
    #linear_arm_actuator_joint = part.y

    set_arm_joint_values(rest_position_start, 1)
    return rest_position_start

def go_to_belt():
    """
    Move to the static positon in front of the belt.

    """   

    rest_position = STATIC_POSITIONS["belt"]
    #linear_arm_actuator_joint = part.y

    set_arm_joint_values(rest_position, 1)
    return rest_position

def go_to_part_bin_front(part_world_position):
    """
    Move to the front of the wanted part.
    part_world_position : world coordinates of the part position.

    """
    rest_position = STATIC_POSITIONS["rest_position"]
    #linear_arm_actuator_joint = part.y

    rest_position[1] = part_world_position[1] + WRIST_LENGTH

    set_arm_joint_values(rest_position, 1)
    return rest_position

def go_to_bin_front(bin_id):
    """
    Move to the front of the wanted bin_id.
    bin_id : key for the wanted bin to be used in the STATIC_POSITIONS MAP.
    """
    
    set_arm_joint_values(STATIC_POSITIONS[bin_id], 1)


def go_to_position_a_bit_above_part(world_position, world_orientation, part_type, time_to_execute_action, solver_type, a_bit_above_value=0.015, ignore_height=False, adjust=False):
    """
        Move the arm to a position a little bit above the part.

        world_position - world coordinates of the part position
        world_orientation - world orientation (rotation) of the part
        part_type - part type (piston_rod, gear, ...)
        time_to_execute_action - time to execute the moviment
        solver_type - value to select which solver to use (tray, bin, agv1, agv2)
        a_bit_above_value - distance value that the arm will be above the part

    """
    world_position_above_part = [world_position[0], world_position[1], world_position[2] + a_bit_above_value]

    rospy.loginfo("[go_to_position_a_bit_above_part] world_position_above_part = " + str(world_position_above_part))

    angles = []
    if solver_type == SolverType.BIN:
        angles = solverBin(
            world_position_above_part, world_orientation, part_type, ignore_height)
    elif solver_type == SolverType.AGV1:
        angles = depositOnTray1(
            world_position_above_part, world_orientation, part_type, adjust=adjust)
    elif solver_type == SolverType.AGV2:
        angles = depositOnTray2(
            world_position_above_part, world_orientation, part_type, adjust=adjust)
    elif solver_type == SolverType.BELT:
        angles = solverBelt(
            world_position_above_part, world_orientation, part_type)
    
    set_arm_joint_values(angles, time_to_execute_action)
    rospy.sleep(0.1)
    return angles
    

def go_down_until_get_piece(world_position, world_orientation, part_type, 
    time=3, distance=0.01, solver_type=SolverType.BIN, ignore_height=False, adjust=False):
    
    rospy.loginfo("[Arm Actions] go_down_until_get_piece")
    tfPos = world_position
    tfOri = world_orientation
    position = [tfPos[0], tfPos[1], tfPos[2] - distance]
    position2 = [tfPos[0], tfPos[1], tfPos[2] + 0.02]

    rospy.loginfo(
        "[go_down_until_get_piece] goDownUntilGetPiece pos = " + str(position))

    if solver_type == SolverType.BIN:
        angles = solverBin(
            position, tfOri, part_type, ignore_height)
        angles2 = solverBin(
            position2, tfOri, part_type, ignore_height)

    elif solver_type == SolverType.AGV1:  
        rospy.loginfo(
        "[go_down_until_get_piece] SolverType.AGV1" )
        angles = depositOnTray1(
            position, tfOri, part_type, ignore_height=ignore_height, adjust=adjust)
        angles2= depositOnTray1(
            position2, tfOri, part_type, ignore_height=ignore_height, adjust=adjust) 
    elif solver_type == SolverType.AGV2:
        angles = depositOnTray2(
            position, tfOri, part_type, ignore_height=ignore_height, adjust=adjust)
        angles2 = depositOnTray2(
            position2, tfOri, part_type, ignore_height=ignore_height, adjust=adjust)        



    set_arm_joint_values(angles, time)
    success = gripper_actions.send_gripping_cmd(toGrip=True)
    if not success:
        rospy.logerr(
        "[go_down_until_get_piece] send_gripping_cmd Failure")
        return False

    success = gripper_actions.wait_for_gripper(toGrip=True, max_wait=time+1, inc_sleep=0.005)
    if not success:
        rospy.logerr(
        "[go_down_until_get_piece] wait_for_gripper Failure")
        return False

    set_arm_joint_values(angles2, 0.1)
    rospy.sleep(0.1)
    return True

    


def set_arm_joint_values(list_of_joint_values, time_to_execute_action):
    """
        Publish the joint values as a JointTrajectory Object
    """
    msg = utils.createJointTrajectory(
        list_of_joint_values, time_to_execute_action)
    rospy.loginfo("[set_arm_joint_values]: Setting joint values " +
                  str(list_of_joint_values) + " in the following order " + str(ARM_JOINT_NAMES))
    joint_trajectory_publisher.publish(msg)


def check_arm_joint_values_published(list_of_joint_values=None, static_position_key=None,
                                     accError=[0.009, 0.009, 0.009, 0.009,
                                               0.015, 0.015, 0.009, 0.009, 0.009],
                                     max_sleep=8, force_check_piece=False, force_grp_sts=True):
    """
    Check if the actual state values of the joints is equal to the list joint states of the expected position.

    list_of_joint_values - list of expected joint values
    static_position_key - key to get a static position from the STATIC_POSITION map
    accError - Acceptable error for each joint
    """

    final_joint_values = list_of_joint_values if list_of_joint_values is not None else STATIC_POSITIONS[static_position_key]

    rospy.loginfo("[check_arm_joint_values_published]: final_joint_values:" + str(final_joint_values))
    inc_sleep = 0.05
    slept = 0
    result = False
    grpOK = True

    while not result and slept < max_sleep and grpOK:
        position = global_vars.current_joint_state.position
        # rospy.loginfo("[check_arm_joint_values_published]: position: " + str(position))
        result, listRest = utils.comparePosition(
            position, final_joint_values, accError)
        if(not result):
            rospy.sleep(inc_sleep)
            slept += inc_sleep
            if(force_check_piece):
                grpOK = (global_vars.gripper_state.attached == force_grp_sts)    

    if(not result):
        rospy.logerr("[check_arm_joint_values_published] - Goal not reached")
        rospy.logerr(
            "[check_arm_joint_values_published] - List Joint: " + str(listRest))
        rospy.logerr("[check_arm_joint_values_published] - Expected position " +
                     str(final_joint_values) + " | Actual position " + str(position))
        return False
    else:
        if (not grpOK):
            rospy.logerr("[check_arm_joint_values_published] - Gripper Failed")
            return False
        else:    
            rospy.loginfo("[check_arm_joint_values_published] - Goal Reached")
            return True


def turnWrist(turn_wrist):
        position = global_vars.current_joint_state.position
        angles = []
        angles.extend(position)
        angles[5] = turn_wrist

        set_arm_joint_values(angles, 0.1)

        check_arm_joint_values_published(
            list_of_joint_values=angles)


def MoveSideWays(move_side):
        
        position = global_vars.current_joint_state.position
        angles = []
        angles.extend(position)
        angles[1] = angles[1] + move_side 
        
        set_arm_joint_values(angles, 1.2)

        check_arm_joint_values_published(
            list_of_joint_values=angles)
        

def moveToolTip(incrementZ=0.3, incrementX=0.1, timeToGoal=0.2):
        
        posUpperArm, angleUpperArm = utils.getUpperArmPose()
        posVacum, angleVacum = utils.getVacuumGripperPos()
        posFore, angleFore = utils.getForeArmPos()

        workingPos = deepcopy(posVacum)
        workingPos[2] += incrementZ
        workingPos[0] += incrementX


        shoulderBTriangle = utils.computeXZDistance(posUpperArm, workingPos)

        xDistance = abs(utils.computeXDistance(posUpperArm, workingPos))
        zDistance = abs(utils.computeZDistance(posUpperArm, workingPos))


        wristBTriangle = utils.computeXZDistance(workingPos, posVacum)

        workingIsAbove = utils.computeZDistance(posUpperArm, workingPos) < 0

        a1, b1, c1, A1, B1, C1 = solve(
            a=shoulderBTriangle, b=FORE_ARM, c=UP_ARM)

        a2, b2, c2, A2, B2, C2 = solve(
            a=xDistance, b=zDistance, c=shoulderBTriangle)

        elbow_joint = math.pi - A1

        shoulder_lift_joint = math.pi / 2 - A2 -B1
        wrist_1_joint =  math.pi - (C1 + B2) + math.pi/2
     
        angles = []
        angles.extend(global_vars.current_joint_state.position)
        angles[0] = elbow_joint
        angles[2] = shoulder_lift_joint
        angles[4] = wrist_1_joint

        # rospy.sleep(1)

        set_arm_joint_values(angles, timeToGoal)

        check_arm_joint_values_published(
            list_of_joint_values=angles)


            
def moveToolTipZY(incrementZ=0.3, incrementY=0.1, timeToGoal=0.2): 
        posUpperArm, angleUpperArm = utils.getUpperArmPose()
        posVacum, angleVacum = utils.getVacuumGripperPos()
        posFore, angleFore = utils.getForeArmPos()

        workingPos = deepcopy(posVacum)
        workingPos[2] += incrementZ
        workingPos[1] += incrementY


        shoulderBTriangle = utils.computeYZDistance(posUpperArm, workingPos)

        yDistance = abs(utils.computeYDistance(posUpperArm, workingPos))
        zDistance = abs(utils.computeZDistance(posUpperArm, workingPos))


        wristBTriangle = utils.computeYZDistance(workingPos, posVacum)

        workingIsAbove = utils.computeZDistance(posUpperArm, workingPos) < 0

        a1, b1, c1, A1, B1, C1 = solve(
            a=shoulderBTriangle, b=FORE_ARM, c=UP_ARM)

        a2, b2, c2, A2, B2, C2 = solve(
            a=yDistance, b=zDistance, c=shoulderBTriangle)

        elbow_joint = math.pi - A1

        shoulder_lift_joint = math.pi / 2 - A2 -B1
        wrist_1_joint =  math.pi - (C1 + B2) + math.pi/2
     
        angles = []
        angles.extend(global_vars.current_joint_state.position)
        angles[0] = elbow_joint
        angles[2] = shoulder_lift_joint
        angles[4] = wrist_1_joint

        # rospy.sleep(1)

        # msg = utils.createJointTrajectory(angles, time=1)
        # joint_trajectory_publisher.publish(msg)
        set_arm_joint_values(angles, timeToGoal)

        check_arm_joint_values_published(
            list_of_joint_values=angles)


