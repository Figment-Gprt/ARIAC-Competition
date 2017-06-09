"""
    All commands that the arm will execute in the simulated world will be summarized here.


"""


import rospy
import utils
import global_vars

from constants import *
from trajectory_msgs.msg import JointTrajectory

from ik_solution import solverBin, solverBelt, depositOnTray2, depositOnTray1


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


def go_to_bin_front(bin_id):
    """
    Move to the front of the wanted bin_id.
    bin_id : key for the wanted bin to be used in the STATIC_POSITIONS MAP.
    """
    set_arm_joint_values(STATIC_POSITIONS[bin_id], 1)


def go_to_position_a_bit_above_part(world_position, world_orientation, part_type, time_to_execute_action, solver_type, a_bit_above_value=0.03, ignore_height=False):
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
            world_position_above_part, world_orientation, part_type, ignoreHeight)
    elif solver_type == SolverType.AGV1:
        angles = depositOnTray1(
            world_position_above_part, world_orientation, part_type)
    elif solver_type == SolverType.AGV2:
        angles = depositOnTray2(
            world_position_above_part, world_orientation, part_type)
    elif solver_type == SolverType.BELT:
        angles = solverBelt(
            world_position_above_part, world_orientation, part_type)
    
    set_arm_joint_values(angles, time_to_execute_action)
    rospy.sleep(0.1)
    return angles
    # rospy.sleep(self.time + 2)




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
                                               0.009, 0.01, 0.009, 0.009, 0.009],
                                     max_sleep=8):
    """
    Check if the actual state values of the joints is equal to the list joint states of the expected position.

    list_of_joint_values - list of expected joint values
    static_position_key - key to get a static position from the STATIC_POSITION map
    accError - Acceptable error for each joint
    """

    final_joint_values = list_of_joint_values if list_of_joint_values not None else STATIC_POSITIONS[static_position_key]

    inc_sleep = 0.1
    slept = 0
    while not result and slept < max_sleep:
        position = global_vars.current_joint_state
        result, listRest = utils.comparePosition(
            position, final_joint_values, accError)
        if(not result):
            rospy.sleep(inc_sleep)
            slept += inc_sleep

    if(not result):
        rospy.logerr("[check_arm_joint_values_published] - Goal not reached")
        rospy.logerr(
            "[check_arm_joint_values_published] - List Joint: " + str(listRest))
        rospy.logerr("[check_arm_joint_values_published] - Expected position " +
                     str(position) + " | Actual position " + str(position))
    else:
        rospy.loginfo("[check_arm_joint_values_published] - Goal Reached")
