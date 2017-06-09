#!/usr/bin/env python
import arm_actions
import global_vars
import rospy
import transform

from utils import PickPlaces
from constants import *


class ExecutePart:

    def __init__(self, partPlan):
        self.partPlan = partPlan

    def execute(self):
        # check where the part is, bin or belt
        part_origin = self.partPlan.pick_piece.origin.value
        part_type = self.partPlan.part.part_type
        part_world_position = self.partPlan.pick_piece.world_position
        part_world_orientation = self.partPlan.pick_piece.world_orientation 

        # if part is on the bin:
        if "bin" in part_origin:

            if part_origin == PickPlaces.ANY_BIN.value:
                camera_id, part_id = global_vars.tf_manager.find_part_name(part_type)
                if len(camera_id) > 0:
                    rospy.loginfo("[ExecutePart]: camera_id: "+ str(camera_id))
                    rospy.loginfo("[ExecutePart]: part: "+ str(part_id))
                    # getting bin id from the part
                    for k, v in BIN_CAMERA.items():
                        if v in camera_id:
                            part_origin = k
                            break
                    # getting position and orientation from the part
                    transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world')
                    part_world_position, part_world_orientation = transform.transform_list_to_world(transforms_list)
                    




                

            # 1 - move to static position near the bin
            arm_actions.go_to_bin_front(part_origin)

            # checking joint states
            arm_actions.check_arm_joint_values_published(static_position_key=part_origin)


            # 2 - move to position a bit above the part
            
            


            list_joint_values = arm_actions.go_to_position_a_bit_above_part(world_position=part_world_position,
             world_orientation=part_world_orientation,
             part_type=part_type, 
             time_to_execute_action=3, 
             solver_type=arm_actions.SolverType.BIN)

            arm_actions.check_arm_joint_values_published(list_of_joint_values=list_joint_values)

            # 3 - go down until get the part



        # if part is on the belt
        elif "belt" in part_origin:
            pass

        # if part is on the tray
        elif "tray" in part_origin:
            pass




        pass
