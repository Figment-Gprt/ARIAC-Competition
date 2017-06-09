#!/usr/bin/env python
import actions, arm_actions

class ExecutePart:

    def __init__(self, partPlan):
        self.partPlan = partPlan

    def execute(self):
        # check where the part is, bin or belt
        part_origin = self.partPlan.pick_piece.origin

        # if part is on the bin:
        if "bin" in origin:

            # 1 - move to static position near the bin
            arm_actions.go_to_bin_front(origin)

            # checking joint states
            arm_actions.check_arm_joint_values_published(static_position_key=origin)


            # 2 - move to position a bit above the part
            part_word_position = self.partPlan.pick_piece.word_position
            part_word_orientation = self.partPlan.pick_piece.word_orientation
            part_type = self.partPlan.part.part_type


            list_joint_values = arm_actions.go_to_position_a_bit_above_part(world_position=part_word_position,
             world_orientation=part_word_orientation,
             part_type=part_type, 
             time_to_execute_action=3, 
             arm_actions.SolverType.BIN)

            arm_actions.check_arm_joint_values_published(list_of_joint_values=list_joint_values)

            # 3 - go down until get the part



        # if part is on the belt
        elif "belt" in origin:
            pass

        # if part is on the tray
        elif "tray" in origin:
            pass




        pass
