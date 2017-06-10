#!/usr/bin/env python
import arm_actions
import global_vars
import rospy
import transform

from utils import PickPlaces
from constants import *
import gripper_actions

class ExecBin:

    def __init__(self, part_plan, exec_part):
        self.part_plan = part_plan
        self.exec_part = exec_part

    def execute(self):

        exec_step = 0
        failed_comple = False
        done = False

###################       STEP 0       ##########################################        

        while(not failed_comple and not done and not self.exec_part.isInterupted()):

            if(exec_step <= 0): #STEP 0 - Setup env
                part_origin = self.part_plan.pick_piece.origin.value
                part_type = self.part_plan.part.part_type
                part_world_position = self.part_plan.pick_piece.world_position
                part_world_orientation = self.part_plan.pick_piece.world_orientation 

                exec_step =+1 #STEP 0 - DONE

###################       STEP 1       ##########################################

            if(exec_step <= 1 and not self.exec_part.isInterupted()): #STEP 1 - Check Any BIN

                if part_origin == PickPlaces.ANY_BIN.value:
                    rospy.loginfo("\n\n[ExecutePart]: STEP 1 \n")
                    # step 0 - get available pose for a part on any bin
                    camera_id, part_id = global_vars.tf_manager.find_part_name("piston_rod_part_4")
                    if(camera_id is None or part_id is None):
                        rospy.loginfo(
                            "[ExecutePart]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    r = self.exec_part.find_part_any_bin(camera_id, part_id, part_type)
                    if(r is None):
                        rospy.loginfo(
                            "[ExecutePart]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    part_world_position, part_world_orientation = r

                    exec_step =+1 #STEP 1 - DONE

###################       STEP 2       ##########################################  

            if(exec_step <= 2 and not self.exec_part.isInterupted()): #STEP 2 - Check Any BIN                  
                
                rospy.loginfo("\n\n[ExecutePart]: STEP 2 \n")
                # step 1 - move to position in front of the piece
                success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecutePart]: step2 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 2 - DONE

###################       STEP 3       ##########################################                     

            if(exec_step <= 3 and not self.exec_part.isInterupted()): #STEP 3 - move to position a bit above the part

                rospy.loginfo("\n\n[ExecutePart]: STEP 3 \n")
                max_attempt = 3
                attempt = 0
                success = False
                while(attempt < max_attempt and not success):            
                    success = self.exec_part.move_wait_above_part(part_world_position, 
                                                        part_world_orientation, 
                                                        part_type)
                    if not success: 
                        attempt +=1           
                        rospy.loginfo("[ExecutePart]: step3 failed. attempt#" + str(attempt))

                if not success:                 
                    rospy.loginfo("[ExecutePart]: step3 failed completly" + str(attempt))
                    global_vars.tf_manager.add_part_id_to_bl(part_id)
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 3 - DONE

###################       STEP 4       ########################################## 

            if(exec_step <= 4 and not self.exec_part.isInterupted()): #STEP 4 - move to position a bit above the part           

                rospy.loginfo("\n\n[ExecutePart]: STEP 4 \n")
                # 3 - go down until get the part
                success = arm_actions.go_down_until_get_piece(part_world_position, 
                                                    part_world_orientation, 
                                                    part_type)
                if not success:
                    rospy.loginfo("[ExecutePart]: step4 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 4 - DONE

###################       STEP 5       ##########################################

            if(exec_step <= 5 and not self.exec_part.isInterupted()): #STEP 5 - Move back to initial position with the piece                  

                rospy.loginfo("\n\n[ExecutePart]: STEP 5 \n")
                success = self.exec_part.move_wait_front_part(part_world_position=part_world_position, 
                                force_check_piece=True, force_grp_sts=True)
                if not success:
                    rospy.loginfo("[ExecutePart]: step failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE

###################       STEP 6       ##########################################                

            if(exec_step <= 6 and not self.exec_part.isInterupted()): #STEP 6 - Temporary Debug
                
                rospy.loginfo("\n\n[ExecutePart]: STEP 6 \n")
                gripper_actions.send_gripping_cmd(toGrip=False)
                gripper_actions.wait_for_gripper(toGrip=False, max_wait=5, inc_sleep=0.01)
                rospy.sleep(0.5)

                done = True



class ExecutePart:

    def __init__(self, partPlan):
        self.partPlan = partPlan
        self.interrupt = False

    def interupt_call_back():
        self.interrupt = True
        
    def isInterupted(self):
        return self.interrupt    

    def check_gripper(self, toGrp):
        return global_vars.gripper_state.attached == toGrp

    def move_wait_front_part(self, part_world_position, 
                        force_check_piece=False, force_grp_sts=True):
        rospy.loginfo("[ExecutePart]: move_wait_front_part: "+ str(part_world_position))
        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                return False


        angles = arm_actions.go_to_part_bin_front(part_world_position)

        # checking joint states
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)
        if not success:
            rospy.logerr("[ExecutePart]: move_wait_front_part failed")
            return False
        else:
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                    return False
                else:
                    return True
            else:
                return True

    def move_wait_above_part(self, part_world_position, part_world_orientation, part_type):
        rospy.loginfo("[ExecutePart]: move_wait_above_part: "+ str(part_world_position))
        list_joint_values = arm_actions.go_to_position_a_bit_above_part(
            world_position=part_world_position,
            world_orientation=part_world_orientation,
            part_type=part_type, 
            time_to_execute_action=3, 
            solver_type=arm_actions.SolverType.BIN)

        success = arm_actions.check_arm_joint_values_published(
            list_of_joint_values=list_joint_values)       
        if not success:
            rospy.logerr("[ExecutePart]: move_wait_above_part failed")
        return success

    def find_part_any_bin(self, camera_id, part_id, part_type):
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
            return transform.transform_list_to_world(transforms_list)

    def execute_bin(self, part_origin):

        exec_bin = ExecBin(part_plan=self.partPlan, exec_part=self)
        exec_bin.execute()



    def execute(self):
        # check where the part is, bin or belt
        part_origin = self.partPlan.pick_piece.origin.value

        # if part is on the bin:
        if "bin" in part_origin:
            self.execute_bin(part_origin)

        # if part is on the belt
        elif "belt" in part_origin:
            pass

        # if part is on the tray
        elif "tray" in part_origin:
            pass




        pass
