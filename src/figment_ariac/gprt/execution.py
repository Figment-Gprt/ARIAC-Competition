#!/usr/bin/env python
import arm_actions
import global_vars
import rospy
import transform

from utils import *
from constants import *
from trajectory_msgs.msg import JointTrajectory
import gripper_actions

from osrf_gear.srv import AGVControl

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
            jump = False

            if(exec_step <= 0): #STEP 0 - Setup env
                part_origin = self.part_plan.pick_piece.origin.value
                part_type = self.part_plan.part.part_type
                part_world_position = self.part_plan.pick_piece.world_position
                part_world_orientation = self.part_plan.pick_piece.world_orientation
                tray_id = self.part_plan.dest_tray_id 
                desired_part_pose = self.part_plan.part.desired_pose
                exec_step =+1 #STEP 0 - DONE
                to_flip = self.part_plan.to_flip
                if to_flip:
                    rospy.loginfo("\n\n[ExecutePart]: STEP 0 : Part must be FLIPPED\n\n\n\n")

###################       STEP 1       ##########################################

            if(not jump and exec_step <= 1 and not self.exec_part.isInterupted()): #STEP 1 - Check Any BIN

                if part_origin == PickPlaces.ANY_BIN.value:
                    rospy.loginfo("\n\n[ExecutePart]: STEP 1 \n")
                    # step 0 - get available pose for a part on any bin
                    camera_id, part_id = global_vars.tf_manager.find_part_name(part_type)
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

            if(not jump and exec_step <= 2 and not self.exec_part.isInterupted()): #STEP 2 - Check Any BIN                  
                
                rospy.loginfo("\n\n[ExecutePart]: STEP 2 \n")
                # step 1 - move to position in front of the piece
                success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecutePart]: step2 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 2 - DONE

###################       STEP 3       ##########################################                     

            if(not jump and exec_step <= 3 and not self.exec_part.isInterupted()): #STEP 3 - move to position a bit above the part

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

            if(not jump and exec_step <= 4 and not self.exec_part.isInterupted()): #STEP 4 - move to position a bit above the part           

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

            if(not jump and exec_step <= 5 and not self.exec_part.isInterupted()): #STEP 5 - Going down to get part and flipping if needed

                rospy.loginfo("\n\n[ExecutePart]: STEP 5 \n")

                # part must be flipped
                if to_flip:
                    success = self.exec_part.flip_part_bin(part_type)
                
                               
                success = self.exec_part.move_wait_front_part(part_world_position=part_world_position, 
                                    force_check_piece=True, force_grp_sts=True)
                

                if not success:
                    rospy.loginfo("[ExecutePart]: step failed. Reseting")
                    # check what to do here
                    self.part_plan.part.reset()
                    return False

                

                exec_step =+1 #STEP  - DONE

###################       STEP 6       ##########################################                
            if(not jump and exec_step <= 6 and not self.exec_part.isInterupted()): #STEP 6 - Move To TRAY
                rospy.loginfo("\n\n[ExecutePart]: STEP 6 \n")
                success = self.exec_part.move_to_tray(tray_id)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecutePart]: step6 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

            if(exec_step <= 6 and not self.exec_part.isInterupted()): #STEP 6 - Temporary Debug
                exec_step =+1 #STEP  - DONE

###################       STEP 7       ##########################################  
#STEP 7 - Put Part at tray              
            if(not jump and exec_step <= 7 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecutePart]: STEP 7 \n")
                #DEBUG LACK OF PARTS
                # gripper_actions.send_gripping_cmd(toGrip=False)
                # gripper_actions.wait_for_gripper(toGrip=False, max_wait=5, inc_sleep=0.01)
                # rospy.sleep(0.5)
                success = self.exec_part.deposit_at_tray(desired_part_pose=desired_part_pose, part_type=part_type, tray_id=tray_id, force_check_piece=True)

                if not success:
                    rospy.loginfo("[ExecutePart]: step7 failed. Reseting")
                                        
                    camera_id, part_id = global_vars.tf_manager.find_part_name(part_name=part_type, 
                                                                    dad="logical_camera_agv_1_frame")
                    r = self.exec_part.find_part_any_agvs(camera_id, part_id, part_type)
                    
                    #print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                    #print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                    #print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 
                    
                    part_world_position, part_world_orientation = r

                    #print ("_____________________________________________________________________________")
                    print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 

                    solver = arm_actions.SolverType.AGV1 if tray_id == 1 else arm_actions.SolverType.AGV2
                    
                    arm_actions.moveToolTip(0.3, 0.1, 1.4)
                    
                    success = self.exec_part.move_wait_above_part(part_world_position, part_world_orientation, part_type, solver, 0.1)

                    success = arm_actions.go_down_until_get_piece(world_position=part_world_position, 
                                                                world_orientation=part_world_orientation, 
                                                                part_type=part_type, 
                                                                time=3, ignore_height=False, 
                                                                distance=0.01, solver_type=arm_actions.SolverType.AGV1)
                    
                    arm_actions.moveToolTip(0.3, 0.1, 1.4)

                    rospy.logerr("........................................................................")
                    if success :
                        rospy.logerr("....................SUCCESS................................")
                        exec_step = 7 #We are coming back here
                        jump = True

                    else:
                        rospy.logerr("[ExecutePart]: step7 failed. We do not know what to do yet")
                        exec_step =+1 #STEP  - DONE
                else:                    

                    exec_step =+1 #STEP  - DONE

###################       STEP 8       ##########################################  
#STEP 8 - Check Falty Piece
            if(not jump and exec_step <= 8 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecutePart]: STEP 8 \n")

                if(self.part_plan.dest_tray_id == 1):
                    faulty_sensor_msg = global_vars.faulty_sensor1
                elif(self.part_plan.dest_tray_id == 2): 
                    faulty_sensor_msg = global_vars.faulty_sensor1    
                else:
                    rospy.logerr("[ExecutePart]: step8 failed. We do not know what to do yet")
                    self.part_plan.part.reset()
                    return False

                falty = len(faulty_sensor_msg) > 0
                if falty:
                    rospy.loginfo("[ExecutePart][STEP8] - Falty part detected")

                xec_step =+1 #STEP  - DONE
                



###################       STEP 9       ##########################################  
#STEP 9 - Move To TRAY              
            if(not jump and exec_step <= 9 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecutePart]: STEP 9 \n")
                
                success = self.exec_part.move_to_tray(tray_id, force_check_piece=False, time=0.5)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecutePart]: step9 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE

                done = True


###################       STEP 11       ##########################################                            
            # if(exec_step <= 11 and not self.exec_part.isInterupted()): #STEP 11 - Temporary Debug
                
            #     rospy.loginfo("\n\n[ExecutePart]: STEP 11 \n")
            #     gripper_actions.send_gripping_cmd(toGrip=False)
            #     gripper_actions.wait_for_gripper(toGrip=False, max_wait=5, inc_sleep=0.01)
            #     rospy.sleep(0.5)

            #     done = True

        return done


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

    def move_wait_above_part(self, part_world_position, part_world_orientation, part_type, solver_type=arm_actions.SolverType.BIN, a_bit_above_value=0.015):
        rospy.loginfo("[ExecutePart]: move_wait_above_part: "+ str(part_world_position))
        list_joint_values = arm_actions.go_to_position_a_bit_above_part(
            world_position=part_world_position,
            world_orientation=part_world_orientation,
            part_type=part_type, 
            time_to_execute_action=3, 
            solver_type=solver_type,
            a_bit_above_value=a_bit_above_value)

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
            for k, v in BINS_CAMERA.items():
                for cams in v:
                    if cams in camera_id:
                        part_origin = k
                        break
            # getting position and orientation from the part
            transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world')
            return transform.transform_list_to_world(transforms_list)

    def find_part_any_agvs(self, camera_id, part_id, part_type):
        if len(camera_id) > 0:
            rospy.loginfo("[ExecutePart]: camera_id: "+ str(camera_id))
            rospy.loginfo("[ExecutePart]: part: "+ str(part_id))
            # getting bin id from the part
            for k, v in AGVS_CAMERA.items():
                if v in camera_id:
                    part_origin = k
                    break
            # getting position and orientation from the part
            transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world')
            return transform.transform_list_to_world(transforms_list)

    def move_to_tray(self, tray_id, force_check_piece=True, force_grp_sts=True, time=2):
        tray_name = "agv" + str(tray_id)
        rospy.loginfo("[ExecutePart]: move_to_tray: "+ tray_name)

        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                return False

        angles = arm_actions.go_to_tray_position(tray_name)
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)


        if not success:
            rospy.logerr("[ExecutePart]: move_to_tray failed")
            return False
        else:
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:move_to_tray - Gripper failed!")
                    return False
                else:
                    return True
            else:
                return True

    def deposit_at_tray(self, desired_part_pose, part_type, tray_id, force_check_piece=False, force_grp_sts=True):
        # calculate position of the part at tray
        solver_type = arm_actions.SolverType.AGV1 if tray_id == 1 else arm_actions.SolverType.AGV2
        part_position_at_tray, part_orientation_at_tray  = calculate_order_position(desired_part_pose, tray_id)


        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed!")
                return False

        # move a bit above the position
        max_attempt = 3
        attempt = 0
        success = False
        while(attempt < max_attempt and not success):            
            success = self.move_wait_above_part(part_position_at_tray, 
                                                part_orientation_at_tray, 
                                                part_type,
                                                solver_type=solver_type)
            if not success: 
                attempt +=1           
                rospy.loginfo("[ExecutePart]: step7 failed. attempt#" + str(attempt))

        if not success:
            rospy.logerr("[ExecutePart]: deposit_at_tray failed")
            return False
        else:
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed to complete the movement holding part!")
                    return False
                else:
                    rospy.sleep(0.1) # waiting a little bit to deposit part
                    success = gripper_actions.send_gripping_cmd(toGrip=False)

                    if success:
                        return True
                    else:
                        rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed to drop part!")
                        return False
            else:
                return True


    def flip_part_bin(self, part_type):

        arm_actions.moveToolTip(0.3, 0.1, 1.4)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 1 \n")
        
        arm_actions.turnWrist(0.01)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 2 \n")
        
        arm_actions.MoveSideWays(0.022)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 3 \n")
        
        
        arm_actions.moveToolTip(-0.275, 0.13, 0.5)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 4 \n")
        # rospy.sleep(30)

        rospy.sleep(0.3)

        gripper_actions.send_gripping_cmd(toGrip=False)
        
        arm_actions.MoveSideWays(0.1)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 5 \n")
        
        gripper_actions.wait_for_gripper(toGrip=False, max_wait=5, inc_sleep=0.01)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 6 \n")
        
        arm_actions.turnWrist(-1.5707963268)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 7 \n")

        # Move ToolTip UP
        arm_actions.moveToolTip(0.4, 0, 0.3)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 8 \n")
        
        #Move a bit to the other side
        arm_actions.MoveSideWays(-0.4)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 9 \n")
        rospy.sleep(0.3)
        
        # Move ToolTip Down
        arm_actions.moveToolTip(-0.01, 0.23, 0.2)
        # rospy.loginfo("\n\nSTEP 5 - PASSO 10 \n")
        rospy.sleep(0.7)
        
         # Move a bit to the other side
        arm_actions.MoveSideWays(0.3)
        # rospy.loginfo("\n\nSTEP 5 - Aqui ok? \n")

        rospy.sleep(0.7)

        camera_id, part_id = global_vars.tf_manager.find_part_name(part_type)
        if(camera_id is None or part_id is None):
            rospy.loginfo(
                "[ExecutePart]:Failed. No available part {} found".format(part_type))
            self.part_plan.part.reset()
            return False
        r = self.find_part_any_bin(camera_id, part_id, part_type)
        if(r is None):
            rospy.loginfo(
                "[ExecutePart]:Failed. No available part {} found".format(part_type))
            self.part_plan.part.reset()
            return False
        part_world_position, part_world_orientation = r

        rospy.sleep(1)

        self.move_wait_above_part(part_world_position, 
                                            part_world_orientation, 
                                            part_type)

        
        success = arm_actions.go_down_until_get_piece(part_world_position, 
                                        part_world_orientation, 
                                        part_type, ignore_height=True)

        # Move ToolTip UP
        arm_actions.moveToolTip(0.2, 0, 0.3)

        return success
        



    def execute_bin(self, part_origin):

        exec_bin = ExecBin(part_plan=self.partPlan, exec_part=self)
        success = exec_bin.execute()

        return success






    def execute(self):
        # check where the part is, bin or belt
        part_origin = self.partPlan.pick_piece.origin.value
        success = False
        # if part is on the bin:
        if "bin" in part_origin:
            success = self.execute_bin(part_origin)

        # if part is on the belt
        elif "belt" in part_origin:
            pass

        # if part is on the tray
        elif "tray" in part_origin:
            pass

        return success



def send_agv(kit, tray_id):
    agvServiceName = "/ariac/agv{0}".format(tray_id)
    rospy.loginfo("sending 'kit_type: " + kit.kit_type +
                  " to : " + agvServiceName)
    rospy.wait_for_service(agvServiceName)
    try:
        send_agv = rospy.ServiceProxy(
            agvServiceName, AGVControl)
        success = send_agv(kit.kit_type)
        rospy.sleep(1)
        return success
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to notify agv %s: %s" % (self.kit_type, exc))
        return False