#!/usr/bin/env python
import rospy
import order_utils
import execution
import global_vars
import math

from utils import PickPlaces

class PickPiece:

    def __init__(self, origin, world_position, world_orientation):
        self.origin             = origin #binX, belt or agvN
        self.world_position     = world_position
        self.world_orientation  = world_orientation

class KitPlan:

    def __init__(self, kit, working_agv, list_part_plan):
        self.kit = kit
        self.working_agv = working_agv
        self.dest_tray_id = working_agv.agv_id
        self.list_part_plan = list_part_plan
        self.planning_score = -1

    def compute_score(self):
        score = 0
        for partPlan in list_part_plan:
            score += partPlan.compute_score()
        return score  

    def __str__(self):

        return 'KitPlan kit:{}; working_agv:{}; planning_score:{}'.format(
            self.kit, self.working_agv, self.planning_score)

    def __repr__(self):
        return 'KitPlan kit:{}; working_agv:{}; planning_score:{}'.format(
            self.kit, self.working_agv, self.planning_score)

class PartPlan:

    def __init__(self, part, pick_piece, kit_plan):
        self.part = part
        self.kit_plan = kit_plan
        self.dest_tray_id = kit_plan.dest_tray_id
        self.pick_piece = pick_piece
        self.planning_score = -1
        # in quaternion representation 1 = PI
        if (self.part.desired_pose.orientation.x == 1.0 or self.part.desired_pose.orientation.y == 1.0) and self.part.part_type == "pulley_part":
            self.to_flip = True
        else:
            self.to_flip = False

    def compute_score(self):
        pass

    def __str__(self):

        return 'PartPlan part:{}; kit_plan:{}; pick_piece:{}; planning_score:{}'.format(
            self.part, self.kit_plan, self.pick_piece, 
            self.planning_score)

    def __repr__(self):
        return 'PartPlan part:{}; kit_plan:{}; pick_piece:{}; planning_score:{}'.format(
            self.part, self.kit_plan, self.pick_piece, 
            self.planning_score)




class WorkingAgv:

    def __init__(self, agv_id):
        self.agv_id = agv_id
        self.reserved = False
        self.reserved_for_kit = None

    def __str__(self):
        return 'WorkingAgv agv_id:{}; reserved:{}; reserved_for_kit_id:{}'.format(
            self.agv_id, self.reserved, self.reserved_for_kit)

    def __repr__(self):
        return 'WorkingAgv agv_id:{}; reserved:{}; reserved_for_kit_id:{}'.format(
            self.agv_id, self.reserved, self.reserved_for_kit.kit_id)

    def reserve(self, kit):
        self.reserved = True
        self.reserved_for_kit = kit
    
    def release(self):
        self.reserved = False
        self.reserved_for_kit = None

    def is_available(self):
        available = self.is_ready_to_deliver() and not self.reserved
        rospy.logdebug("[WorkingAgv] is_available: " + str(available))
        return available

    def is_ready_to_deliver(self):        
        if(self.agv_id == 1):
            status = global_vars.agv1_status
        elif(self.agv_id == 2): 
            status = global_vars.agv2_status
        else:
            rospy.logerr("[WorkingAgv] We really should not be here")

        rospy.logdebug("[WorkingAgv] is_ready_to_deliver: " + str(status))
        #possible status: [ready_to_deliver, preparing_to_deliver, delivering, returning]
        if(status != "ready_to_deliver"):
            return False

        return True



class Scheduler:



    def __init__(self, competition):
        self.order_list = []
        self.finished = False
        self.competition = competition
        self.configure_agvs()

    #[TODO] Are we sure to have always two?
    def configure_agvs(self):

        agv1 = WorkingAgv(1)    
        agv2 = WorkingAgv(2)
        self.agvs = [agv1, agv2]


    def append_order(self, order):
        self.order_list.append(order)

    def isFinished(self):
        return self.finished

    def setFinished(self, status):
        self.finished = status

    def execute(self):
        while not self.isFinished():
            part_plan = self.get_next_part_plan()
            if part_plan is None:
                rospy.loginfo("[Scheduler] No possible Part Plan yet...")
                rospy.loginfo("[Scheduler] Sleeping...")
                rospy.sleep(1)
            else:
                execute_part = execution.ExecutePart(part_plan)
                status = execute_part.execute()
                # part deposited successfully at tray
                if status:
                    part_plan.part.set_done()
                    rospy.loginfo("[Scheduler] Part from kit deposited successfully")
                    
                    if part_plan.part.parent_kit.get_status() == order_utils.Status.DONE:
                        rospy.loginfo("[Scheduler] Kit completed successfully")
                        rospy.loginfo("[Scheduler] Sending AGV")
                        success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                        if(success_agv_cmd):
                            part_plan.kit_plan.working_agv.release()
                        else:
                            rospy.loginfo("[Scheduler] Not Comp Implemented Yet - Could not send Agv Cmd")  
                            attempt_max = 3
                            attempt = 0
                            while  attempt <  attempt_max and not success_agv_cmd:
                                success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                            if(success_agv_cmd):
                                part_plan.kit_plan.working_agv.release()







    #Get non finished high priority order
    def get_non_fin_hp_order(self):
        len_order_list = len(self.order_list)       
        idx_high_priority = len_order_list-1 #Last means high priority

        while idx_high_priority >= 0:
            working_order = self.order_list[idx_high_priority] 
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(working_order.get_status() is not order_utils.Status.DONE): 
                return working_order
            idx_high_priority-=1

    #If there is no available agv, return None
    def get_available_agv(self):
        for agv in self.agvs:
            if agv.is_available():
                return agv

    def get_available_agv_and_reserve(self, kit):
        for agv in self.agvs:
            if agv.is_available():
                rospy.loginfo("[Scheduler] get_available_agv_and_reserve: Available - " + str(agv))    
                agv.reserve(kit)
                return agv

    def get_plan_for_kit(self, kit):

        rospy.logerr("[WorkingAgv] get_plan_for_kit not implemented yet")
        working_agv = self.get_available_agv_and_reserve(kit)
        while(working_agv is None): #TODO improve later. We do not actually need the agv for planning
            rospy.logerr("[Scheduler] No available AGV")
            rospy.sleep(1)
            working_agv = self.get_available_agv_and_reserve(kit)


        kit_plan = KitPlan(kit=kit, working_agv=working_agv, list_part_plan=None)
        return kit_plan


    def get_plan_for_part(self, working_part):

        part_plan = working_part.plan
        if(part_plan is not None):
            rospy.logerr("[Scheduler] get_plan_for_part working_part already has part_plan")    
            return part_plan
        else:
            rospy.loginfo("[Scheduler] get_plan_for_part computing plan for working_part: " + str(working_part))    

        parent_kit = working_part.parent_kit

        kit_plan =  parent_kit.plan
        if(kit_plan is None):
            rospy.loginfo("[Scheduler] get_plan_for_part computing kit_plan: " + str(working_part))    
            kit_plan = self.get_plan_for_kit(parent_kit)
            if(kit_plan is None):
                rospy.logerr("[Scheduler] get_plan_for_part kit_plan still None")
                return
            else:
                parent_kit.plan = kit_plan    


        
        pick_piece = PickPiece(PickPlaces.ANY_BIN, None, None)
        part_plan = PartPlan(part = working_part, 
                        pick_piece=pick_piece, kit_plan = kit_plan)

        rospy.loginfo("[Scheduler]: " + str(part_plan))    

        return part_plan



    def get_next_part_plan(self):
        len_order_list = len(self.order_list)

        if(len_order_list == 0):
            rospy.loginfo("[Scheduler] No Orders yet...")   
            return

        working_order = self.get_non_fin_hp_order()
        if(working_order is None):
            rospy.loginfo("[Scheduler] There are no unfinished orders") 
            return

        working_kit = self.get_frs_non_fin_kit_from_ord(working_order)
        if(working_kit is None):
            rospy.logerr("[Scheduler] There are no unfinished kit but the order is not finished")
            rospy.logerr(working_order.get_full_repr()) 
            return

        working_part = self.get_frs_non_fin_part_from_kit(working_kit)
        if(working_part is None):
            rospy.logerr("[Scheduler] There are no unfinished part but the kit is not finished")
            rospy.logerr(working_order.get_full_repr()) 
            return

        part_plan = self.get_plan_for_part(working_part)
        working_part.plan = part_plan
        return part_plan

    #Get first non finished part from kit
    def get_frs_non_fin_part_from_kit(self, kit):
        for part in kit.parts:
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(part.get_status() is not order_utils.Status.DONE):
                return part
        


    #Get first non finished kit from order
    def get_frs_non_fin_kit_from_ord(self, order):
        for kit in order.kits:
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(kit.get_status() is not order_utils.Status.DONE):
                return kit




