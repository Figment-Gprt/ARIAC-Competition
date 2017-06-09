#!/usr/bin/env python
import rospy
import order_utils
import execution

from utils import PickPlaces

class PickPiece:

    def __init__(self, origin, world_position, world_orientation):
        self.origin             = origin #binX, belt or agvN
        self.world_position     = world_position
        self.world_orientation  = world_orientation

class KitPlan:

    def __init__(self, kit, dest_tray_id, list_part_plan):
        self.kit = kit
        self.dest_tray_id = dest_tray_id
        self.list_part_plan = list_part_plan
        self.planning_score = -1

    def compute_score(self):
        score = 0
        for partPlan in list_part_plan:
            score += partPlan.compute_score()
        return score    

class PartPlan:

    def __init__(self, part, dest_tray_id, pick_piece):
        self.part = part
        self.dest_tray_id = dest_tray_id
        self.pick_piece = pick_piece
        self.planning_score = -1

    def compute_score(self):
        pass




class WorkingAgv:

    def __init__(self, agv_id):
        self.agv_id = agv_id
        self.reserved = False
        self.reserved_for_kit = None

    def reserve(kit):
        self.reserved = True
        self.reserved_for_kit = kit
    
    def release():
        self.reserved = False
        self.reserved_for_kit = None

    def is_absent():
        rospy.logerr("[WorkingAgv] get_status not implemented yet")
        return False


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

    def setFinished(status):
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

    def get_available_agv(self):
        rospy.logerr("[WorkingAgv] get_available_agv not implemented yet")  
        return self.agvs[0]

    def get_plan_for_kit(self, kit):

        rospy.logerr("[WorkingAgv] get_plan_for_kit not implemented yet")
        working_agv = self.get_available_agv()
        kit_plan = KitPlan(kit=kit, dest_tray_id=working_agv.agv_id, list_part_plan=None)
        return kit_plan


    def get_plan_for_part(self, working_part):

        part_plan = working_part.plan
        if(part_plan is not None):
            return part_plan

        parent_kit = working_part.parent_kit

        kit_plan =  parent_kit.plan
        if(kit_plan is None):
            kit_plan = self.get_plan_for_kit(parent_kit)
            if(kit_plan is None):
                rospy.logerr("[Scheduler] get_plan_for_part kit_plan still None")
                return


        
        pick_piece = PickPiece(PickPlaces.ANY_BIN, None, None)
        part_plan = PartPlan(part = working_part, 
                        dest_tray_id = kit_plan.dest_tray_id, pick_piece=pick_piece)

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






