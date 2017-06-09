#!/usr/bin/env python
import rospy
import order_utils
import execution

class PickPiece:

	def __init__(self, origin, position):
		self.origin = origin #binX, belt or agvN
		self.position = position

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


class Scheduler:



	def __init__(self, competition):
		self.order_list = []
		self.finished = False
		self.competition = competition


	def append_order(self, order):
		pass

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



	def get_next_part_plan(self):
		pass



