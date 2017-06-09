#!/usr/bin/env python
from __future__ import print_function

import time
import tf2_ros
import tf as transf
import rospy
import yaml
import math
import actions
import utils
import global_vars
from tf_manager import *



from constants import *
from osrf_gear.msg import Order, VacuumGripperState, LogicalCameraImage, ConveyorBeltState
from osrf_gear.srv import VacuumGripperControl, GetMaterialLocations, ConveyorBeltControl, AGVControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, TransformStamped

import order_utils

import scheduler

class Competition:

    def __init__(self):
        # REMOVE AFTER MERGE
        # self.joint_trajectory_publisher = \
        #     rospy.Publisher("/ariac/arm/command",
        #                     JointTrajectory, queue_size=5)

        self.current_comp_state = None
        self.received_orders = []
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.has_been_zeroed = False

        # store the orders from the ariac competition
        self.orders = []
        # store the actions which the robot will execute
        self.actions_tray1 = []
        self.actions_tray2 = []
        self.faulty_sensor1 = []
        self.faulty_sensor2 = []
        # True when activated
        self.beltState = True
        self.scheduler = scheduler.Scheduler(self)

    # def go_to_initial_position(self):
    #     global STATIC_POSITIONS
    #     msg = utils.createJointTrajectory(STATIC_POSITIONS["initial_position"], 0.5)
    #     rospy.loginfo("[initial_position] Send robot to the initial position")
    #     self.joint_trajectory_publisher.publish(msg)

    def start_plan_and_execute(self):
        rospy.loginfo("[Competition] Starting plan and execute...")
        self.scheduler.execute()

    def __process_order(self, order):
        '''

            An order is an instruction containing kits (maybe more than one) for the robot system to complete

        '''
        for kit in order.kits:
            rospy.loginfo("kit_type " + str(kit.kit_type))

            self.orders.append(
                KitObject(kit.objects, kit.kit_type, order.order_id))

        self.process_orders()

    def __process_initial_actions_into_stack(self, kit_object, actions_stack, kit_type):
        # 1
        object_type = kit_object.type
        # DEBUG
        # if "piston_rod" in object_type:
        rospy.loginfo("Adding material check for object " +
                      str(object_type))
        checkAction = actions.ActionMaterialLocation(
            object_type, kit_object, kit_type)
        self.add_to_action_tray(actions_stack, checkAction)

    def add_to_action_tray(self, action_tray, action, endPos=False):
        if(endPos):
            action_tray.append(action)
        else:
            action_tray.insert(0, action)


    def process_order_into_actions(self, comp_class, action, actionsList, kit_object, tray_id, material_location, kit_type, last, insertAtEnd):

        action_tray = self.actions_tray1 if tray_id == 1 else self.actions_tray2
        # move to initial position
        # initial_position = ActionMove([0,0,0])
        # action_tray.append(initial_position)
        # processing the object position
        # rospy.loginfo("## kit_object = " + str(kit_object))
        # rospy.loginfo("## material_location = " + str(material_location))

        material_destination = None
        if len(material_location) > 1:
            # searching for a bin
            for obj in material_location:
                if "belt" in obj.unit_id:
                    material_destination = obj.unit_id
                    break

            # if a bin is not found get the first location
            if material_destination is None:
                material_destination = material_location[0].unit_id

        else:
            material_destination = material_location[0].unit_id

        initial_position = None

        # Belt must be stopped
        if "belt" in material_destination:

            # belt is not stopped readding action to be executed later
            if self.beltState:
                self.add_to_action_tray(action_tray, action)
                return None
            else:
                rospy.loginfo("## going to belt position")
                # going to bin position
                # TODO UNCOMENT
                initial_position = actions.ActionPose(
                    STATIC_POSITIONS[material_destination], self.joint_trajectory_publisher, 1.0)
                self.add_to_action_tray(
                    action_tray, initial_position)

                actSleep = actions.ActionGoal(
                    "Waiting arm move to initial_position_belt. mp: " + material_destination, self, STATIC_POSITIONS[material_destination], tray_id=tray_id)
                self.add_to_action_tray(action_tray, actSleep)

                # calculating object position according to camera
                object_pos = actions.ActionCameraTF(kit_object, material_destination)
                self.add_to_action_tray(action_tray, object_pos)

                position_over_piece = actions.ActionMovePiece(
                    object_pos, self.joint_trajectory_publisher, 1, kit_object.type, actions.ActionMove.BELT, gprt=self)
                self.add_to_action_tray(
                    action_tray, position_over_piece)

                # actSleep = actions.ActionGoal(
                #     "Waiting arm move to move over piece", self, STATIC_POSITIONS[material_destination], max_sleep=50)
                # self.add_to_action_tray(action_tray, actSleep)

                # grapping_action = actions.ActionGripping(True)
                # self.add_to_action_tray(action_tray, grapping_action)

                # actSleep = actions.ActionGoal("Sleeping while picking piece", self, action=position_over_piece, max_sleep=5)
                # self.add_to_action_tray(action_tray, actSleep)

                moveUpAct = actions.MoveUpToolTip(
                    self, self.joint_trajectory_publisher)
                self.add_to_action_tray(action_tray, moveUpAct)

                self.add_to_action_tray(
                    action_tray, initial_position)



        # bin actions
        else:

            rospy.loginfo("## going to bin position")
            # going to bin position
            # TODO UNCOMENT
            initial_position = actions.ActionPose(
                STATIC_POSITIONS[material_destination], self.joint_trajectory_publisher, 1.0)
            self.add_to_action_tray(
                action_tray, initial_position)

            actGoal = actions.ActionGoal(
                "Waiting arm move to initial_position_bin. mp: " + material_destination, self, 
                 position=STATIC_POSITIONS[material_destination],
                  max_sleep=8, tray_id=tray_id, kit_type=kit_object.type, kit_object = kit_object)
            self.add_to_action_tray(action_tray, actGoal)

            # rospy.loginfo("## calculating object position according to camera")
            # calculating object position according to camera
            actionCameraTF = actions.ActionCameraTF(kit_object, material_destination)
            self.add_to_action_tray(action_tray, actionCameraTF)

            # TO REMOVE
            # ac = ActionTest(self, self.joint_trajectory_publisher)
            # action_tray.append(ac)

            # TODO Change
            #isToTurn = False
            isToTurn = (kit_object.type == "pulley_part") #TODO review. based
            # on rpy

            if(isToTurn):
                turn_action = actions.ActionTurnSameBin(actionCameraTF, kit_object,
                                self.joint_trajectory_publisher, self, 1.0)
                # turn_action = actions.ActionTurn(actionCameraTF,
                #                          kit_object.pose, self.joint_trajectory_publisher, 1.0, tray_id, material_destination)
                self.add_to_action_tray(action_tray, turn_action)

            else:

                # position_over_piece = actions.ActionMovePiece(
                #     actionCameraTF, self.joint_trajectory_publisher, 1.5, kit_object.type, actions.ActionMove.BIN)
                # self.add_to_action_tray(
                #     action_tray, position_over_piece)
                position_over_piece =  actions.ActionMoveABitAbovePiece (
                    actionCameraTF, self.joint_trajectory_publisher, 1.5, kit_object.type, actions.ActionMove.BIN)
                self.add_to_action_tray(action_tray, position_over_piece)

                actionGoDown = actions.ActionGoDownUntilGetPiece(positionAct = position_over_piece, 
                    joint_trajectory_publisher=self.joint_trajectory_publisher,  
                    object_type=kit_object.type, time=3, ignoreHeight=False, distance=0.03)
                self.add_to_action_tray(action_tray, actionGoDown)

                # grapping_action = actions.ActionGripping(True)
                # self.add_to_action_tray(
                # action_tray, grapping_action)
                # actSleep = actions.ActionGoal("Sleeping while picking piece", self, action=position_over_piece, max_sleep=5)
                # self.add_to_action_tray(action_tray, actSleep)

                moveUpAct = actions.MoveUpToolTip(
                    self, self.joint_trajectory_publisher)
                self.add_to_action_tray(action_tray, moveUpAct)

                self.add_to_action_tray(
                    action_tray, initial_position)


        


        # actGoal = actions.ActionGoal(
        #     "Waiting arm move to initial_position", self, action=initial_position, max_sleep=8)
        # self.add_to_action_tray(action_tray, actGoal)

        movepos, rotapos = utils.calculate_order_position(kit_object.pose, tray_id)
        movepos[2] += 0.03  # drop piece a bit higher

        if tray_id == 1:
            near_agv = actions.ActionPose(
                STATIC_POSITIONS["agv1"], self.joint_trajectory_publisher, 1.0)
            actMove = actions.ActionMove.AGV1
        elif tray_id == 2:
            near_agv = actions.ActionPose(
                STATIC_POSITIONS["agv2"], self.joint_trajectory_publisher, 1.0)
            actMove = actions.ActionMove.AGV2


        self.add_to_action_tray(action_tray, near_agv)

        actGoal = actions.ActionGoal(
            "Waiting arm move to Near AGV", self, self.joint_trajectory_publisher, kit_type=kit_type, kit_object=kit_object, action=near_agv, max_sleep=8, tray_id=tray_id)
        self.add_to_action_tray(action_tray, actGoal)

        # actSleep = ActionGoal("Sleeping before placing piece", 8, self)
        # self.add_to_action_tray(action_tray, actSleep, insertAtEnd)

        piece_to_tray = actions.ActionMove(
            movepos, rotapos, self.joint_trajectory_publisher, 1.5, kit_object.type, actMove)
        self.add_to_action_tray(action_tray, piece_to_tray)

        actGoal = actions.ActionGoal(
            "Waiting while reaching piece_to_tray", self, self.joint_trajectory_publisher, kit_type=kit_type, kit_object=kit_object, action=piece_to_tray, tray_id=tray_id)
        self.add_to_action_tray(action_tray, actGoal)


        actSleep = actions.ActionSleep(time=0.2, msg="Sleeping before dropping piece")
        self.add_to_action_tray(action_tray, actSleep)

        grapping_action = actions.ActionGripping(False)
        self.add_to_action_tray(action_tray, grapping_action)

        actSleep = actions.ActionSleep(
            msg= "Sleeping before checking piece faulty", time=0.2)
        self.add_to_action_tray(action_tray, actSleep)

        checkPiece = actions.ActionCheckAndDiscardPiece(
            self, self.joint_trajectory_publisher, tray_id, kit_object, action, actionsList, comp_class, last)
        self.add_to_action_tray(action_tray, checkPiece)

        # rospy.loginfo("## ActionTray: " + str(action_tray))

        # send agv

    def process_orders(self):

        while len(self.orders) > 0:
            order = self.orders.pop()
            rospy.loginfo("Processing order " + str(order.order_id))

            """
                It is necessary to store which trays are being used, 
                because if the two are being used the new order will be halted
            """
            if len(self.actions_tray2) == 0:
                # creating action material check for each object
                for kit_object in order.kit_objects:
                    self.__process_initial_actions_into_stack(
                        kit_object, self.actions_tray2, order.kit_type)

            elif len(self.actions_tray1) == 0:
                # creating action material check for each object
                for kit_object in order.kit_objects:
                    self.__process_initial_actions_into_stack(
                        kit_object, self.actions_tray1, order.kit_type)

            else:
                self.orders.append(order)

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("[Competition] State CallBack - comp state: " + str(msg.data))
        self.current_comp_state = msg.data

    def order_callback(self, ariac_order_msg):
    	order = order_utils.Order(ariac_order_msg)    	
        rospy.loginfo('New order received. {}'.format(order.get_full_repr()))
        self.scheduler.append_order(order)
        #self.received_orders.append(ariac_order_msg)
        #self.__process_order(ariac_order_msg)

    def joint_state_callback(self, msg):
        
        if time.time() - self.last_joint_state_print >= 10:
            # rospy.loginfo("Current Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_joint_state_print = time.time()
        global_vars.current_joint_state = msg

    def gripper_state_callback(self, msg):

        if time.time() - self.last_gripper_state_print >= 10:
            # rospy.loginfo("Current gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_gripper_state_print = time.time()
        self.current_gripper_state = msg
        global_vars.gripper_state = self.current_gripper_state

    def quality_control_sensor_1_callback(self, msg):
        pose = msg.pose  # camera position, DO NOT KNOW IF IT IS NEEDED
        self.faulty_sensor1 = msg.models

    def quality_control_sensor_2_callback(self, msg):
        pose = msg.pose  # camera position, DO NOT KNOW IF IT IS NEEDED
        self.faulty_sensor2 = msg.models


def camera_callback(msg, objs):
    
    camera_id, gprt = objs
    objects_hash = str(hash("".join([obj.type for obj in msg.models])))
    if global_vars.logical_cameras[camera_id]["hash"] != objects_hash:
        global_vars.logical_cameras[camera_id]["hash"] = objects_hash
        if camera_id == BIN_CAMERA["belt"]:
            if len(msg.models) > 0:
            #     control_belt = actions.ActionBeltControl(0)
            #     rospy.sleep(0.5)
            #     gprt.beltState = False
            # else:
            #     control_belt = actions.ActionBeltControl(100)
            #     gprt.beltState = True
                # y_max = 10
                for model in msg.models:
                    if "part" in model.type:
                        gprt.beltState = False
                    else:
                        gprt.beltState = True
                        # if model.pose.position.y < y_max:
                        #     y_max = model.pose.position.y
                        #     global_vars.belt_pieces[model.type] = (model.pose, time.time())
                # print(global_vars.belt_pieces)
            # control_belt.execute_action()
        global_vars.logical_cameras[camera_id]["models"] = msg.models
        global_vars.logical_cameras[camera_id]["camera_position"] = msg.pose


def setup_sensors(gprt):
    
    stream = file(template_files[0], 'r')
    config_file = yaml.load(stream)
    # rospy.loginfo("%s" % config_file)
    sensors = config_file['sensors']
    for k, sensor in sensors.items():
        if sensor['type'] == "break_beam":
            pass
        elif sensor['type'] == "proximity_sensor":
            pass
        elif sensor['type'] == "laser_profiler":
            pass
        elif sensor['type'] == "logical_camera":
            subscribe_path = "/ariac/" + str(k)
            rospy.loginfo("camera " + subscribe_path + " added.")
            global_vars.logical_cameras[k] = {
                "hash": 0,
                "models": [],
                "camera_position": None
            }
            camera_sub = rospy.Subscriber(
                subscribe_path, LogicalCameraImage, callback=camera_callback, callback_args=(k, gprt))


def start_competition():
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    return response.success

def connect_callbacks(comp_class):
    rospy.loginfo("[connect_callbacks] setting comp_state_callback")
    comp_state_sub = rospy.Subscriber(
        "/ariac/competition_state", String, comp_class.comp_state_callback)
    rospy.loginfo("[connect_callbacks] comp_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting order_callback")
    order_sub = rospy.Subscriber(
        "/ariac/orders", Order, comp_class.order_callback)
    rospy.loginfo("[connect_callbacks] order_callback OK")
    rospy.loginfo("[connect_callbacks] setting joint_state_callback")
    joint_state_sub = rospy.Subscriber(
        "/ariac/joint_states", JointState, comp_class.joint_state_callback)
    rospy.loginfo("[connect_callbacks] joint_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting gripper_state_callback")
    gripper_state_sub = rospy.Subscriber(
        "/ariac/gripper/state", VacuumGripperState, comp_class.gripper_state_callback)
    rospy.loginfo("[connect_callbacks] gripper_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting quality_control_sensor_1_callback")
    quality_control_sensor_1 = rospy.Subscriber(
        "/ariac/quality_control_sensor_1", LogicalCameraImage, comp_class.quality_control_sensor_1_callback)
    rospy.loginfo("[connect_callbacks] quality_control_sensor_1_callback OK")
    rospy.loginfo("[connect_callbacks] setting quality_control_sensor_2_callback")
    quality_control_sensor_2 = rospy.Subscriber(
        "/ariac/quality_control_sensor_2", LogicalCameraImage, comp_class.quality_control_sensor_2_callback)
    rospy.loginfo("[connect_callbacks] quality_control_sensor_2_callback OK")
    rospy.loginfo("[connect_callbacks] setting tf_callback")
    tf_sub = rospy.Subscriber(
        "/tf", TFMessage, callback=global_vars.tf_manager.tf_callback)
    rospy.loginfo("[connect_callbacks] tf_callback OK")
    tf_static_sub = rospy.Subscriber(
        "/tf_static", TFMessage, callback=global_vars.tf_manager.tf_static_callback)
    rospy.loginfo("[connect_callbacks] tf_static_callback OK")
    
    

def init_global_vars(comp_class):


    # global_vars.tfBuffer = tf2_ros.Buffer(rospy.Duration(2200.0))


    # global_vars.listener = tf2_ros.TransformListener(global_vars.tfBuffer)
    # global_vars.tf_publisher = rospy.Publisher("/tf_static",  TFMessage, queue_size=20) 

    # global_vars.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    # global_vars.tf_listener = tf2_ros.TransformListener(global_vars.tfBuffer) 
    global_vars.tf_manager = TfManager()