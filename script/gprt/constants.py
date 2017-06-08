import os

STATIC_POSITIONS = {
    "bin5"	: [2.8797932658, -1.1661, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin6"	: [2.8797932658, -0.3711, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin7"	: [2.8797932658, 0.3939, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin8"	: [2.8797932658, 1.1589, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "agv1": [2.8797932658, 2.1, -1.308996939, 1.57, 3.1415926536, -1.5707963268, 0],
    "agv2": [2.8797932658, -2.1, -1.308996939, 4.71, 3.1415926536, -1.5707963268, 0],
    "disBelAgv2": [1.76, -2.10, -0.88, 5.91, 3.77, -1.57, 0.0],
    "disBelAgv1": [1.76, 2.10, -1.26, 0.13, 4.15, -1.57, 0.0],
    "belt": [1.89, 1.19, -1.13, 0, 4.52, -1.57, 0.0],
    "initial_position": [2.51, 0.0, -1.13, 3.14, 3.14, -1.51, 0.0]
}

BIN_CAMERA = {
    "bin5"	: "logical_camera_2",
    "bin6"	: "logical_camera_2",
    "bin7"	: "logical_camera_1",
    "bin8"	: "logical_camera_1",
    "belt"	: "logical_camera_5"
}

AGV_CAMERA = {
    "agv1": "logical_camera_3_agv1",
    "agv2": "logical_camera_4_agv2",
}

TRAY_FRAME = {
    1 : "agv1_load_point_frame",
    2 : "agv2_load_point_frame"
}

TRAY_POSITIONS = {
    1: [0.3002, 3.15, 0.77],
    2: [0.3, -3.15, 0.77]
}

TRAY_POSITIONS2 = {
    1: [0, -0.15, 0.03],
    2: [0, 0.15, 0.03]
}


ARM_JOINT_NAMES = [
    'elbow_joint',
    'linear_arm_actuator_joint',
    'shoulder_lift_joint',
    'shoulder_pan_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

X_BASE = 0.3
Y_BASE = 2.1
Z_BASE = 0.9999

GRIPPER = 0.01 + 0.005415
WRIST_1_2 = 0.0922

H_BASE = 0.128
H_WRIST = 0.1157

UP_ARM = 0.6127
FORE_ARM = 0.5716

WRIST_LENGTH = 0.1639

OBJECT_HEIGHT = {
    "PISTON_ROD_PART": 0.005,
    "GEAR_PART": 0.006,
    "PULLEY_PART": 0.0719, 
    "PULLEY_PART_TURN": 0.0719,
    "GASKET_PART": 0.0223999,
    "DISK_PART": 0.0248
}

this_dir = os.path.abspath(os.path.dirname(__file__))
template_files = [
    os.path.join(this_dir, '..', '..', 'config', 'figment_gear_conf.yaml')
]
