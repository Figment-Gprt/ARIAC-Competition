from math import pi, atan2, sqrt
from trianglesolver import solve, degree
from constants import *
import rospy


def solverBin(pos, rot, object_type, ignore_height=False):

	rospy.loginfo(
		'[solverBin] pos: {}; rot: {}; object_type {}; ignore_height: {}'.format(
		pos, rot, object_type, ignore_height))
	X_PIECE = pos[0]
	Y_PIECE = pos[1]
	Z_PIECE = pos[2]

	object_height = OBJECT_HEIGHT[object_type.upper()]
	if(not ignore_height):
		Z_PIECE+=object_height

	T2 = (X_BASE - X_PIECE) - H_WRIST

	line_arm_actuator = Y_PIECE + WRIST_LENGTH

	H1 = ((Z_BASE - Z_PIECE) - (GRIPPER + WRIST_1_2)) + H_BASE

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	angles = [angle_elbow, line_arm_actuator,
			 angle_shoulder, 3.14, angle_wrist, -1.57, 1.57 - rot[2]]

	return angles


def computeToolTipGoal(pos, currJoints):
	X_PIECE = pos[0]
	Y_PIECE = pos[1]
	Z_PIECE = pos[2]
	

	T2 = (X_BASE - X_PIECE) - H_WRIST

	line_arm_actuator = Y_PIECE + WRIST_LENGTH

	H1 = ((Z_BASE - Z_PIECE) - (GRIPPER + WRIST_1_2)) + H_BASE

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	angles = [angle_elbow, currJoints[1],
			 angle_shoulder, currJoints[3], angle_wrist, currJoints[5], currJoints[6]]

	return angles


#DO NOT USE THIS || Have a look at class MoveUpToolTip
def moveUp(pos, object_type):
	X_PIECE = pos[0]
	Y_PIECE = pos[1]
	Z_PIECE = pos[2]

	#object_height = OBJECT_HEIGHT[object_type.upper()]
	#Z_PIECE+=object_height

	T2 = (X_BASE - X_PIECE) - H_WRIST

	line_arm_actuator = Y_PIECE + WRIST_LENGTH

	H1 = ((Z_BASE - Z_PIECE) - (GRIPPER + WRIST_1_2)) + H_BASE

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	angles = [angle_elbow + 0.21, line_arm_actuator,
			 angle_shoulder-0.11, 3.14, angle_wrist, -1.57, 0.0]

	return angles	


def solverBelt(pos, rot, object_type):
	X_PIECE = pos[0]
	Y_PIECE = pos[1]
	Z_PIECE = pos[2]

	object_height = OBJECT_HEIGHT[object_type.upper()]
	Z_PIECE+=object_height

	T2 = (X_PIECE - X_BASE) - H_WRIST

	H1 = 0.128 - ((GRIPPER + WRIST_1_2) - (Z_BASE - Z_PIECE))

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	angles = [angle_elbow, Y_PIECE - WRIST_LENGTH,
			 angle_shoulder, 0.0, angle_wrist, -1.57, 1.57 + rot[2]]
	# angles = [2.51, Y_PIECE - WRIST_LENGTH, -1.76, 0, 3.69, -1.56, 0.0]

	return angles

def depositOnTray1(pos, rot, object_type, ignore_height=False, adjust=False):
	X_PIECE = pos[0]
	Y_PIECE = pos[1]
	Z_PIECE = pos[2]

	
	if object_type == "pulley_part":
		object_height = OBJECT_HEIGHT[object_type.upper()]
		Z_PIECE+=((object_height*2))
	else:
		object_height = OBJECT_HEIGHT[object_type.upper()]
		Z_PIECE+=object_height
		

	angle_var = (atan2(X_PIECE - X_BASE,Y_PIECE - Y_BASE) + atan2(WRIST_LENGTH,Y_PIECE - Y_BASE))
	angle_shoulder_pan = 1.57 - angle_var
	
	T1 = sqrt((Y_PIECE - Y_BASE)**2 + (X_PIECE - X_BASE)**2)
	T2 = T1 - H_WRIST

	h_base_piece = Z_BASE - Z_PIECE
	h_base_imaginary = (GRIPPER + WRIST_1_2) - h_base_piece
	H1 = H_BASE - h_base_imaginary

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder_lift = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	if adjust:
		angles = [angle_elbow, 2.10, angle_shoulder_lift,
				  angle_shoulder_pan, angle_wrist, -1.57, angle_var - 1.57 - rot[2]]
	else:
		angles = [angle_elbow, 2.10, angle_shoulder_lift,
				  angle_shoulder_pan, angle_wrist, -1.57, 3.1415 - rot[2] - angle_var]

	return angles

def depositOnTray2(pos, rot, object_type, ignore_height=False, adjust=False):
	X_PIECE = pos[0]
	Y_PIECE = abs(pos[1])
	Z_PIECE = pos[2]

	if object_type == "pulley_part":
		object_height = OBJECT_HEIGHT[object_type.upper()]
		Z_PIECE+=((object_height*2))
	else:
		object_height = OBJECT_HEIGHT[object_type.upper()]
		Z_PIECE+=object_height

	angle_var = (atan2(X_PIECE - X_BASE,Y_PIECE - Y_BASE) - atan2(WRIST_LENGTH,Y_PIECE - Y_BASE))
	angle_shoulder_pan = 4.71 + angle_var

	T1 = sqrt((Y_PIECE - Y_BASE)**2 + (X_PIECE - X_BASE)**2)
	T2 = T1 - H_WRIST
	
	h_base_piece = Z_BASE - Z_PIECE
	h_base_imaginary = (GRIPPER + WRIST_1_2) - h_base_piece
	H1 = H_BASE - h_base_imaginary

	_,_,T3,A1,B1,_ = solve(a=T2, b=H1, C=90*degree)

	_,_,_,A2,B2,C2 = solve(a=T3, b=UP_ARM, c=FORE_ARM)

	##--------------- Angle Conversion -------------##

	angle_elbow = pi - A2
	angle_shoulder_lift = (pi/2) - (A1 + C2)
	angle_wrist = (pi/2) + (pi - (B1+B2))

	if adjust:
		angles = [angle_elbow, 2.10, angle_shoulder_lift,
				  angle_shoulder_pan, angle_wrist, -1.57, angle_var + 1.57 - rot[2]]
	else:
		angles = [angle_elbow, -2.10, angle_shoulder_lift, 
				  angle_shoulder_pan, angle_wrist, -1.57, 3.14-rot[2]+angle_var]

	return angles


def computeAnglesFromSizes(a, b, c):
	a1,b1,c1,A1,B1,C1 = solve(a=a, b=b, c=c)
	rospy.loginfo("#A1,B1,C1 = " + str(A1/degree), + str(B1/degree), + str(C1/degree))
