import sys

reload(sys)
sys.setdefaultencoding('utf-8')

import roslib; roslib.load_manifest('kinova_demo')
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

# for gui screen
import math
import argparse
from kinova_msgs.srv import HomeArm, Stop, Start
import threading
import time as t

currentFingerPosition = [0.0, 0.0, 0.0];

#function for kinova finger open and close
def gripper_client(prefix, finger_positions):

    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions';

    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction);
    client.wait_for_server();

    goal = kinova_msgs.msg.SetFingersPositionGoal();
    goal.fingers.finger1 = float(finger_positions[0]);
    goal.fingers.finger2 = float(finger_positions[1]);

    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:

        goal.fingers.finger3 = 0.0;
    else:

        goal.fingers.finger3 = float(finger_positions[2]);

    client.send_goal(goal);

    if client.wait_for_result(rospy.Duration(5.0)):

        return client.get_result();
    else:

        client.cancel_all_goals();
        rospy.logwarn('        the gripper action timed-out');

        return None

# function for robot arm position control
def cartesian_pose_client(prefix, position, orientation):

	"""Send a cartesian goal to the action server."""
	action_address = '/' + prefix + 'driver/pose_action/tool_pose';
	client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction);
	client.wait_for_server();

	goal = kinova_msgs.msg.ArmPoseGoal();
	goal.pose.header = std_msgs.msg.Header(frame_id = (prefix + 'link_base'));
	goal.pose.pose.position = geometry_msgs.msg.Point(x = position[0], y = position[1], z = position[2]);
	goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
		x = orientation[0], y = orientation[1], z = orientation[2], w = orientation[3]);

	# print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

	client.send_goal(goal);

	if client.wait_for_result(rospy.Duration(10.0)):

		return client.get_result();
	else:

		client.cancel_all_goals();
		print('        the cartesian action timed-out');

		return None

######             common             ######
def QuaternionNorm(Q_raw):

	qx_temp, qy_temp, qz_temp, qw_temp = Q_raw[0:4];
	qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp);
	qx_ = qx_temp/qnorm;
	qy_ = qy_temp/qnorm;
	qz_ = qz_temp/qnorm;
	qw_ = qw_temp/qnorm;
	Q_normed_ = [qx_, qy_, qz_, qw_];
	
	return Q_normed_;

def Quaternion2EulerXYZ(Q_raw):

	Q_normed = QuaternionNorm(Q_raw);
	qx_ = Q_normed[0];
	qy_ = Q_normed[1];
	qz_ = Q_normed[2];
	qw_ = Q_normed[3];

	tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_));
	ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_);
	tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_));
	EulerXYZ_ = [tx_,ty_,tz_];
	
	return EulerXYZ_;

def EulerXYZ2Quaternion(EulerXYZ_):

	tx_, ty_, tz_ = EulerXYZ_[0:3];
	sx = math.sin(0.5 * tx_);
	cx = math.cos(0.5 * tx_);
	sy = math.sin(0.5 * ty_);
	cy = math.cos(0.5 * ty_);
	sz = math.sin(0.5 * tz_);
	cz = math.cos(0.5 * tz_);

	qx_ = sx * cy * cz + cx * sy * sz;
	qy_ = -sx * cy * sz + cx * sy * cz;
	qz_ = sx * sy * cz + cx * cy * sz;
	qw_ = -sx * sy * sz + cx * cy * cz;

	Q_ = [qx_, qy_, qz_, qw_];
	
	return Q_;

def verboseParser(verbose, pose_mq_):

	""" Argument verbose """
	position_ = pose_mq_[:3];
	orientation_q = pose_mq_[3:];

	if verbose:

		orientation_rad = Quaternion2EulerXYZ(orientation_q);
		orientation_deg = list(map(math.degrees, orientation_rad));

		print('Cartesian position is: {}'.format(position_));
		print('Cartesian orientation in Quaternion is: ');
		print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], 
																orientation_q[1], orientation_q[2], orientation_q[3]));

		print('Cartesian orientation in Euler-XYZ(radian) is: ');
		print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]));

		print('Cartesian orientation in Euler-XYZ(degree) is: ');
		print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]));

class RobotArm():

	def __init__(self,kinova_robotType):

		#Robot param set
		self.robot_category = kinova_robotType[0];
		self.robot_category_version = int(kinova_robotType[1]);
		self.wrist_type = kinova_robotType[2];
		self.arm_joint_number = int(kinova_robotType[3]);
		self.robot_mode = kinova_robotType[4];
		self.finger_number = int(kinova_robotType[5]);
		self.prefix = kinova_robotType+ "_";

		finger_maxDist = 18.9/2/1000;  # max distance for one finger in meter
		finger_maxTurn = 6800;  # max thread turn for one finger

		#Rosnode init
		rospy.init_node(self.prefix + 'pose_action_client');

		self.unit = 'mdeg';
		self.currentCartesian = [0.2, -0.27, 0.506, 1.64, 1.108, -0.04];
		self.setpoint = [0.2, -0.27, 0.506, 1.64, 1.108, -0.04];

		self.arm_home = rospy.ServiceProxy('/j2n6s300_driver/in/home_arm', HomeArm);
		self.arm_stop = rospy.ServiceProxy('/j2n6s300_driver/in/stop', Stop);
		self.arm_start = rospy.ServiceProxy('/j2n6s300_driver/in/start', Start);
		self.arm_s = rospy.Subscriber('arm/command', Int32, self.arm_get);
		self.arm_p = rospy.Publisher('arm/finish', Int32, queue_size = 1);

		self.trig_hus = rospy.Publisher('/trigger_husky', Int32, queue_size = 1);
		self.G0_message = Int32();
		self.m = self.setpoint;
		self.condition = 0;

	def arm_get(self, data):

		command = data.data;

		if command == 1:

			self.arm_home();

	def go(self): 

		t3 = threading.Thread(target = self.arm_go_thread);
		t3.start();
		t3.join();

	def arm_go_thread(self):

		pose_mq, pose_mdeg, pose_mrad = self.unitParser(self.m); # get q deg and rag

		try:

			poses = [float(n) for n in pose_mq];

			result = cartesian_pose_client(self.prefix,poses[:3], poses[3:]);

			print('Cartesian pose sent!');

		except rospy.ROSInterruptException:

			print("program interrupted before completion");

		verboseParser(1, poses);

	def go_to_setpoint(self):

		self.arm_home();

	def stop(self):

		self.arm_stop();

	def start(self):

		self.arm_start();

	def reset_point(self):

		for i in range(6):

			self.m[i].set(self.currentCartesian[i]);

	def unitParser(self, pose_value_):

		relative_ = 0;
		position_ = pose_value_[:3];
		orientation_ = pose_value_[3:];

		if relative_:

			orientation_rad_list =  currentCartesianCommand[3:];
			orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)];

		else:

			orientation_rad = orientation_;

		orientation_deg = list(map(math.degrees, orientation_rad));
		orientation_q = EulerXYZ2Quaternion(orientation_rad);

		pose_mq_ = position_ + orientation_q;
		pose_mdeg_ = position_ + orientation_deg;
		pose_mrad_ = position_ + orientation_rad;

		return pose_mq_, pose_mdeg_, pose_mrad_;

	def getcurrentCartesianCommand(self, prefix_):

	# wait to get current position
		topic_address = '/' + prefix_ + 'driver/out/cartesian_command';
		rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand);
		rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose);

		print('position listener obtained message for Cartesian pose. ');

	def setcurrentCartesianCommand(self, feedback):

		currentCartesianCommand_str_list = str(feedback).split("\n");

		for index in range(0,len(currentCartesianCommand_str_list)):

			temp_str = currentCartesianCommand_str_list[index].split(": ");
			self.currentCartesian[index] = float(temp_str[1]);

	def grip_ball(self):

		gripper_client(self.prefix,[6800,6800,6800]);

	def throw_ball(self):

		gripper_client(self.prefix,[0,0,0]);

	def Husky_go(self):

		self.G0_message.data = int(1);
		self.trig_hus.publish(self.G0_message);

	def Husky_stop(self):

		self.G0_message.data = int(0);
		self.trig_hus.publish(self.G0_message);

	def get_position_grip(self, coord):

		self.m[1] = coord.linear.x;
		self.m[2] = coord.linear.y;
		self.m[3] = coord.linear.z;

		self.go();

		t1 = threading.Thread(target = self.grip_ball);
		t1.start();
		t1.join();

		self.Husky_go();

	def get_position_throw(self):

		t2 = threading.Thread(target = self.throw_ball);
		t2.start();
		t2.join();

		self.Husky_go();

	def Go_to_target(self):

		# subscribe to /trigger_kinova and wait for message
		_type = rospy.wait_for_message("/trigger_kinova", Int32, timeout = None);
		self.condition = int(_type.data);

		self.Husky_stop();
	
		# 1 is for gripping ball
		if self.condition == 1:

			# subscribe to camera and wait for message
			coordinate_m = rospy.wait_for_message("/tennis_position", Twist, timeout = None);
			self.get_position(coordinate_m);

		# 2 is for gripping ball
		elif self.condition == 2:

			self.get_position_throw();

		#need to build a subscriber to subscribe the topic which is published by cameraW
		rospy.init_node("get_position", anonymous = True);
		coordinate_m = rospy.wait_for_message("/tennis_position", geometry_msgs.msg.Twist, timeout = None);
		self.get_position(coordinate_m);

if __name__ == '__main__':

	kinova_robotType = 'j2n6s300';
	KINOVA = RobotArm(kinova_robotType);
	KINOVA.go_to_setpoint();
	KINOVA.throw_ball();

	while True:

		try:

			KINOVA.Go_to_target();

		except rospy.ROSInterruptException:

			KINOVA.stop();
			break;

		t.sleep(1);
		