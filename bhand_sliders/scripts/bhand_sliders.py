#! /usr/bin/env python
import rospy
import time
import serial

from wam_msgs.msg import HandCommand
from std_msgs.msg import Empty
from math import pi

class JointPub:
	def __init__(self):
		joint_pub_topic = rospy.get_param("joint_pub_topic", "/bhand/hand_cmd")
		print "Creating joint command publisher."
		self.close_hand_pub = rospy.Publisher("/bhand/close_grasp", Empty, queue_size=1)
		self.open_hand_pub = rospy.Publisher("/bhand/open_grasp", Empty, queue_size=1)
		self.joint_pub = rospy.Publisher(joint_pub_topic, HandCommand, queue_size=1)
		self.hand_command_template = HandCommand()

	def publish_jnts(self, joint_list):
		self.hand_command_template.header.stamp = rospy.Time.now()
		self.hand_command_template.f1 = joint_list[0]
		self.hand_command_template.f2 = joint_list[2]
		self.hand_command_template.f3 = joint_list[1]
		
		self.hand_command_template.spread = joint_list[3]
		rospy.loginfo("Publishing hand command.")
		self.joint_pub.publish(self.hand_command_template)

	def open_hand(self):
		self.open_hand_pub.publish(Empty())
	
	def close_hand(self):
		self.close_hand_pub.publish(Empty())

if __name__ == "__main__":
	rospy.init_node("bhand_sliders")
	rospy.loginfo("Mechanical slider node online.")

	jnt_pub = JointPub()
	
	# Open the serial connection
	ser = serial.Serial('/dev/ttyACM0', 9600)
	ser.readline()

	while True:
		vals = ser.readline().strip()
		print "Raw data '" + vals + "'"
		vals = vals.split(",")
		print "Post split: ", vals
		for idx, x in enumerate(vals):
			vals[idx] = float(x)

		jnt_pub.publish_jnts(vals)
