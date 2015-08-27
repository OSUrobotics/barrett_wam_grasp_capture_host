#! /usr/bin/env python
import rospy
import rosbag

from grasp_manager.msg import GraspSnapshot
from sensor_msgs.msg import Image, JointState

from shared_globals import *
from shared_playback import *

if __name__ == "__main__":
	rospy.init_node("extraction_replay")
	rospy.loginfo("Replay node online!")
	
	wam_joint_state_pub = rospy.Publisher("/wam/joint_states", JointState, queue_size=1, latch=True)
	hand_joint_state_pub = rospy.Publisher("/bhand/joint_states", JointState, queue_size=1, latch=True)
	rgb_image_pub = rospy.Publisher("/grasp_rgb", Image, queue_size=1, latch=True)
	data_repub = rospy.Publisher("/grasp_extremes", GraspSnapshot, queue_size=3, latch=True)
	while not rospy.is_shutdown():
		print "obtaining data dir"
		data_dir, obj_num, sub_num = get_data_dir()
		extreme_bag = rosbag.Bag(data_dir + "/" + extreme_bag_name , "r")

		for topic, msg, t in extreme_bag.read_messages(topics=[grasp_extreme_topic]):
			user_input = raw_input("Press enter to show grasp or n to stop: ")
			if user_input.lower() == "n":
				break

			data_repub.publish(msg)
			wam_joint_state_pub.publish(msg.wam_joints)
			hand_joint_state_pub.publish(msg.hand_joints)
			rospy.loginfo("Showing extreme " + str(msg.extreme_num) + " of grasp idx " + str(msg.grasp_idx) + " (absolute grasp set " + str(msg.grasp_num) + "for obj " + str(msg.obj_num) + " and sub " + str(msg.sub_num))

		extreme_bag.close()
