#! /usr/bin/env python
import rospy
import rosbag

from grasp_manager.msg import GraspSnapshot
from sensor_msgs.msg import Image, JointState, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from shared_globals import *
from shared_playback import *

import csv

def save_cloud(filename, cloud):
	f_handle = open(filename, "w")
	csv_writer = csv.writer(f_handle, delimiter=",")
	gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))
	csv_writer.writerow(['x', 'y', 'z', 'rgb'])

	printed = None
	for pt in gen:
		if printed == None:
			print "point: ", pt
			printed = True
		csv_writer.writerow(pt)

if __name__ == "__main__":
	rospy.init_node("extraction_replay")
	rospy.loginfo("Replay node online!")
	
	wam_joint_state_pub = rospy.Publisher("/wam/joint_states", JointState, queue_size=1, latch=True)
	hand_joint_state_pub = rospy.Publisher("/bhand/joint_states", JointState, queue_size=1, latch=True)
	rgb_image_pub = rospy.Publisher("/grasp_rgb", Image, queue_size=1, latch=True)
	depth_image_pub = rospy.Publisher("/grasp_depth", Image, queue_size=1, latch=True)
	
	ptcloud_pub = rospy.Publisher("/grasp_cloud", PointCloud2, queue_size=1, latch=True)
	
	data_repub = rospy.Publisher("/grasp_extremes", GraspSnapshot, queue_size=3, latch=True)
	while not rospy.is_shutdown():
		print "obtaining data dir"
		data_dir, obj_num, sub_num = get_data_dir()

		defrag_path = data_dir + "_defrag" + "/" + extreme_bag_name
		extreme_bag_path = data_dir + "/" + extreme_bag_name
		if os.path.exists(defrag_path):
			extreme_bag_path = defrag_path
		elif os.path.exists(extreme_bag_path):
			rospy.logdebug("Bag found.")
		else:
			rospy.logwarn("Snapshot bagfile not found: " + extreme_bag_path)
			continue

		rospy.loginfo("Opening bag: " + extreme_bag_path)
		extreme_bag = rosbag.Bag(extreme_bag_path, "r")

		for topic, msg, t in extreme_bag.read_messages(topics=[grasp_extreme_topic]):
			user_input = raw_input("Press anything to show grasp, n to stop, and s to skip: ")
			if user_input.lower() == "n":
				break
			if user_input.lower() =="s":
				continue

			
			#user_input = raw_input("Save pointcloud? (y/n): ")
			#if user_input.lower() == "y":
			#	filename = "obj" + str(msg.obj_num) + "_sub" + str(msg.sub_num) + "_grasp" + str(msg.grasp_num) + "_pointcloud.csv"
			#	save_cloud(filename, msg.cloud_image)
			
			data_repub.publish(msg)
			wam_joint_state_pub.publish(msg.wam_joints)
			hand_joint_state_pub.publish(msg.hand_joints)
			rgb_image_pub.publish(msg.rgb_image)
			depth_image_pub.publish(msg.depth_image)
			ptcloud_pub.publish(msg.cloud_image)
			o_or_e = ""
			o_or_e_num = 0
			if msg.is_optimal:
				o_or_e = "optimal"
				o_or_e_num = msg.optimal_num
			else:
				o_or_e = "extreme"
				o_or_e_num = msg.extreme_num
			rospy.loginfo("Showing " + o_or_e  + " " + str(o_or_e_num) + " of grasp idx " + str(msg.grasp_idx) + " (absolute grasp set " + str(msg.grasp_num) + ") for obj " + str(msg.obj_num) + " and sub " + str(msg.sub_num))

		extreme_bag.close()
