#! /usr/bin/env python
# Description: Iterates through all available grasp snapshot data (grasp_extreme_snapshots.bag files)
#	in the directory specified by grasp_data_directory and saves png snapshots to file according
#	to object number
import rospy
import rosbag

from grasp_manager.shared_playback import *

import cv2
from cv_bridge import CvBridge, CvBridgeError

grasp_pic_dir = "/home/eva/grasp_pics"

if __name__ == "__main__":
	rospy.init_node("image_extractor")

	cv_bridge = CvBridge()

	# Get data directories
	data_dirs = get_data_dirs(grasp_data_directory)

	for (data_dir_path, obj_num, sub_num) in data_dirs:
		extreme_bag = None
		try:
			extreme_bag = rosbag.Bag(data_dir_path + "/" + "grasp_extreme_snapshots.bag", "r")
		except:
			rospy.loginfo("Troubling opening extreme bag in dir " + data_dir_path)
			continue

		# Distinguish between good and bad directory paths
		pic_dir_name = "obj" + str(obj_num)
		if "good" in data_dir_path:
			pic_path = grasp_pic_dir + "/good/" + pic_dir_name
		else:	
			pic_path = grasp_pic_dir + "/bad/" + pic_dir_name
		
		# Create a directory for the object and store all relevant
		#	grasp pictures in it
		if not os.path.exists(pic_path):
			os.makedirs(pic_path)
		rospy.loginfo("Processing " + data_dir_path)
		for topic, msg, t in extreme_bag.read_messages():
			pic_name = "sub" + str(msg.sub_num) + "_grasp" + str(msg.grasp_num)
			if msg.is_optimal:
				pic_name += "_optimal" + str(msg.stamp)
			else:
				pic_name += "_extreme" + str(msg.extreme_num)

			rgb_path = pic_path + "/" + pic_name + ".png"
		
			if os.path.exists(rgb_path):
				continue

			# Write the image to disk
			rospy.loginfo("Writing " + pic_name)
			rgb_image = cv_bridge.imgmsg_to_cv2(msg.rgb_image)
			cv2.imwrite(rgb_path, rgb_image)

