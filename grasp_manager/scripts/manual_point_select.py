#! /usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from grasp_manager.msg import GraspSnapshot

from copy import deepcopy

depth_image = None
rgb_image = None

def get_click_pos(event, x, y, flags, param):
	global mouse_click_coords, depth_image, rgb_image
	if event == cv2.EVENT_LBUTTONDOWN:
		mouse_click_coords = [x,y]
		if depth_image != None:
			cv2.circle(rgb_image,(x,y), 5, (0,255,0), -1)
			cv2.imshow("rgb_image", rgb_image)
			d = depth_image[y][x] / 1000.0
			print "X: ", x, "Y: ", y, "D: ", d
			print x, "\t", y, "\t", d
			cv2.waitKey(0)
		else:
			rospy.logerr("No depth image loaded!")

# Recieve all of the information in one go and process it
def image_cb(msg):
	global cv_bridge, depth_image, rgb_image

	# Convert depth and rgb images
	rgb_image = cv_bridge.imgmsg_to_cv2(msg.rgb_image)
	depth_image = cv_bridge.imgmsg_to_cv2(msg.depth_image)

	# Show the images
	cv2.imshow("rgb_image", rgb_image)
	cv2.waitKey(0)
	
	user_input = raw_input("Good image for calibration? (y/n): ")
	if user_input.lower() != "y":
		rospy.loginfo("Skipping...")
		return

	# Save the original file
	base_path = '/home/eva/grasp_data/config/'
	base_file_name = "obj"+ str(msg.obj_num)  
	base_file_name += "_sub" + str(msg.sub_num) 
	base_file_name += "_grasp" + str(msg.grasp_num) 
	base_file_name += "_extreme" + str(msg.extreme_num)
	cv2.imwrite(base_path + base_file_name + "_orig.png", rgb_image)
	rospy.loginfo("Click on the rgb image to identify points. Press enter with image selected to continue.")
	cv2.waitKey(0)

	# Save the speckled file!
	raw_input("Press enter to save dotted file.")
	cv2.imwrite(base_path + base_file_name + "_dotted.png", rgb_image)

if __name__ == "__main__":
	rospy.init_node("point_selector")
	rospy.loginfo("Started point selector!")

	# Create the OpenCV viewer and machinery
	cv_bridge = CvBridge()
	cv2.namedWindow("rgb_image", cv2.WINDOW_AUTOSIZE)
	cv2.setMouseCallback("rgb_image", get_click_pos)

	rospy.loginfo("Starting subscriber.")
	rospy.Subscriber("/grasp_extremes", GraspSnapshot, image_cb)

	rospy.spin()

