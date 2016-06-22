import getpass
import os

local_data_path = os.path.expanduser("~") + "/grasp_data"	# The fully qualified path to the grasp data
harddrive_path = "/media/" + getpass.getuser() + "/FA648F24648EE2AD/grasp_data"	# The fully qualified path to the grasp data

# Default grasp data location
grasp_info_dir = harddrive_path

kinect_topic_prefix = "/kinect2/qhd/"
kinect_data_topics = [kinect_topic_prefix + "image_color_rect/compressed", kinect_topic_prefix + "image_depth_rect/compressed"]
