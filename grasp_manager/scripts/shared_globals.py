import getpass
import os

#grasp_info_dir = os.path.expanduser("~") + "/grasp_data"	# The fully qualified path to the grasp data
grasp_info_dir = "/media/" + getpass.getuser() + "/FA648F24648EE2AD/grasp_data"	# The fully qualified path to the grasp data
kinect_topic_prefix = "/kinect2/qhd/"
kinect_data_topics = [kinect_topic_prefix + "image_color_rect/compressed", kinect_topic_prefix + "image_depth_rect/compressed"]
