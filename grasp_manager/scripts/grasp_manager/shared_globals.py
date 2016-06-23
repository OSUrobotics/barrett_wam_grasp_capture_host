import getpass
import os
import rospkg

valid_grasp_dir = rospkg.RosPack().get_path('valid_grasp_generator')
catkin_ws_location = valid_grasp_dir[:-26]

user = getpass.getuser()
#base_dir = os.path.expanduser("~") + "/grasping_user_study" # The fully qualified path to the grasp data
base_dir = "/media/" + user + "/FA648F24648EE2AD/grasping_user_study" # The fully qualified path to the grasp data

# Default grasp data location
grasp_info_dir = base_dir + "/grasp_data"

obj_transform_dir = base_dir + "/processed_data/grasp_transforms/"

grasping_data_folder = base_dir + '/grasping_data/'

kinect_topic_prefix = "/kinect2/qhd/"
kinect_data_topics = [kinect_topic_prefix + "image_color_rect/compressed", kinect_topic_prefix + "image_depth_rect/compressed"]
