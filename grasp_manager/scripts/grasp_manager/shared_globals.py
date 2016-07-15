import getpass
import os
import rospkg

valid_grasp_dir = rospkg.RosPack().get_path('valid_grasp_generator')
catkin_ws_location = valid_grasp_dir[:-26]

user = getpass.getuser()
local_data_path = os.path.expanduser("~") + "/grasping_user_study" 
#harddrive_path = "/media/" + user + "/FA648F24648EE2AD/grasping_user_study" 
harddrive_path = "/media/" + user + "/FA648F24648EE2AD/grasp_study_2015" 

# Default grasp data location
base_dir = harddrive_path
grasp_info_dir = base_dir + "/grasp_data"

obj_transform_dir = base_dir + "/processed_data/grasp_transforms/"

grasping_data_folder = base_dir + '/grasping_data/'

kinect_topic_prefix = "/kinect2/qhd/"
kinect_data_topics = [kinect_topic_prefix + "image_color_rect/compressed", kinect_topic_prefix + "image_depth_rect/compressed"]

# All topics necessary for image capture and physical scene reconstruction
scene_capture_topics = kinect_data_topics + ['/camera1/depth/points/world_frame', 'camera2/depth/points/world_frame']
