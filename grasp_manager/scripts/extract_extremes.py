#! /usr/bin/env python
import rospy
import rosbag
import rospkg

from std_msgs.msg import String
from wam_msgs.msg import StampedString
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2
from rosgraph_msgs.msg import Clock
from ar_track_alvar_msgs.msg import AlvarMarkers as ARMarker
from grasp_manager.msg import GraspSnapshot
from grasp_manager.srv import TransformCloud



import sys
import time

from shared_globals import *
from shared_playback import *

class CamInfoSpammer:
	def __init__(self):
		global kinect_topic_prefix
		self.cam_info_msg = self.get_camera_info_msg()
		self.camera_info_pub = rospy.Publisher(kinect_topic_prefix + "camera_info", CameraInfo, queue_size=1)

	def publish_info(self, stamp):
		self.cam_info_msg.header.stamp = stamp
		self.camera_info_pub.publish(self.cam_info_msg)

	def get_camera_info_msg(self):
		rospack = rospkg.RosPack()
		camera_info_pkg_path = rospack.get_path("grasp_manager")
		camera_info_bag_path = camera_info_pkg_path + "/config/kinect2_cam_info.bag"
		camera_info_bag = rosbag.Bag(camera_info_bag_path)
		for topic, msg, t in camera_info_bag.read_messages(topics=[kinect_topic_prefix + "camera_info"]):
			camera_info_bag.close()
			return msg

		# If we reach here, its a problem
		rospy.logerr("No camera info message found in " + camera_info_bag_path)
		sys.exit(1)

# Hosts specific data recapturing mechanisms for getting depth registered
#	pointclouds out of our data
class Recapturer:
	def __init__(self):
		# Data to capture
		self.msg_data = {}
		self.msg_data['ptcloud'] = None
		self.msg_data['rgb'] = None
		self.msg_data['depth'] = None
		self.msg_data['camera_info'] = None
		self.msg_data['marker_pose'] = None
		
		# Subscribers to listen to
		self.subscribers = []
		self.subscribers.append(rospy.Subscriber("/kinect2/qhd/depth_registered/points", PointCloud2, self.generic_capture, callback_args='ptcloud'))
		self.subscribers.append(rospy.Subscriber("/kinect2/qhd/image_depth_rect", Image, self.generic_capture, callback_args='depth'))
		self.subscribers.append(rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.generic_capture, callback_args='rgb'))
		self.subscribers.append(rospy.Subscriber("/kinect2/qhd/camera_info", CameraInfo, self.generic_capture, callback_args='camera_info'))
		self.subscribers.append(rospy.Subscriber("/ar_pose_marker", ARMarker, self.marker_capture))

	def generic_capture(self, msg, dict_key):
		self.msg_data[dict_key] = msg

	def marker_capture(self, marker_msg):
		for marker in marker_msg.markers:
			if marker.id == 3:
				self.msg_data['marker_pose'] = marker.pose.pose

	def get_data(self, data_key):
		if self.msg_data[data_key] == None:
			rospy.logerr("No message recorded for key " + str(data_key))
			sys.exit(1)

		return self.msg_data[data_key]

# Returns a dictionary of the timestamps of extreme ranges
#	indexed by the grasp set number. Each entry is a list
# 	corresponding to the number of extremes specified
def get_extreme_timestamps(annotation_bag_path):
	annotations_bag = rosbag.Bag(annotation_bag_path)
	relevant_grasp_timestamps = {}
	grasp_set_num = -1
	for topic, msg, t in annotations_bag.read_messages(topics=['/grasp_annotations']):
		# Keep track of grasp number
		if "Optimal" in msg.data:
			grasp_set_num += 1
			continue
		
		# Find image timestamps
		if "Grasp range extreme" in msg.data:
			try:
				relevant_grasp_timestamps[grasp_set_num].append(msg.stamp)
			except KeyError:
				relevant_grasp_timestamps[grasp_set_num] = [msg.stamp]
	return relevant_grasp_timestamps


# Republishes relevant data at the request timestamp for depth registration
#	and pointcloud processing
# We want to publish a lot of frames to get an accurate kinect-robot transform, but
#	we also want the pointcloud from a specific second. So, we publish few depth
#	images and lots of rgb images.
def repub_data_stamp(grasp_stamp, img_depth_bag, cam_info_spammer):
	max_num_pub_msgs = 100
	pub_next_msgs = -1
	play_time = rospy.Duration(3.0)
	lead_time = rospy.Duration(0.5)

	rospy.loginfo("Starting playback searching for stamp " + str(grasp_stamp))
	depth_count = 10
	for topic, msg, t in img_depth_bag.read_messages(topics=kinect_data_topics, start_time=(grasp_stamp - lead_time), end_time=(grasp_stamp + play_time)):
		if msg.header.stamp >= grasp_stamp and "color" in topic and pub_next_msgs == -1:
			pub_next_msgs = max_num_pub_msgs
		
		if pub_next_msgs > 0 and "color" in topic:
			clock_pub.publish(t)
			cam_info_spammer.publish_info(msg.header.stamp)
			rgb_pub.publish(msg)
			pub_next_msgs -= 1
			time.sleep(0.01)
			print "Image, camera info, and clock published."
			continue

		if pub_next_msgs > 0 and depth_count > 0 and "depth" in topic:
			depth_pub.publish(msg)
			depth_count -= 1
			print "Published depth image."
			continue
			
		if pub_next_msgs == 0:
			break
		
if __name__ == "__main__":
	rospy.init_node("extract_extremes")
	print "Data directory: ", grasp_data_directory

	# ROS republishing interface
	color_topics = [s for s in kinect_data_topics if "color" in s]
	depth_topics = [s for s in kinect_data_topics if "depth" in s]
	rgb_pub   = rospy.Publisher(color_topics[0], CompressedImage, queue_size=0)
	depth_pub = rospy.Publisher(depth_topics[0], CompressedImage, queue_size=0, latch=True)
	clock_pub = rospy.Publisher("/clock", Clock, queue_size=0)
	cam_info_spammer = CamInfoSpammer()
	rospy.wait_for_service('transform_pointcloud')
	cloud_transformer = rospy.ServiceProxy('transform_pointcloud', TransformCloud)

	while not rospy.is_shutdown():
		data_dir_path, obj_num, sub_num = get_data_dir()

		# Iterate through relevant bag file for images
		relevant_grasp_timestamps = get_extreme_timestamps(data_dir_path + "/" + "robot_grasp_annotations.bag")
		print "Available grasps and timestamps: ", relevant_grasp_timestamps

		# Process results and save to output bag
		output_bag = rosbag.Bag(data_dir_path + "/" + extreme_bag_name, "w")
		for grasp_set_num in relevant_grasp_timestamps:
			grasp_set = relevant_grasp_timestamps[grasp_set_num]
			extreme_num = 0
			for idx, grasp_stamp in enumerate(grasp_set):
				# Create the unified message
				img_depth_bag = rosbag.Bag(data_dir_path + "/" + "kinect_robot_capture.bag")
				relevant_data = Recapturer()
				repub_data_stamp(grasp_stamp, img_depth_bag, cam_info_spammer)
				
				grasp_cloud = relevant_data.get_data('ptcloud')
				marker_pose = relevant_data.get_data('marker_pose')
				try:
					out_cloud = cloud_transformer(marker_pose, grasp_cloud).cloud
				except rospy.ServiceException, e:
					rospy.logerr("Transformation call failed!")

				grasp = GraspSnapshot()
				grasp.stamp = grasp_stamp
				grasp.obj_num = int(obj_num)
				grasp.sub_num = int(sub_num)
				grasp.grasp_num = int(grasp_set_num)
				grasp.extreme_num = extreme_num
				grasp.cloud_image = out_cloud
				grasp.rgb_image = relevant_data.get_data('rgb')
				grasp.depth_image = relevant_data.get_data('depth')
				grasp.cam_info = relevant_data.get_data('camera_info')
			
				# Save the data to output file
				output_bag.write(grasp_extreme_topic, grasp)

				extreme_num += 1

		output_bag.flush()
		output_bag.close()
