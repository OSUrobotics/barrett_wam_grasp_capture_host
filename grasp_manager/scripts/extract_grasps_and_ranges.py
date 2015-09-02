#! /usr/bin/env python
import rospy
import rosbag
import rospkg

from std_msgs.msg import String
from wam_msgs.msg import StampedString
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2, JointState
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
			raise IOError

		return self.msg_data[data_key]

# Returns a dictionary of the timestamps of extreme ranges
#	indexed by the grasp set number. Each entry is a list
# 	corresponding to the number of extremes specified
def get_grasp_timestamps(annotation_bag_path):
	annotations_bag = rosbag.Bag(annotation_bag_path)
	relevant_grasp_timestamps = {}
	task = "pickup"
	for topic, msg, t in annotations_bag.read_messages(topics=['/grasp_annotations']):
		# Find grasp timestamps
		if "Start of natural task" in msg.data:
			task = "natural"
			continue
		
		if "Grasp range extreme" in msg.data or "Optimal grasp for" in msg.data:
			if "Grasp range extreme" in msg.data:
				op_str = "e"
			else:
				op_str = "o"
			file_grasp_num = get_grasp_num(msg.data)
			try:
				relevant_grasp_timestamps[file_grasp_num].append((msg.stamp, file_grasp_num, op_str, task))
			except KeyError:
				relevant_grasp_timestamps[file_grasp_num] = [(msg.stamp, file_grasp_num, op_str, task)]
	return relevant_grasp_timestamps

# Republishes relevant data at the request timestamp for depth registration
#	and pointcloud processing
# We want to publish a lot of frames to get an accurate kinect-robot transform, but
#	we also want the pointcloud from a specific second. So, we publish few depth
#	images and lots of rgb images.
def repub_data_stamp(grasp_stamp, img_depth_bag, cam_info_spammer):
	max_num_pub_msgs = 10
	pub_next_msgs = -1
	play_time = rospy.Duration(3.0)
	lead_time = rospy.Duration(0.5)

	rospy.loginfo("Starting depth/rgb playback searching for stamp " + str(grasp_stamp))
	depth_count = -1
	rgb_img = None
	played_depth = played_rgb = 0
	for topic, msg, t in img_depth_bag.read_messages(start_time=(grasp_stamp - lead_time), end_time=(grasp_stamp + play_time)):
		if msg.header.stamp >= grasp_stamp and "color" in topic and pub_next_msgs == -1:
			rgb_img = msg
			pub_next_msgs = max_num_pub_msgs
		
		if pub_next_msgs > 0 and "color" in topic:
			clock_pub.publish(t)
			cam_info_spammer.publish_info(msg.header.stamp)
			if "compressed" in topic:
				rgb_pub.publish(msg)
			else:
				rgb_uncomp_pub.publish(msg)
			played_rgb = 1
			pub_next_msgs -= 1
			time.sleep(0.01)
			#print "Image, camera info, and clock published."
			continue

		if pub_next_msgs > 0 and "depth" in topic:
			if "compressed" in topic:
				depth_pub.publish(msg)
			else:
				depth_uncomp_pub.publish(msg)

			played_depth = 2
			#print "Published depth image."
			continue
			
		if pub_next_msgs == 0:
			break

	return played_depth + played_rgb

def get_joint_values(data_dir_path, grasp_stamp):
	arm_jnts = JointState()
	hand_jnts = JointState()
	
	try:
		rospy.loginfo("Loading arm bag from " + data_dir_path + " looking for stamp " + str(grasp_stamp))
		arm_bag = rosbag.Bag(data_dir_path + "/" + "wam_traj.bag", "r")
		arm_jnts = msg_from_bag(arm_bag, ["/wam_grasp_capture/recording/joint_states", "wam_jnts"], grasp_stamp)
		arm_bag.close()
		if arm_jnts == None:
			rospy.logerr("Couldn't get wam arm joints for this grasp.")
			arm_jnts = JointState()
	except:
		rospy.logerr("Couldn't get arm joints for this grasp. Probably issues with bag loading.")
		pass

	try:
		hand_bag = rosbag.Bag(data_dir_path + "/" + "hand_commands.bag", "r")
		hand_jnts = msg_from_bag(hand_bag, ["/bhand/joint_states"], grasp_stamp)
		hand_bag.close()
		if hand_jnts == None:
			rospy.logerr("Couldn't get barrett hand joints for this grasp.")
			hand_jnts = JointState()

	except:
		rospy.logerr("Couldn't get barrett hand joints. Probably issues with bag loading.")
		pass

	return (arm_jnts, hand_jnts)

def msg_from_bag(bag, topics, time_stamp):
	lead_time = rospy.Duration(0.5)
	play_time = rospy.Duration(0.5)
	for topic, msg, t in bag.read_messages(topics=topics, start_time=(grasp_stamp - lead_time), end_time=(grasp_stamp + play_time)):
		#print "data: ", msg
		if t >= time_stamp:
			return msg
	# No luck
	return None

if __name__ == "__main__":
	rospy.init_node("extract_extremes")
	print "Data directory: ", grasp_data_directory

	# ROS republishing interface
	color_topics = [s for s in kinect_data_topics if "color" in s]
	depth_topics = [s for s in kinect_data_topics if "depth" in s]
	uncompressed_color_topic = "/".join(color_topics[0].split('/')[:-1])
	uncompressed_depth_topic = "/".join(depth_topics[0].split('/')[:-1])
	rospy.loginfo("Uncompressed color topic: " + uncompressed_color_topic + " depth: " + uncompressed_depth_topic)

	rgb_pub   = rospy.Publisher(color_topics[0], CompressedImage, queue_size=0)
	rgb_uncomp_pub = rospy.Publisher(uncompressed_color_topic, Image, queue_size=0, latch=True)
	depth_pub = rospy.Publisher(depth_topics[0], CompressedImage, queue_size=0, latch=True)
	depth_uncomp_pub = rospy.Publisher(uncompressed_depth_topic, Image, queue_size=0, latch=True)
	clock_pub = rospy.Publisher("/clock", Clock, queue_size=0)
	cam_info_spammer = CamInfoSpammer()

	data_dirs = get_data_dirs(grasp_data_directory)
	for (data_dir_path, obj_num, sub_num) in data_dirs:
		output_bag_path = data_dir_path + "/" + extreme_bag_name
		if os.path.exists(output_bag_path):
			rospy.loginfo("Skipping " + output_bag_path + " since it already exists.")
			continue

		# Iterate through relevant bag file for images
		relevant_grasp_timestamps = {}
		try:
			relevant_grasp_timestamps = get_grasp_timestamps(data_dir_path + "/" + "robot_grasp_annotations.bag")
		except:
			rospy.logerr("Cannot open annotations bag in " + data_dir_path + ". Skipping in the interest of saving time.")
			continue
		
		print "Available grasps and timestamps: ", relevant_grasp_timestamps
		print "Data dir: ", data_dir_path
		# Process results and save to output bag
		output_bag = rosbag.Bag(output_bag_path, "w")
		for grasp_set_num in relevant_grasp_timestamps:
			grasp_set = relevant_grasp_timestamps[grasp_set_num]
			extreme_num = 0
			optimal_num = 0
			for idx, grasp_tuple in enumerate(grasp_set):
				grasp_stamp = grasp_tuple[0]
				grasp_abs_num = grasp_tuple[1]

				# Create the unified message
				img_depth_bag = rosbag.Bag(data_dir_path + "/" + "kinect_robot_capture.bag")
				relevant_data = Recapturer()
				data_retrievable = -1
				grasp_cloud = None
				while True:
					data_retrievable = repub_data_stamp(grasp_stamp, img_depth_bag, cam_info_spammer)
					if data_retrievable < 3:
						break
					try:
						grasp_cloud = relevant_data.get_data('ptcloud')
						break
					except:
						# Republish until it works. The listeners just need to be fired up
						rospy.loginfo("Retrying publishing for pointclouds...")
						pass
				if data_retrievable == 1:
					# RGB and no depth
					rospy.logwarn("RGB/Depth data not retrievable for " + str(grasp_stamp))
					grasp_cloud = PointCloud2()
				elif data_retrievable == 2:
					# Depth and no RGB
					rospy.logwarn("No RGB data. Skipping.")
					continue
				## AR Marker transforms. Killed because it's inaccurate. Switching to ICP against robot arm
				#marker_pose = relevant_data.get_data('marker_pose')
				#try:
				#	out_cloud = cloud_transformer(marker_pose, grasp_cloud).cloud
				#except rospy.ServiceException, e:
				#	rospy.logerr("Transformation call failed!")

				robot_jnts = get_joint_values(data_dir_path, grasp_stamp)

				grasp = GraspSnapshot()
				grasp.stamp = grasp_stamp
				grasp.obj_num = int(obj_num)
				grasp.sub_num = int(sub_num)
				grasp.grasp_num = int(grasp_abs_num)
				grasp.grasp_idx = int(grasp_set_num)
				if grasp_tuple[2] == "o":
					grasp.is_optimal = True
					grasp.optimal_num = optimal_num
					grasp.extreme_num = 0
					optimal_num += 1
				else:
					grasp.is_optimal = False
					grasp.extreme_num = extreme_num
					grasp.optimal_num = 0
					extreme_num += 1
				grasp.task = grasp_tuple[3]
				grasp.cloud_image = grasp_cloud
				try:
					grasp.rgb_image = relevant_data.get_data('rgb')
				except:
					rospy.logerr("Missing rgb, skipping.")
					continue
				try:
					grasp.cam_info = relevant_data.get_data('camera_info')
					grasp.depth_image = relevant_data.get_data('depth')
				except:
					rospy.logerr("Missing one of the key messages. Continuing anyway.")
					grasp.depth_image = Image()
				grasp.wam_joints = robot_jnts[0]
				grasp.hand_joints = robot_jnts[1]

				# Save the data to output file
				output_bag.write(grasp_extreme_topic, grasp)

		output_bag.flush()
		output_bag.close()
