import rospy
from sensor_msgs.msg import CompressedImage, Image

import threading

# Currently only monitors the depth topic
class KinectMonitor:
	def __init__(self, depth_topic):
		self.recv_depth_msg = False
		self.continue_monitor = True
		self.depth_topic = depth_topic

		rospy.loginfo("Started kinect depth monitor for topic " + depth_topic)

	def monitor(self):
		self.continue_monitor = True
		period = rospy.Duration(5.0)
		while not rospy.is_shutdown() and self.continue_monitor:
			check_kinect_depth(0.1)
			rospy.sleep(period)

		rospy.loginfo("Kinect monitoring stopped.")

	def stop_monitor(self):
		self.continue_monitor = False

	# Parameters: delay - time to wait for messages in seconds (float)
	def check_kinect_depth(self, delay):
		delay = rospy.Duration(delay)
		self.recv_depth_msg = False
		self.depth_sub = rospy.Subscriber(self.depth_topic, CompressedImage, self.depth_callback)

		rospy.sleep(delay)
		
		if not self.recv_depth_msg:
			rospy.logerr("Kinect depth data stream down!")
			return False

		return True

	def block_for_kinect(self):
		while True:
			if self.check_kinect_depth(0.5):
				rospy.loginfo("Kinect is publishing depth information.")
				break
			else:
				rospy.logerr("Kinect is not publishing depth data!")
				raw_input("Please reset the kinect and then press [Enter]")

	def depth_callback(self, msg):
		self.recv_depth_msg = True
		self.depth_sub.unregister()
