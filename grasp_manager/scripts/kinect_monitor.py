import rospy
from sensor_msgs.msg import CompressedImage, Image

import threading

# Currently only monitors the depth topic
class KinectMonitor:
	def __init__(self, depth_topic):
		self.acceptable_kinect_publish_time = 0.5 # Includes startup delay
		self.monitor_period = 3 # seconds
		self.recv_depth_msg = False
		self.depth_topic = depth_topic

		rospy.loginfo("Started kinect depth monitor for topic " + depth_topic)

		self.monitor_thread = threading.Thread(target=self.monitor)
		self.monitor_lock = threading.Lock()
		self.monitor_lock.acquire()
		self.monitor_thread.start()

	def monitor(self):
		self.continue_monitor = True
		period = rospy.Duration(self.monitor_period)

		while not rospy.is_shutdown() and self.continue_monitor:
			self.monitor_lock.acquire()
			self.check_kinect_depth(self.acceptable_kinect_publish_time)
			self.monitor_lock.release()
			
			rospy.sleep(period)

	def start_monitor(self):
		try:
			self.monitor_lock.release()
		except threading.ThreadError:
			rospy.logerr("Trying to release the monitor lock, but the main thread doesn't own it.")
			return

		rospy.loginfo("Kinect monitoring started.")

	def stop_monitor(self):
		self.monitor_lock.acquire()
		rospy.loginfo("Kinect monitoring stopped.")

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
			if self.check_kinect_depth(self.acceptable_kinect_publish_time):
				rospy.loginfo("Kinect is publishing depth information.")
				break
			else:
				rospy.logerr("Kinect is not publishing depth data!")
				raw_input("Please reset the kinect and then press [Enter]")

	def depth_callback(self, msg):
		self.recv_depth_msg = True
		self.depth_sub.unregister()
