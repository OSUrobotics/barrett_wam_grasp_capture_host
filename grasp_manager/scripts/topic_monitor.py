# File: topic_monitor.py
# Description: This file provides a class TopicMonitor which will 
#	subscribe to a given topic with a given callback and timeout
#	and listen to the topic, calling the callback if the time between
#	messages received on the topic ever exceeds the given timeout
# Notes: The monitor pauses for the duration of the callback
import rospy
import threading

class TopicMonitor:
	# Parameters:
	#	name: an ID for errors
	#	topic: string name of topic
	#	timeout: ROS duration
	#	cb: callback function to execute.
	def __init__(self, name, topic, timeout, cb):
		self.name = name
		self.topic = topic
		self.timeout = timeout
		self.cb = cb
		self.should_stop = False

		self.recv_msg = False
		self.recv_msg_lock = threading.Lock()
                
                self.init_sub()

		self.monitor_thread = threading.Thread(target=self.monitor)
		self.paused = False
		self.pause_lock = threading.Lock()
		self.monitor_thread.start()

        # Subscribe to the monitor topic, if possible
        def init_sub(self):
                topic_class = rostopic.get_topic_class(self.topic, blocking=False)[0]
                if topic_class == None:
                    rospy.logerr("%s monitor %s topic not instantiated. Monitor useless.")
                    raise Exception("see ros log.")
                self.topic_sub = rospy.Subscriber(topic, topic_class, self.sub_cb)
	
        def monitor(self):
		while not rospy.is_shutdown() and not self.should_stop:
			# Wait for a message
			self.pause_lock.acquire()
			self.pause_lock.release()
			rospy.sleep(timeout)

			# Check if we've received a message and reset it
			self.recv_msg_lock.acquire()
			recv_msg = self.recv_msg
			self.recv_msg = False
			self.recv_msg_lock.release()
			
			if recv_msg != True:
				self.cb()
		rospy.loginfo("%s monitor terminating." % self.name)

	def sub_cb(self, msg):
		self.recv_msg_lock.acquire()
		self.recv_msg = True
		self.recv_msg_lock.release()

	def pause_monitor(self):
		if self.paused:
			rospy.logerr("%s monitor already paused!" % self.name)
			return
		self.pause_lock.acquire()
		self.paused = True

	def resume_monitor(self):
		if not self.paused:
			rospy.logerr("%s monitor cannot resume: not paused" % self.name)
			return
		self.pause_lock.release()
		self.paused = False

	def kill_monitor(self):
		self.should_stop = True
		if self.paused:
			self.pause_lock.release()
                rospy.loginfo("Joining %s monitor" % self.name)
                self.monitor_thread.join()
                rospy.loginfo("Finished joining %s monitor" % self.name)
