# File: topic_oneshot.py
# Description: This file instantiates a ROS node that will listen to a
#	given topic and buffer the last message received on that topic.
#	When the user commands (or the proper service is triggered), the 
#	node will publish the most recent message on the given output topic
# Parameters:
#	in_topic: The topic to buffer
#	out_topic: The topic to output on
import rospy
import rostopic
import threading

class TopicOneShot:
	def __init__(self, in_topic, out_topic):
		self.in_topic = in_topic
		self.out_topic = out_topic

		self.cur_msg = None
		self.cur_msg_lock = threading.Lock()
		self.subscribe()
		self.pub = rospy.Publisher(self.in_topic, self.topic_class, queue_size=1, latch=True)
	
	def subscribe(self):
		self.topic_class = rostopic.get_topic_class(self.topic, blocking=False)[0]
		if self.topic_class == None:
			rospy.logerr("%s topic not instantiated. One-shot useless." % self.in_topic)
			raise Exception("see ros log.")
	
		self.sub = rospy.Subscriber(self.in_topic, self.topic_class, self.sub_cb)
	
	def sub_cb(self, msg):
		self.cur_msg_lock.acquire()
		self.cur_msg = msg
		self.cur_msg_lock.release()
		

def get_args():
	in_topic_full_name = rospy.search_param('~in_topic')
	in_topic = rospy.get_param(in_topic_full_name)
	out_topic_full_name = rospy.search_param('~out_topic')
	out_topic = rospy.get_param(out_topic_full_name)

	return in_topic, out_topic

if __name__ == "__main__":
	self.init_node('topic_oneshot', anonymous=True)

	in_topic, out_topic = get_args()

	oneshot = TopicOneShot()

	while not rospy.is_shutdown():
		raw_input("Press [Enter] to trigger the one shot")
