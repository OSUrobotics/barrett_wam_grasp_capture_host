import rospy
from bag_tools.srv import *

# Provides an easy API to the bag_tools bag recorder interface
#	No safety checks are made  when opening or closing bags
#	it is the caller's responsibility to remember bag ids and
#	manage errors
class BagManager:
	def __init__(self):
		self.recorder_start_topic = "start_record"
		self.recorder_stop_topic = "stop_record"

		rospy.loginfo("Waiting for bag_tools' recorder services to become available.")
		rospy.wait_for_service(self.recorder_start_topic)
		rospy.wait_for_service(self.recorder_stop_topic)
		
		self.record_start_srv = rospy.ServiceProxy(self.recorder_start_topic, recordStart)
		self.record_stop_srv = rospy.ServiceProxy(self.recorder_stop_topic, recordStop)
	
	# Begins a bag file recording, always creating the bag path if
	#	it doesn't exist
	# Returns (None) on failure
	def start_recording(self, bag_name, topic_list):
		bag_id = None
		try:
			res = self.record_start_srv(bag_name, topic_list, True)
			if res.ret != res.SUCCESS:
				rospy.logerr("Cannot open bag file " + bag_path + " ret: " + str(res))
				return (False, None)
			else:
				bag_id = res.bag_id
		except:
			rospy.logerr("Cannot open bag file: " + bag_path)
			return (False, None)

		return (bag_id, None)

	# Returns a tuple with the first element
	#	  true on success or 
	#	  false on failure
	def stop_recording(self, bag_id):
		try:
			res = self.record_stop_srv(bag_id)
			if res.ret != res.SUCCESS:
				rospy.logerr("Trouble closing bag file with id: " + bag_id)
				return (False, res)

		except:
			rospy.logerr("Error during service call closing bag with id: " + bag_id)
			return (False, None)

		return (True, None)


