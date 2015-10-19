import rospy
import time

from wam_msgs.msg import HandCommand, StampedString
from wam_srvs.srv import *

prelog_hand_pose = [0,0,0,0]

class HandLogger:
	def __init__(self, bag_manager):
		self.bag_manager = bag_manager
		self.bag_name = "hand_commands.bag"

		self.hand_cmd_pub = rospy.Publisher("/bhand/hand_cmd", HandCommand,  queue_size=1)
		self.hand_cmd_blk_srv = rospy.ServiceProxy("/bhand/hand_cmd_blocking", HandCommandBlk)

		self.log_dir = None
		self.bag_id = None

	def set_log_dir(self, log_dir):
		if log_dir[-1] != "/":
			rospy.logerr("Invalid logging directory for hand data. Recorder expects a trailing '/'")
			return

		self.log_dir = log_dir

	# Preconditions: The hand has been sent to its home position
	def start_hand_capture(self):
		setup_hand(self.hand_cmd_blk_srv)
		time.sleep(1)
		if self.log_dir == None:
			rospy.logerr("No log directory set to record hand motion.")
			return
		
		bag_path = self.log_dir + self.bag_name
		topic_list = ["/bhand/hand_cmd", "/bhand/joint_states"]

		res = self.bag_manager.start_recording(bag_path, topic_list)
		if res[0] == False:
			rospy.logerr("Hand capture bagging returned " + str(res.response.ret))
		else:
			self.bag_id = res[0]
			rospy.sleep(0.3)
			self.add_first_hand_msg()
			return

		self.bag_id = None
		return

	def add_first_hand_msg(self):
		global prelog_hand_pose
		hc = HandCommand()
		hc.header.stamp = rospy.Time.now()
		hc.f1 = prelog_hand_pose[0]
		hc.f2 = prelog_hand_pose[1]
		hc.f3 = prelog_hand_pose[2]
		hc.spread = prelog_hand_pose[3]
		self.hand_cmd_pub.publish(hc)

	def end_hand_capture(self):
		if self.bag_id == None:
			rospy.logerr("Bag_id is none in handLogger. No bag file started.")
			return

		res = self.bag_manager.stop_recording(self.bag_id)
		if res[0] == False:
			rospy.logerr("Hand motion capture STOP UNSUCCESSFUL: " + str(res.response.ret))

		self.bag_id = None
		self.log_dir = None

def setup_hand(hand_cmd_blk_srv):
	global prelog_hand_pose
	req = HandCommandBlkRequest()
	req.command.f1 = prelog_hand_pose[0]
	req.command.f2 = prelog_hand_pose[1]
	req.command.f3 = prelog_hand_pose[2]
	req.command.spread = prelog_hand_pose[3]
	
	try:
		hand_cmd_blk_srv(req)
	except:
		rospy.logerr("Trouble resetting the hand position for playback/recording.")


