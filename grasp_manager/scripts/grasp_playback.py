## Currently, the playback functionality has been suspended and the following code
## 	probably does not work. It also needs the appropriate driver file.
import rospy
from wam_srvs.srv import *
from wam import *
from hand_logger import *

# Resets the hand and WAM arm
class GraspPlayback:
	def __init__(self):
		self.wam = wam()
		self.hand_cmd_blk_srv rospy.ServiceProxy("/bhand/hand_cmd_blocking", HandCommandBlk)
		self.hand_playback_start_pub = rospy.Publisher('start_hand_playback', String, queue_size=1)
		self.hand_playback_stop_pub = rospy.Publisher('stop_hand_playback', EmptyM, queue_size=1)


	def setup_playback(self, bag_file):
		setup_hand(self.hand_cmd_blk_srv)

		wam.move_wam_traj_onboard(bag_file)
		wam.load_traj()

	def commence_playback(self, grasp_dir):
		bag_file = grasp_dir + wam.wam_traj_name
		try:
			self.setup_playback(bag_file)
			wam.start_playback()
			self.hand_playback_start_pub.publish(grasp_dir + hand_logger.bag_name)
			raw_input("Press [Enter] when playback is complete.")
		except:
			rospy.logerr("Playback aborted.")
			pass

if __name__ == "__main__":
	rospy.init_node("grasp_playback")
	grasp_replay = GraspPlayback()

	while not rospy.is_shutdown():
		# TODO: Get the grasp information
# Playback usage
#commence_playback(hand_cmd_blk_srv, playback_load_srv, playback_start_pub, hand_playback_start_pub, hand_logger, cur_grasp_data.get_log_dir())
