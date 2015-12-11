import rospy
import paramiko
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty as EmptyM
from wam_srvs.srv import HandCommandBlkRequest, HandCommandBlkResponse, HandCommandBlk, JointRecordPlayback, CartPosMove, CartPosMoveRequest, CartPosMoveResponse, GravityComp, GravityCompRequest, GravityCompResponse, PoseMove, PoseMoveRequest, PoseMoveResponse, OrtnMove, OrtnMoveRequest, OrtnMoveResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import sys

class PoseMoveException(Exception):
	def __init__(self, req_pos):
		self.req_pos = req_pos
	def __str__(self):
		return str(self.req_pos)

class BagManagerStopException(Exception):
	def __init__(self):
		pass
	def __str__(self):
		return "Could not stop bag manager from recording wam joint angles."

class NotRecordingException(Exception):
	def __init__(self):
		pass
	def __str__(self):
		return "WAM class was not recording. Yet it has been asked to stop."


# An interface to the WAM data collection and control mechanisms
class WAM:
	def __init__(self, bag_manager):
		self.bag_manager = bag_manager
		self.recording = False
		self.wam_bag_id = -1
		self.wam_sftp = None
		self.wam_jnt_topic = "/wam_grasp_capture/recording/joint_states"
		self.wam_traj_name = "wam_traj.bag"
		self.wam_traj_location = "/tmp/" + self.wam_traj_name
		self.cur_wam_pose = None
		self.transport_wam_pose = None

		self.init_wam_sftp()
		
		# Start up the ros interface
		rospy.loginfo("Waiting for wam services.")
		rospy.wait_for_service("/wam/cart_move")
		rospy.loginfo("Found wam services.")
		self.wam_home_srv = rospy.ServiceProxy('/wam/go_home', Empty)
		self.record_start_pub = rospy.Publisher('/wam_grasp_capture/jnt_record_start', EmptyM, queue_size=1)
		self.record_stop_pub = rospy.Publisher('/wam_grasp_capture/jnt_record_stop', EmptyM, queue_size=1)
		self.playback_load_srv = rospy.ServiceProxy('/wam_grasp_capture/jnt_playback_load', JointRecordPlayback)
		self.playback_start_pub = rospy.Publisher('/wam_grasp_capture/jnt_playback_start', EmptyM, queue_size=1)
		self.playback_stop_pub = rospy.Publisher('/wam_grasp_capture/jnt_playback_stop', EmptyM, queue_size=1)
		self.gravity_comp_srv = rospy.ServiceProxy("/wam/gravity_comp", GravityComp)
		self.wam_pos_sub = rospy.Subscriber('/wam/pose', PoseStamped, self.wam_pose_cb)
		move_wam = rospy.ServiceProxy("/wam/cart_move", CartPosMove)
		orient_wam = rospy.ServiceProxy("/wam/ortn_move", OrtnMove)

	def init_wam_sftp(self):
		rospy.loginfo("Initializing WAM SFTP connection.")

		client = paramiko.SSHClient()
		client.load_system_host_keys()
		client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		
		try:
			client.connect("WAM",username="robot")
		except paramiko.BadHostKeyException as e:
			print e.what()
		except paramiko.AuthenticationException as e:
			rospy.logerr("Authentication failed for wam ssh.")
			print e.what()
		except paramiko.SSHException as e:
			rospy.logerr("General ssh exception while connecting to WAM.")
			print e.what()
		except:
			rospy.logerr("Unknown exception while connecting to WAM via ssh.")
			rospy.logerr("Are you on the proper network?")
			sys.exit(1)

		self.wam_ssh = client
		self.wam_sftp = paramiko.SFTPClient.from_transport(client.get_transport())

	def start_recording(self, logging_dir):
		self.record_start_pub.publish(EmptyM())
		self.wam_bag_id = self.bag_manager.start_recording(logging_dir + self.wam_traj_name, [self.wam_jnt_topic])[0]
		self.recording = True

	def finish_recording(self):
		if not self.recording:
			raise NotRecordingException()
	
		rospy.loginfo("Stopping wam joint recording.")
		self.record_stop_pub.publish()
		try:
			self.bag_manager.stop_recording(self.wam_bag_id)
		except:
			raise BagManagerStopException()

	def move_wam_traj_onboard(self, bag_location):
		if self.wam_sftp == None:
			rospy.logerr("Cannot place wam trajectory on WAM because sftp connection is non-existant.")
	
		if not os.path.exists(bag_location):
			rospy.logerr("Cannot move bag file onto WAM: bag file " + bag_location + " does not exist.")
			raise Exception("bag troubles.")
		
		try:
			self.wam_sftp.put(bag_location, self.wam_traj_location, confirm=True)
		except Exception as e:
			rospy.logerr("Trouble moving wam trajectory onto WAM from host.")
			print e.what()

	def move_wam_traj_offboard(self, dest_path):
		if self.wam_sftp == None:
			rospy.logerr("Cannot pull off wam trajectory because sftp connection is non-xistant.")

		try:
			self.wam_sftp.get(self.wam_traj_location, dest_path)
		except NameError as e:
			rospy.logerr("Trajectory bag file does not exist.")
		except:
			rospy.logerr("Trouble moving wam trajectory onto host computer.")


	def handle_transport_object(self, cur_grasp_data):
		user_input = raw_input("Would you like to transport the object? (y/n): ")
		if user_input.lower().strip() == "y":
			transport_object(self.gravity_comp_srv)
			while True:
				user_input = raw_input("Did the grasp succeed? (y/n/u): ")
				user_input = user_input.lower().strip()
				if user_input == "y":
					cur_grasp_data.add_annotation("Grasp success.")
					break
				elif user_input == "n":
					cur_grasp_data.add_annotation("Grasp failure.")
					break
				elif user_input == "u":
					cur_grasp_data.add_annotation("Grasp evaluation unsuccessful.")
					break
				else:
					continue


	def transport_object(self):
		# Get a current pose
		sleep_period = rospy.Duration(.5)
		while cur_wam_pose == None:
			rospy.sleep(sleep_period)
			continue
		self.transport_wam_pose = self.cur_wam_pose

		try:
			sleep_period = rospy.Duration(1)
			command_wam_pose_move([0, 0, 0.3])
			rospy.sleep(sleep_period)
			command_wam_pose_move([0, 0.3, 0])
			rospy.sleep(sleep_period)
			command_wam_pose_move([0, 0, -0.1])
			command_wam_ortn_move(orient_wam)
		except PoseMoveException as e:
			print e
			try:
				self.gravity_comp_srv(True)	
			except:
				rospy.logerr("Could not reenable gravity compensation.")
				pass
			cur_wam_pose = None
			return False
		
		self.gravity_comp_srv(True)
		self.transport_wam_pose = None
		return True

	def wam_pose_cb(self, msg):
		global cur_wam_pose
		#rospy.loginfo("Got current wam pose.")
		self.cur_wam_pose = msg.pose

	def command_wam_pose_move(self, adjustments):
		xyz = self.transport_wam_pose
		xyz.x += adjustments[0]
		xyz.y += adjustments[1]
		xyz.z += adjustments[2]

		#try:
		res = self.move_wam([xyz.x, xyz.y, xyz.z])
		#except:
		#	rospy.logerr("Could not command wam to pose: " + str(pose))
		#	raise PoseMoveException(pose) 

	def command_wam_ortn_move(self):
		rospy.logwarn("command wam ortn move needs an exception wrapper")
		ros_quat = self.transport_wam_pose.orientation
		res = self.orient_wam([ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w])
	
	def move_wam_home(self):
		try:
			self.wam_home_srv(EmptyRequest())
		except:
			rospy.logerr("Can't move the WAM home...")

	def load_traj(self):
		self.wam_load_srv(self.wam_traj_location, self.wam_jnt_topic)

	def start_playback(self):
		self.playback_start_pub.publish(EmptyM())
