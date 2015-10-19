#! /usr/bin/env python
import rospy
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyM
from geometry_msgs.msg import PoseStamped
from wam_srvs.srv import HandCommandBlkRequest, HandCommandBlkResponse, HandCommandBlk, JointRecordPlayback, CartPosMove, CartPosMoveRequest, CartPosMoveResponse, GravityComp, GravityCompRequest, GravityCompResponse, PoseMove, PoseMoveRequest, PoseMoveResponse, OrtnMove, OrtnMoveRequest, OrtnMoveResponse
from wam_msgs.msg import HandCommand, StampedString
from kinect_monitor import *

import sys
import os
import datetime
import time
import paramiko

from bag_manager import BagManager
from shared_globals import *

cur_wam_pose = None
prelog_hand_pose = [0,0,0,0]
wam_jnt_topic = "/wam_grasp_capture/recording/joint_states"
wam_traj_name = "wam_traj.bag"
wam_traj_location = "/tmp/" + wam_traj_name
wam_sftp = None
wam_ssh = None

class PoseMoveException(Exception):
	def __init__(self, req_pos):
		self.req_pos = req_pos
	def __str__(self):
		return str(self.req_pos)


#TODO: Factor out the wam class into its own file

# Returns None or a number
def get_gui_int(gui, control_id, data_name):
	try:
		num = int(gui.get_text(control_id))
		return num
	except:
		gui.show_error(data_name + " must be a number")
	
	return None

# To be used when starting/completing a component of the workflow
def enable_gui_elements(gui, control_id_list):
	for e in control_id_list:
		gui.enable_element(e)

# The same as enable_gui_elements except it disables elements
def disable_gui_elements(gui, control_id_list):
	for e in control_id_list:
		gui.disable_element(e)


# Elements to be enabled when a workflow phase is started
#	and then disabled when that phase is complete
workflow_gui_elements = {'new_trial_elements':["_new_trial", "_good_bad", "_sub_num", "_obj_num", "_begin_trial"] }

class GraspCapture:
	def __init__(self, gui):
		self.gui = gui

		self.init_ros_interface()
		self.set_data_storage_path()

		verify_data_directories()
		self.gui = gui
		rospy.logerr("SKIPPING WAM CONNECTION. NEEDS TO BE UNDONE LATER.")
		#self.wam = WAM()
		self.bag_manager = BagManager()
		self.hand_logger = HandLogger(self.bag_manager)
		self.kinect_monitor = KinectMonitor(kinect_data_topics[1])


		wam_home_srv = rospy.ServiceProxy('/wam/go_home', Empty)
		record_start_pub = rospy.Publisher('/wam_grasp_capture/jnt_record_start', EmptyM, queue_size=1)
		record_stop_pub = rospy.Publisher('/wam_grasp_capture/jnt_record_stop', EmptyM, queue_size=1)
		playback_load_srv = rospy.ServiceProxy('/wam_grasp_capture/jnt_playback_load', JointRecordPlayback)
		playback_start_pub = rospy.Publisher('/wam_grasp_capture/jnt_playback_start', EmptyM, queue_size=1)
		playback_stop_pub = rospy.Publisher('/wam_grasp_capture/jnt_playback_stop', EmptyM, queue_size=1)
		hand_playback_start_pub = rospy.Publisher('start_hand_playback', String, queue_size=1)
		hand_playback_stop_pub = rospy.Publisher('stop_hand_playback', EmptyM, queue_size=1)
		hand_cmd_blk_srv = rospy.ServiceProxy("/bhand/hand_cmd_blocking", HandCommandBlk)
		
		gravity_comp_srv = rospy.ServiceProxy("/wam/gravity_comp", GravityComp)
		
		self.cur_grasp_data = None

		# Create the synchronization constructs
		self.robot_recording = False
		self.human_recording = False

	def init_ros_interface(self):
		rospy.init_node("grasp_capture")
		print "Grasp logging node online."
		
		sounder_pub = rospy.Publisher("/make_beep", EmptyM, queue_size=1)

	def set_data_storage_path(self):
		global grasp_info_dir, harddrive_path, local_data_path
		if os.path.exists(harddrive_path):
			grasp_info_dir = harddrive_path
			self.gui.show_error("Using harddrive for storage.")
			rospy.loginfo("Using harddrive.")
		else:
			grasp_info_dir = local_data_path
			self.gui.show_error("Not using harddrive for storage. Using local")
			rospy.logerr("Not using harddrive for storage. Using local")

#############################
###### Main Workflow ########
#############################
	def start_new_trial(self):
		# Ensure that the old trial has ended
		if self.cur_grasp_data != None:
			self.gui.show_error("Current grasp trial not complete. Not beginning new trial")
			return

		# Setup the next trial data structures
		self.cur_grasp_data = GraspData(self.gui, self.bag_manager)
		self.gui.show_info("Beginning new trial.")
		self.gui.disable_element("_new_trial")
		self.gui.register_button_cb("_begin_trial", self.validate_trial_and_begin)
	
	def validate_trial_and_begin(self):
		if not self.cur_grasp_data.make_data_dir():
			return

		self.hand_logger.set_log_dir(self.cur_grasp_data.get_log_dir())
		self.gui.disable_element("_begin_trial")
		enable_gui_elements(self.gui, ["_robot_phase_first", "_human_phase_first"])
		self.gui.register_button_cb("_robot_phase_first", self.robot_phase_first)
		self.gui.register_button_cb("_human_phase_first", self.human_phase_first)

	def robot_phase_first(self):
		self.gui.show_info('Beginning robot grasp phase.')
		disable_gui_elements(self.gui, ["_robot_phase_first", "_human_phase_first"])
		enable_gui_elements(self.gui, ["_end_phase"])
		self.gui.register_button_cb("_end_phase", self.cleanup_recording)
		
		self.human_kinect_bag_path = self.cur_grasp_data.get_log_dir() + "kinect_hand_capturel.bag"
		self.init_robot_phase()


	def human_phase_first(self):
		self.gui.show_info("Starting human phase.")
		disable_gui_elements(self.gui, ["_robot_phase_first", "_human_phase_first"])
		enable_gui_elements(self.gui, ["_end_phase"])
		self.gui.register_button_cb("_end_phase", self.cleanup_recording)
		
		self.human_kinect_bag_path = self.cur_grasp_data.get_log_dir() + "kinect_hand_capturef.bag"
		self.init_human_phase()

	def robot_phase_second(self):
		self.gui.show_info("Starting robot phase.")
		self.init_robot_phase()

	def human_phase_second(self):
		self.gui.show_info("Starting human phase.")

		self.init_human_phase()


	# Sets up the robot grasp capture phase:
	#	Saving data structures, kinect connection, etc...
	def init_robot_phase(self):
		rospy.loginfo("Beginning robot capture phase.")

		self.robot_recording = True

	def init_human_phase(self):
		global kinect_data_topics
		rospy.loginfo("Beginning human capture phase.")
		self.kinect_monitor.block_for_kinect()
		self.kinect_monitor.start_monitor()
		
		sounder_pub.publish(EmptyM())
		self.cur_grasp_data.start_human_grasp_annotations()
		kinect_bag_id = self.bag_manager.start_recording(self.human_kinect_bag_path, kinect_data_topics)[0]
		time.sleep(0.1)
		self.cur_grasp_data.add_annotation("Human grasp capture start.")

		self.human_recording = True

	def cleanup_recording(self):
		if self.robot_recording:
			# Stop the robot recording
			#TODO:Stop the robot recording
			self.gui.register_button_cb("_begin_next_phase", self.human_phase_second)
			self.gui.show_info("Completed cleaning up robot capture.")
			self.robot_recording = False

		elif self.human_recording:
			# Stop the human recording
			#TODO: Integrate the GUI with the annotations interface.
			#cur_grasp_data.add_annotations("Press [Enter] to complete kinect hand data recording.")
			self.cur_grasp_data.add_annotation("Human grasp capture end.")
			self.kinect_monitor.stop_monitor()
		
			self.bag_manager.stop_recording(kinect_bag_id)
			self.cur_grasp_data.stop_human_grasp_annotations()
			

			self.gui.register_button_cb("_begin_next_phase", self.robot_phase_second)
			self.gui.show_info("Completed cleaning up human capture.")
			self.human_recording = False
		else:
			rospy.logerr("No capture session going, but cleanup_recording() was called!")
			self.gui.show_error("No capture session going, but cleanup_recording() was called!")
			return

		self.gui.disable_element("_end_phase")
		self.gui.enable_element("_begin_next_phase")
			
# An interface to the WAM data collection and control mechanisms
class WAM:
	def __init__(self):
		init_wam_sftp()
		

class GraspData:
	def __init__(self, gui, bag_manager):
		self.gui = gui
		self.bag_manager = bag_manager
		self.initialized = False
		self.grasp_set_num = 0
		
		self.general_info = {}
		self.general_info['date'] = str(datetime.date.today())
		
		self.workflow_elements = ["_new_trial", "_good_bad", "_sub_num", "_obj_num", "_begin_trial"]
		enable_gui_elements(self.gui, self.workflow_elements)

		
	# Create the directory for this subject/object's grasp trial data
	# Returns: True on success and False on failure
	def make_data_dir(self):
		gb = self.gui.get_text("_good_bad") #raw_input("Is this grasp good or bad? (g/b): ")
		
		# Get the object number
		obj_num = get_gui_int(self.gui, "_obj_num", "object number")
		if obj_num == None or obj_num < 0:
			self.gui.show_error("object number must be greater than zero.")
			return False

		# Get the subject number
		sub_num = get_gui_int(self.gui, "_sub_num", "subject number")
		if sub_num == None or sub_num < 0:
			self.gui.show_error("subject number must be greater than zero.")
			return False

		# Form the directory name
		base_dir = "obj" + str(obj_num) + "_sub" + str(sub_num)
		if gb == "good":
			self.instance_dir = grasp_info_dir + "/good/" + base_dir
		else:
			self.instance_dir = grasp_info_dir + "/bad/" + base_dir 

		# Create the directory
		if os.path.exists(self.instance_dir):
			rospy.logwarn("Grasping directory already exists. Appending unique id.")
			suffix_num =  get_cur_grasp_num("/".join(self.instance_dir.split("/")[0:-1]), base_dir)

			self.instance_dir += "_" + suffix_num
		os.mkdir(self.instance_dir, 0755)

		print "Created current test's data directory: ", self.instance_dir
		self.gui.show_info("Created current test's data directory: " + self.instance_dir)
		self.finish_init()
		self.initialized = True
		return True

	def finish_init(self):
		self.info_file_path = self.instance_dir + "/" + "robot_grasp_annotations.bag"
		self.human_grasp_path = self.instance_dir + "/" + "human_grasp_annotations.bag"
		self.annotations_topic = "/grasp_annotations"
		self.annotations_pub = rospy.Publisher(self.annotations_topic, StampedString, queue_size=1)

		# Disable gui components
		self.gui.show_error("")
		disable_gui_elements(self.gui, self.workflow_elements)

	def start_robot_grasp_annotations(self):
		rospy.loginfo("Starting robot grasp annotations.")
		self.annotation_id = self.bag_manager.start_recording(self.info_file_path, [self.annotations_topic])[0]
	
	def start_human_grasp_annotations(self):
		rospy.loginfo("Starting human grasp_annotations.")
		self.human_grasp_annotation_id = self.bag_manager.start_recording(self.human_grasp_path, [self.annotations_topic])[0]

	def add_annotation(self, annotation_string):
		self.annotations_pub.publish(rospy.Time.now(), annotation_string)
		print "Added annotation: ", annotation_string

	def add_annotations(self, command_str):
		while True:
			cmd = raw_input(("\nAnnotation reference: \n\t"
			"n - new grasp set\n\t"
			"o - optimal grasp for grasp set\n\t"
			"e - range extreme marker\n\t"
			"r - rotational symmetry\n\t"
			"s - start natural task\n\t"
			"OR\n" + command_str + "\n"))

			cmd = cmd.lower().strip()
			if cmd == "n":
				self.grasp_set_num += 1
				self.add_annotation("Begin grasp set " + str(self.grasp_set_num))
			elif cmd == "o":
				self.add_annotation("Optimal grasp for grasp set " + str(self.grasp_set_num))
			elif cmd == "e":
				self.add_annotation("Grasp range extreme for grasp set: " + str(self.grasp_set_num))
			elif cmd == "r":
				self.add_annotation("There is rotational symmetry about this axis for grasp set: " + str(self.grasp_set_num))
			elif cmd == "s":
				self.add_annotation("Start of natural task.")
			elif cmd == "\n" or cmd == "":
				return
			else:
				self.add_annotation(cmd)

	def get_log_dir(self):
		return (self.instance_dir + '/')

	def close_info_file(self):
		ret = self.bag_manager.stop_recording(self.annotation_id)
		if ret[0]:
			print "Grasp info file closed."

	def stop_human_grasp_annotations(self):
		ret = self.bag_manager.stop_recording(self.human_grasp_annotation_id)
		if ret[0]:
			print "Human grasp annotations file closed."
		

# Performs a walk through the grasp info directory to find the 
#	highest value unused directory
# Preconditions: the grasp_dir must be a fully qualified path
# Returns: An index higher than 1. 1 is reserved for renaming fragmented
#	directories during post processing. See processing scripts
def get_cur_grasp_num(grasp_dir, base_name):
	files = os.listdir(grasp_dir)
	if files == []:
		return 0

	nums = []
	f_max = 2
	for f in files:
		pre_slash = "_".join(f.split("_")[0:-1])
		if pre_slash == base_name:
			try:
				f_num = int(f.split("_")[-1])
				if f_num >= f_max:
					f_max = f_num + 1
			except:
				pass
	return str(f_max)

def grasp_hand():
	rospy.logdebug("Waiting for hand close service.")
	rospy.wait_for_service("/bhand/close_grasp")
	try:
		close_hand = rospy.ServiceProxy('/bhand/close_grasp', Empty)
		resp = close_hand()
	except:
		rospy.logerr("Closing barrett hand failed.")


class HandLogger:
	def __init__(self, bag_manager):
		self.bag_manager = bag_manager
		self.bag_name = "hand_commands.bag"

		self.hand_cmd_pub = rospy.Publisher("/bhand/hand_cmd", HandCommand,  queue_size=1)

		self.log_dir = None
		self.bag_id = None

	def set_log_dir(self, log_dir):
		if log_dir[-1] != "/":
			rospy.logerr("Invalid logging directory for hand data. Recorder expects a trailing '/'")
			return

		self.log_dir = log_dir

	# Preconditions: The hand has been sent to its home position
	def start_hand_capture(self):
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

# Resets the hand and WAM arm
def setup_playback(hand_cmd_blk_srv, wam_load_srv, bag_file):
	global wam_traj_location
	setup_hand(hand_cmd_blk_srv)

	move_wam_traj_onboard(bag_file)
	wam_load_srv(wam_traj_location, wam_jnt_topic)

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

def move_wam_traj_onboard(bag_location):
	global wam_sftp, wam_traj_location
	if wam_sftp == None:
		rospy.logerr("Cannot place wam trajectory on WAM because sftp connection is non-existant.")

	if not os.path.exists(bag_location):
		rospy.logerr("Cannot move bag file onto WAM: bag file " + bag_location + " does not exist.")
		raise Exception("bag troubles.")
	
	try:
		wam_sftp.put(bag_location, wam_traj_location, confirm=True)
	except Exception as e:
		rospy.logerr("Trouble moving wam trajectory onto WAM from host.")
		print e.what()

def move_wam_traj_offboard(dest_path):
	global wam_sftp, wam_traj_location
	if wam_sftp == None:
		rospy.logerr("Cannot pull off wam trajectory because sftp connection is non-xistant.")

	try:
		wam_sftp.get(wam_traj_location, dest_path)
	except NameError as e:
		rospy.logerr("Trajectory bag file does not exist.")
	except:
		rospy.logerr("Trouble moving wam trajectory onto host computer.")


def handle_transport_object(cur_grasp_data, gravity_comp_srv):
	user_input = raw_input("Would you like to transport the object? (y/n): ")
	if user_input.lower().strip() == "y":
		transport_object(gravity_comp_srv)
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


def transport_object(gravity_comp_srv):
	global cur_wam_pose
	wam_pos_sub = rospy.Subscriber('/wam/pose', PoseStamped, wam_pose_cb)
	while cur_wam_pose == None:
		continue
	
	wam_pos_sub.unregister()
	rospy.wait_for_service("/wam/cart_move")
	move_wam = rospy.ServiceProxy("/wam/cart_move", CartPosMove)
	orient_wam = rospy.ServiceProxy("/wam/ortn_move", OrtnMove)
	
	try:
		command_wam_pose_move(move_wam, cur_wam_pose.position, [0, 0, 0.3])
		time.sleep(1)
		command_wam_pose_move(move_wam, cur_wam_pose.position, [0, 0.3, 0])
		time.sleep(1)
		command_wam_pose_move(move_wam, cur_wam_pose.position, [0, 0, -0.1])
		command_wam_ortn_move(orient_wam, cur_wam_pose.orientation)
	except PoseMoveException as e:
		print e
		try:
			gravity_comp_srv(True)	
		except:
			rospy.logerr("Could not reenable gravity compensation.")
			pass
		cur_wam_pose = None
		return False
	
	gravity_comp_srv(True)
	cur_wam_pose = None
	return True


def wam_pose_cb(msg):
	global cur_wam_pose
	rospy.loginfo("Got current wam pose.")
	cur_wam_pose = msg.pose

def command_wam_pose_move(move_wam, xyz, adjustments):
	xyz.x += adjustments[0]
	xyz.y += adjustments[1]
	xyz.z += adjustments[2]

	#try:
	res = move_wam([xyz.x, xyz.y, xyz.z])
	#except:
	#	rospy.logerr("Could not command wam to pose: " + str(pose))
	#	raise PoseMoveException(pose) 

def command_wam_ortn_move(orient_wam, ros_quat):
	rospy.logwarn("command wam ortn move needs an exception wrapper")
	res = orient_wam([ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w])

def verify_data_directories():
	global grasp_info_dir
	rospy.loginfo("Verifying grasp data directory existance.")

	dir_path = grasp_info_dir + "/good"
	if not os.path.exists(dir_path):
		rospy.loginfo("Data directory 'good' doesnt exist. Creating.")
		try:
			os.makedirs(dir_path)
		except OSError:
			rospy.logerr("Cannot create grasping directory... Is the harddrive connected?")

	dir_path = grasp_info_dir + "/bad"
	if not os.path.exists(dir_path):
		rospy.loginfo("Data directory 'bad' doesnt exist. Creating.")
		os.makedirs(dir_path)
		
def init_wam_sftp():
	global wam_sftp, wam_ssh
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

	wam_ssh = client
	wam_sftp = paramiko.SFTPClient.from_transport(client.get_transport())


#def kinect_hand_capture(sounder_pub, cur_grasp_data, hand_first, kinect_monitor, bag_manager):

def commence_playback(hand_cmd_blk_srv, playback_load_srv, playback_start_pub, hand_playback_start_pub, hand_logger, grasp_dir):
	global wam_traj_name
	user_input = raw_input("Playback? (y/n): ")
	if user_input.lower().strip() == "y":
		print "Beginning playback."
		bag_file = grasp_dir + wam_traj_name
		try:
			setup_playback(hand_cmd_blk_srv, playback_load_srv, bag_file)
			playback_start_pub.publish(EmptyM())
			hand_playback_start_pub.publish(grasp_dir + hand_logger.bag_name)
			raw_input("Press [Enter] when playback is complete.")
		except:
			rospy.logerr("Playback aborted.")
			pass	

def move_wam_home(wam_home_srv):
	try:
		wam_home_srv(EmptyRequest())
	except:
		rospy.logerr("Can't move the WAM home...")



#if __name__ == "__main__":
#		# Query the order of human hand motion capture and robot motion capture
#		researcher_input = raw_input("Hand motion capture before robot motion capture?(y/n)")
#		if researcher_input.strip().lower() == "y":
#			kinect_hand_cap_before_mocap = True
#			kinect_hand_capture(sounder_pub, cur_grasp_data, kinect_hand_cap_before_mocap, kinect_monitor, bag_manager)
#			raw_input("End the eye tracking! Then press [Enter]")
#		else:
#			kinect_hand_cap_before_mocap = False
#
#
#		# Begin motion capture
#		raw_input("Press [Enter] to start the motion capture")
#		cur_grasp_data.start_robot_grasp_annotations()
#		kinect_monitor.block_for_kinect()
#		kinect_monitor.start_monitor()
#		cur_grasp_data.add_annotation("Motion Capture Start")
#		sounder_pub.publish()
#		setup_hand(hand_cmd_blk_srv)
#		hand_logger.start_hand_capture();
#		record_start_pub.publish(EmptyM())
#		wam_bag_id = bag_manager.start_recording(cur_grasp_data.get_log_dir() + wam_traj_name, [wam_jnt_topic])[0]
#		kinect_bag_id = bag_manager.start_recording(cur_grasp_data.get_log_dir() + "kinect_robot_capture.bag", kinect_data_topics)[0]
#
#		# End motion capture
#		cur_grasp_data.add_annotations("Press [Enter] to end the motion capture")
#		cur_grasp_data.add_annotation("Motion Capture End")
#		kinect_monitor.stop_monitor()
#		record_stop_pub.publish()
#		hand_logger.end_hand_capture();
#		try:
#			bag_manager.stop_recording(kinect_bag_id)
#			bag_manager.stop_recording(wam_bag_id)
#		except:
#			rospy.logerr("Trouble closing the kinect data file at end of motion capture.")
#	
#		if not kinect_hand_cap_before_mocap:
#			raw_input("End the eye tracking! Then press [Enter]")
#
#		# Move the hand to the ledge
#		#handle_transport_object(cur_grasp_data, gravity_comp_srv)
#		
#		# Playback trial
#		#commence_playback(hand_cmd_blk_srv, playback_load_srv, playback_start_pub, hand_playback_start_pub, hand_logger, cur_grasp_data.get_log_dir())
#
#		# Save grasping data
#		cur_grasp_data.add_annotation("Bag file closing. End final grasp set.")
#		cur_grasp_data.close_info_file()
#
#		# Move home
#		move_wam_home(wam_home_srv)
#
#		# If we didn't capture the hand before, do it now
#		if not kinect_hand_cap_before_mocap:
#			raw_input("Press [Enter] to begin kinect capture for human grasps.")
#			kinect_hand_capture(sounder_pub, cur_grasp_data, kinect_hand_cap_before_mocap, kinect_monitor, bag_manager)
#
#		repeat_input = raw_input("Would you like to run another test? (y/n)")
#		if repeat_input.lower().strip() != "y":
#			break
#	
#	kinect_monitor.kill_monitor()
##	rospy.loginfo("Grasp testing complete =)")
