#! /usr/bin/env python
import rospy
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyM
from wam_msgs.msg import HandCommand, StampedString
from kinect_monitor import *

import sys
import os
import datetime
import time

from bag_manager import BagManager
from shared_globals import *
from wam import *
from gui_utils import *
from hand_logger import *


class GraspCapture:
	def __init__(self, gui):
		self.gui = gui

		self.init_ros_interface()
		self.set_data_storage_path()

		verify_data_directories()
		self.gui = gui
		self.bag_manager = BagManager()
		self.hand_logger = HandLogger(self.bag_manager)
		self.wam 	 = WAM(self.bag_manager)
		self.kinect_monitor = KinectMonitor(kinect_data_topics[1])

		# Create the synchronization constructs
		self.cur_grasp_data = None
		self.human_kinect_bag_path = ""
		self.kinect_bag_id = None
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
		self.recording_gui_update()
		
		self.human_kinect_bag_path = self.cur_grasp_data.get_log_dir() + "kinect_hand_capturel.bag"
		self.init_robot_phase()


	def human_phase_first(self):
		self.gui.show_info("Starting human phase.")
		self.recording_gui_update()
		
		self.human_kinect_bag_path = self.cur_grasp_data.get_log_dir() + "kinect_hand_capturef.bag"
		self.init_human_phase()

	def recording_gui_update(self):
		disable_gui_elements(self.gui, ["_robot_phase_first", "_human_phase_first"])
		enable_gui_elements(self.gui, ["_end_phase"])
		self.gui.register_button_cb("_end_phase", self.cleanup_recording)

	def robot_phase_second(self):
		self.gui.show_info("Starting robot phase.")
		self.init_robot_phase()
		self.gui.enable_element("_end_phase")
		self.gui.register_button_cb("_end_phase", self.cleanup_second_phase)

	def human_phase_second(self):
		self.gui.show_info("Starting human phase.")
		self.init_human_phase()
		self.gui.enable_element("_end_phase")
		self.gui.register_button_cb("_end_phase", self.cleanup_second_phase)

	# Sets up the robot grasp capture phase:
	#	Saving data structures, kinect connection, etc...
	def init_robot_phase(self):
		global kinect_data_topics
		rospy.loginfo("Beginning robot capture phase.")
		self.cur_grasp_data.start_robot_grasp_annotations()
		kinect_monitor.block_for_kinect()
		kinect_monitor.start_monitor()
		
		cur_grasp_data.add_annotation("Motion Capture Start")	
		logging_dir = cur_grasp_data.get_log_dir()
		sounder_pub.publish()
		self.hand_logger.start_hand_capture();
		self.wam.start_recording(logging_dir)
		self.kinect_bag_id = self.bag_manager.start_recording(logging_dir + "kinect_robot_capture.bag", kinect_data_topics)[0]
		
		self.robot_recording = True

	def init_human_phase(self):
		global kinect_data_topics
		rospy.loginfo("Beginning human capture phase.")
		self.kinect_monitor.block_for_kinect()
		self.kinect_monitor.start_monitor()
		
		sounder_pub.publish(EmptyM())
		self.cur_grasp_data.start_human_grasp_annotations()
		self.kinect_bag_id = self.bag_manager.start_recording(self.human_kinect_bag_path, kinect_data_topics)[0]
		time.sleep(0.1)
		self.cur_grasp_data.add_annotation("Human grasp capture start.")

		self.human_recording = True

	def cleanup_recording(self):
		if self.robot_recording:
			# Stop the robot recording
			#TODO: Integrate the GUI with the annotations interface
			self.cur_grasp_data.add_annotation("Motion Capture End")
			self.kinect_monitor.stop_monitor()
			self.wam.finish_recording()
			self.hand_logger.end_hand_capture();
			try:
				self.bag_manager.stop_recording(self.kinect_bag_id)
			except:
				rospy.logerr("Trouble closing the kinect data file at end of motion capture.")
		
			self.gui.register_button_cb("_begin_next_phase", self.human_phase_second)
			self.gui.show_info("Completed cleaning up robot capture.")
			self.robot_recording = False

		elif self.human_recording:
			# Stop the human recording
			#TODO: Integrate the GUI with the annotations interface.
			self.cur_grasp_data.add_annotation("Human grasp capture end.")
			self.kinect_monitor.stop_monitor()
		
			self.bag_manager.stop_recording(self.kinect_bag_id)
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
			
	def cleanup_second_phase(self):
		self.cleanup_recording()
		self.gui.disable_element("_begin_next_phase")
		
		# Save grasping data
		self.cur_grasp_data.add_annotation("Bag file closing. End final grasp set.")
		self.cur_grasp_data.close_info_file()

		# Move home
		self.gui.show_info("Moving WAM home.")
		self.wam.move_wam_home(wam_home_srv)

		# Get ready for another trial
		self.gui.show_info("Trial complete. Press New Trial to begin another trial.")
		self.cur_grasp_data = None
		self.gui.enable_element("_new_trial")

	def cleanup_process(self):
		kinect_monitor.kill_monitor()
		rospy.loginfo("Completed grasp testing =).")

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
