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
from grasp_manager.shared_globals import *
from wam import *
from gui_utils import *
from hand_logger import *

# Colons denote that a value will follow. There will be no spaces.
grasp_capture_annotation_messages = {"new_grasp":"Begin grasp set:", "extreme":["Grasp range extreme:", ":for grasp set:"], "optimal":"Optimal grasp for grasp set:", "rotation":"There is rotational symmetry about this axis for grasp set:", "natural":"Start of natural task."}

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
		self.kinect = KinectMonitor(self, self.gui)
		self.cur_grasp_data = GraspData(self.gui, self.bag_manager)
		
		# Complete with the general initialization
		self.connect_to_gui()
		self.reinit()

	def reinit(self):
		# Create the synchronization constructs
		self.current_phase = 0
		self.human_kinect_bag_path = ""
		self.kinect_bag_id = None
		self.robot_recording = False
		self.completed_robot = False
		self.human_recording = False
		self.completed_human = False

	def init_ros_interface(self):
		rospy.init_node("grasp_capture")
		print "Grasp logging node online."
		
		self.sounder_pub = rospy.Publisher("/make_beep", EmptyM, queue_size=1)

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
############################
###### GUI Init. ###########
############################
	def connect_to_gui(self):
		self.gui.register_button_cb("_begin_trial", self.validate_trial_and_begin)
		self.gui.register_button_cb("_robot_phase_first", self.robot_phase_first)
		self.gui.register_button_cb("_human_phase_first", self.human_phase_first)
		self.gui.register_button_cb("_end_phase", self.cleanup_recording)
		self.gui.register_button_cb("_begin_next_phase", self.begin_second_phase)


#############################
###### Main Workflow ########
#############################
	def validate_trial_and_begin(self):
                # Ensure that the old trial has ended
		if self.cur_grasp_data.initialized:
			self.gui.show_error("Current grasp trial not complete. Not beginning new trial")
			return

		# Setup the next trial data structures
		self.current_phase = 0
		self.gui.show_info("Beginning new trial.")

		if not self.cur_grasp_data.make_data_dir():
			return

		self.hand_logger.set_log_dir(self.cur_grasp_data.get_log_dir())
		self.gui.disable_element("_begin_trial")
		enable_gui_elements(self.gui, ["_robot_phase_first", "_human_phase_first"])

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

	def begin_second_phase(self):
		if self.completed_human:
			self.robot_phase_second()
		elif self.completed_robot:
			self.human_phase_second()
		else:
			rospy.logerr("No phases have been completed and begin_second_phase() has been called.")
			self.gui.show_error("Developer MSG: Work being completed out of order!")

	def robot_phase_second(self):
		self.gui.disable_element("_begin_next_phase")
		self.gui.show_info("Starting robot phase.")
		self.init_robot_phase()
		self.gui.enable_element("_end_phase")

	def human_phase_second(self):
		self.gui.disable_element("_begin_next_phase")
		self.gui.show_info("Starting human phase.")
		self.init_human_phase()
		self.gui.enable_element("_end_phase")

	# Sets up the robot grasp capture phase:
	#	Saving data structures, kinect connection, etc...
	def init_robot_phase(self):
		global kinect_data_topics
		rospy.loginfo("Beginning robot capture phase.")
		self.cur_grasp_data.start_robot_grasp_annotations()
		self.kinect.resume_monitors()
		
		self.cur_grasp_data.add_annotation("Motion Capture Start")	
		logging_dir = self.cur_grasp_data.get_log_dir()
		self.sounder_pub.publish()
		self.hand_logger.start_hand_capture();
		self.wam.start_recording(logging_dir)
		self.kinect_bag_id = self.bag_manager.start_recording(logging_dir + "kinect_robot_capture.bag", kinect_data_topics)[0]
		
		self.robot_recording = True

	def init_human_phase(self):
		global kinect_data_topics
		rospy.loginfo("Beginning human capture phase.")
		self.kinect.resume_monitors()
		
		self.sounder_pub.publish(EmptyM())
		self.cur_grasp_data.start_human_grasp_annotations()
		self.kinect_bag_id = self.bag_manager.start_recording(self.human_kinect_bag_path, kinect_data_topics)[0]
		time.sleep(0.1)
		self.cur_grasp_data.add_annotation("Human grasp capture start.")

		self.human_recording = True

	def cleanup_recording(self):
		self.gui.disable_element("_end_phase")
		print "CLEANUP CALLED"
		if self.current_phase > 1:
			self.gui.show_error("Developer MSG: user has been allowed more phases than regular.")
			return

		if self.robot_recording:
			# Stop the robot recording
			#TODO: Integrate the GUI with the annotations interface
			self.cur_grasp_data.add_annotation("Motion Capture End")
			self.kinect.pause_monitors()
			self.hand_logger.end_hand_capture();
			self.cur_grasp_data.stop_grasp_annotations()
			
			try:
				ret = self.wam.finish_recording()
			except BagManagerStopException as e:
				rospy.logerr(str(e))
				self.gui.show_error(str(e))
			except NotRecordingException as e:
				rospy.logerr(str(e))
				self.gui.show_error(str(e))
			
			try:
				self.bag_manager.stop_recording(self.kinect_bag_id)
			except:
				rospy.logerr("Trouble closing the kinect data file at end of motion capture.")
		
			self.gui.show_info("Completed cleaning up robot capture.")
			self.robot_recording = False
			self.completed_robot = True

		elif self.human_recording:
			# Stop the human recording
			#TODO: Integrate the GUI with the annotations interface.
			self.cur_grasp_data.add_annotation("Human grasp capture end.")
			self.kinect.pause_monitors()
		
			self.bag_manager.stop_recording(self.kinect_bag_id)
			self.cur_grasp_data.stop_grasp_annotations()
			

			self.gui.show_info("Completed cleaning up human capture.")
			print "Setting human recording to false."
			self.human_recording = False
			self.completed_human = True
		else:
			rospy.logerr("No capture session going, but cleanup_recording() was called!")
			self.gui.show_error("No capture session going, but cleanup_recording() was called!")
			return
		
		self.current_phase += 1

		if self.current_phase == 1:
			self.gui.enable_element("_begin_next_phase")
		else:
			self.cleanup_second_phase()
			
	def cleanup_second_phase(self):
		self.gui.disable_element("_begin_next_phase")
		
		# Move home
		self.gui.show_info("Moving WAM home.")
		self.wam.move_wam_home()

		# Get ready for another trial
		self.gui.show_error("")
		self.gui.show_info("Trial complete. Press Begin Trial to begin another trial.")
		self.cur_grasp_data.reinit()
		self.reinit()
		self.gui.enable_element("_begin_trial")

	def cleanup_process(self):
		self.kinect.kill_monitors()
		rospy.loginfo("Completed grasp testing =).")

class GraspData:
	def __init__(self, gui, bag_manager):
		global grasp_capture_annotation_messages
		self.gui = gui
		self.bag_manager = bag_manager
		self.grasp_capture_annotation_messages = grasp_capture_annotation_messages
		self.workflow_elements = ["_good_bad", "_sub_num", "_obj_num", "_begin_trial"]
		self.annotation_elements = ["_new_grasp", "_extreme_grasp", "_optimal_grasp", "_rotation_symm", "_start_natural"]
		self.starting_annotation_elements = ["_new_grasp", "_start_natural"]
		self.annotations_topic = "/grasp_annotations"
		self.annotations_pub = rospy.Publisher(self.annotations_topic, StampedString, queue_size=1)
		
		self.connect_to_gui()
		self.common_init()

	def reinit(self):
		self.common_init()

	def connect_to_gui(self):
		self.gui.register_button_cb("_new_grasp", self.add_new_grasp)
		self.gui.register_button_cb("_optimal_grasp", self.specify_optimal_grasp)
		self.gui.register_button_cb("_extreme_grasp", self.specify_extreme_grasp)
		self.gui.register_button_cb("_rotation_symm", self.specify_rotational_symmetry)
		self.gui.register_button_cb("_start_natural", self.start_natural_task)

	def common_init(self):
		self.instance_dir = ""
		self.robot_annotation_path = ""
		self.human_annotation_path = ""
		self.annotation_id = -1
		self.initialized = False
		self.grasp_complete = True	# If a grasp hasn't started, it is trivially complete
		self.grasp_set_num = 0
		self.grasp_extreme_num = 0

		self.general_info = {}
		self.general_info['date'] = str(datetime.date.today())
		
		enable_gui_elements(self.gui, self.workflow_elements)
		
	# Create the directory for this subject/object's grasp trial data
	# Returns: True on success and False on failure
	def make_data_dir(self):
		gb = self.gui.get_text("_good_bad")
		
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
                        self.gui.show_error("Grasping directory already exists. Appending unique ID (assuming continuation of a previous trial). Continue if correct.")
			suffix_num =  get_cur_grasp_num("/".join(self.instance_dir.split("/")[0:-1]), base_dir)

			self.instance_dir += "_" + suffix_num
		os.mkdir(self.instance_dir, 0755)

		print "Created current test's data directory: ", self.instance_dir
		self.gui.show_info("Created current test's data directory: " + self.instance_dir)
		self.finish_init()
		self.initialized = True
		return True

	def finish_init(self):
		self.robot_annotation_path = self.instance_dir + "/" + "robot_grasp_annotations.bag"
		self.human_annotation_path = self.instance_dir + "/" + "human_grasp_annotations.bag"

		# Disable gui components
		self.gui.show_error("")
		disable_gui_elements(self.gui, self.workflow_elements)

	def start_robot_grasp_annotations(self):
		rospy.loginfo("Starting robot grasp annotations.")
		self.annotation_id = self.bag_manager.start_recording(self.robot_annotation_path, [self.annotations_topic])[0]
		self.enable_annotations()
	
	def start_human_grasp_annotations(self):
		rospy.loginfo("Starting human grasp_annotations.")
		self.annotation_id = self.bag_manager.start_recording(self.human_annotation_path, [self.annotations_topic])[0]
		self.enable_annotations()

	def enable_annotations(self):
		enable_gui_elements(self.gui, self.starting_annotation_elements)

	def add_annotation(self, annotation_string):
		self.annotations_pub.publish(rospy.Time.now(), annotation_string)
		print "Added annotation: ", annotation_string

	def add_new_grasp(self):
		self.gui.disable_element("_new_grasp")
		self.gui.disable_element("_rotation_symm")
                self.gui.disable_element("_start_natural")
		self.grasp_complete = False
		self.grasp_set_num += 1
		self.add_annotation(self.grasp_capture_annotation_messages['new_grasp'] + str(self.grasp_set_num))
		self.gui.show_info("Beginning grasp set " + str(self.grasp_set_num))
		self.gui.enable_element("_optimal_grasp")
                self.gui.enable_element("_extreme_grasp")
	
	def specify_optimal_grasp(self):
		self.gui.disable_element("_optimal_grasp")
                self.grasp_complete = True
		self.add_annotation(self.grasp_capture_annotation_messages['optimal'] + str(self.grasp_set_num))
		self.gui.show_info("Optimal grasp recorded.")
		self.gui.enable_element("_new_grasp")
                self.gui.enable_element("_rotation_symm")
                self.gui.enable_element("_start_natural")
	
	def specify_extreme_grasp(self):
		self.grasp_extreme_num += 1
		self.add_annotation(self.grasp_capture_annotation_messages['extreme'][0] + str(self.grasp_extreme_num) + self.grasp_capture_annotation_messages['extreme'][1] + str(self.grasp_set_num))
		self.gui.show_info("Added extreme grasp " + str(self.grasp_extreme_num) + " for set " + str(self.grasp_set_num))

	def specify_rotational_symmetry(self):
		self.add_annotation(self.grasp_capture_annotation_messages['rotation'] + str(self.grasp_set_num))
		self.gui.show_info("Added rotational symmetry for grasp " + str(self.grasp_set_num))
                self.gui.disable_element("_rotation_symm")

	def start_natural_task(self):
		if not self.grasp_complete:
			self.gui.show_info("Complete or delete this grasp before continuing to the natural task.")
			return

		self.gui.disable_element("_start_natural")
		self.add_annotation(self.grasp_capture_annotation_messages['natural'])
		self.gui.show_info(self.grasp_capture_annotation_messages['natural'])
		self.gui.disable_element

	def get_log_dir(self):
		return (self.instance_dir + '/')

	def stop_grasp_annotations(self):
		disable_gui_elements(self.gui, self.annotation_elements)
		self.add_annotation("Bag file closing. End final grasp set.")
		time.sleep(0.1)
		ret = self.bag_manager.stop_recording(self.annotation_id)
		if ret[0]:
			rospy.loginfo("Grasp annotations file closed.")
		

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
