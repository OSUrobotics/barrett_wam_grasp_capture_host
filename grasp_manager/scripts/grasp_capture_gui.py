#! /usr/bin/python
import rospy
import grasp_capture
from threading import Lock
import pyforms
from   pyforms          import BaseWidget
from   pyforms.Controls import ControlText
from   pyforms.Controls import ControlButton
from   pyforms.Controls import ControlCombo

class App(BaseWidget):
	def __init__(self):
		# Firstly, create a dummy gui. Elements are present, but nothing is connected
		super(App, self).__init__('Hide Test')
                self.button_list = []
	    	self.button_state_list = []
	    	self.button_state_list_lock = Lock()
		self._info_box = ControlText("Info:")
		self._error_box = ControlText("Errors:")
		self.add_start_trial_gui_components()
		self.add_order_choice_elements()
		self.add_capture_elements()

		self._formset = ["_info_box", "_error_box", ("_good_bad", "_obj_num", "_sub_num", "_begin_trial"), ("_robot_phase_first", "_human_phase_first"), "_kinect_reset", ("_end_phase", "_begin_next_phase"), ("_new_grasp", "_optimal_grasp", "_extreme_grasp"), ("_start_natural", "_rotation_symm")]

		# Create the control structures and hook them up
		self.grasp_capture = grasp_capture.GraspCapture(self)

	def add_start_trial_gui_components(self):
		self.add_button("_begin_trial", "Begin Trial")
		
		self._good_bad = ControlCombo("Good/Bad")
		self._good_bad.addItem("Good", "good")
		self._good_bad.addItem("Bad", "bad")
		self.disable_element('_good_bad')

		self._sub_num = ControlText('Subject Num')
		self._obj_num = ControlText('Object Num')
		self.disable_element('_sub_num')
		self.disable_element('_obj_num')

	def add_order_choice_elements(self):
                self.add_button("_robot_phase_first", "Robot Grasps First")
                self.add_button("_human_phase_first", "Human Grasps First")

	def add_capture_elements(self):
                self.add_button("_kinect_reset", "Kinect was Reset")
                self.add_button("_end_phase", "End Capture")
                self.add_button('_begin_next_phase', "Begin Next Phase")

                self.add_button("_new_grasp", "New Grasp")
                self.add_button("_extreme_grasp", "Add Extreme Grasp")
                self.add_button("_optimal_grasp", "Add Optimal Grasp")
                self.add_button("_rotation_symm", "Add Rotational Symmetry")
                self.add_button("_start_natural", "Start Natural Task")
                
###########################
### Application GUI API ###
###########################
	# May return an empty string on error
	def get_text(self, text_id):
		try:
			return eval("self." + text_id + ".value")
		except:
			self.show_error("DEVELOPER MSG: trying to get text from nonexistant text box " + text_id)
		return ""

	def disable_element(self, control_id):
		#try:
		exec("self." + control_id + ".enabled = False")
		#except:
		#	self.show_error("DEVELOPER MSG: trying to disable element " + control_id + " doesn't exist.")

	def enable_element(self, control_id):
		try:
			exec("self." + control_id + ".enabled = True")
		except:
			self.show_error("DEVELOPER MSG: trying to enable element " + control_id + " doesn't exist.")

	def is_enabled(self, control_id):
		try:
			return eval("self." + control_id + ".enabled")
		except:
			self.show_error("DEVELOPER MSG: control element does not exist. Cannot check enabled status.")
			return False

	def register_button_cb(self, button_id, callback):
		#try:
			exec("self." + button_id + ".value = callback")
		#except:
		#	self.show_error("DEVELOPER MSG: cannot register button " + button_id + ".")

	def show_info(self, msg):
		self._info_box.value = msg

	def show_error(self, error_msg):
		self._error_box.value = error_msg

        def add_button(self, name, text):
            exec("self." + name + " = ControlButton('" + text +"')")
            self.disable_element(name)
            self.button_list.append(name)
            #print "button_list: ", self.button_list
        
	def pickle_interface(self):
	    self.button_state_list_lock.acquire()
	    if len(self.button_state_list) != 0:
		    rospy.logerr("Grasp capture interface already pickled. Skipping.")
	    else:
            	rospy.loginfo("Pickling grasp capture interface.")
	    	for b in self.button_list:
                        print "b:", b
                        exec("self.button_state_list.append(('" + b + "', self." + b + ".enabled))")
	                self.disable_element(b)
            self.button_state_list_lock.release()

        def unpickle_interface(self):
	    self.button_state_list_lock.acquire()
	    if len(self.button_state_list) == 0:
		    rospy.logerr("Grasp capture interface not pickled.")
		    return
	    else:
            	rospy.loginfo("Unpickling grasp capture interface.")
	    	for b in self.button_state_list:
                    if b[1]:
                        self.enable_element(b[0])
	    	self.button_state_list = []
	    self.button_state_list_lock.release()

if __name__ == "__main__":
	pyforms.startApp(App)
