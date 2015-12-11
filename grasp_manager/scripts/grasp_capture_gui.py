#! /usr/bin/python
import grasp_capture
import pyforms
from   pyforms          import BaseWidget
from   pyforms.Controls import ControlText
from   pyforms.Controls import ControlButton
from   pyforms.Controls import ControlCombo

class App(BaseWidget):
	def __init__(self):
		# Firstly, create a dummy gui. Elements are present, but nothing is connected
		super(App, self).__init__('Hide Test')
		self._info_box = ControlText("Info:")
		self._error_box = ControlText("Errors:")
		self.add_start_trial_gui_components()
		self.add_order_choice_elements()
		self.add_capture_elements()

		self._formset = ["_info_box", "_error_box", ("_new_trial", "_good_bad", "_obj_num", "_sub_num", "_begin_trial"), ("_robot_phase_first", "_human_phase_first"), ("_end_phase", "_begin_next_phase"), ("_new_grasp", "_optimal_grasp", "_extreme_grasp"), ("_start_natural", "_rotation_symm")]

		# Create the control structures and hook them up
		self.grasp_capture = grasp_capture.GraspCapture(self)
		self._new_trial.value = self.grasp_capture.start_new_trial

	def add_start_trial_gui_components(self):
		self._new_trial = ControlButton('New Trial')
		self._begin_trial = ControlButton('Begin Trial')
		self.disable_element('_begin_trial')
		
		self._good_bad = ControlCombo("Good/Bad")
		self._good_bad.addItem("Good", "good")
		self._good_bad.addItem("Bad", "bad")
		self.disable_element('_good_bad')

		self._sub_num = ControlText('Subject Num')
		self._obj_num = ControlText('Object Num')
		self.disable_element('_sub_num')
		self.disable_element('_obj_num')

	def add_order_choice_elements(self):
		self._robot_phase_first = ControlButton('Robot Grasps First')
		self._human_phase_first = ControlButton('Human Grasps First')
		self.disable_element('_robot_phase_first')
		self.disable_element('_human_phase_first')

	def add_capture_elements(self):
		self._end_phase = ControlButton("End Capture")
		self._begin_next_phase = ControlButton("Begin Next Phase")
		self.disable_element("_end_phase")
		self.disable_element("_begin_next_phase")

		self._new_grasp = ControlButton("New Grasp")
		self._extreme_grasp = ControlButton("Add Extreme Grasp")
		self._optimal_grasp = ControlButton("Add Optimal Grasp")
		self._rotation_symm = ControlButton("Add Rotational Symmetry")
		self._start_natural = ControlButton("Start Natural Task")

		self.disable_element("_new_grasp")
		self.disable_element("_extreme_grasp")
		self.disable_element("_optimal_grasp")
		self.disable_element("_rotation_symm")
		self.disable_element("_start_natural")

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
		try:
			exec("self." + button_id + "._form.pushButton.disconnect()")
			exec("self." + button_id + ".value = callback")
		except:
			self.show_error("DEVELOPER MSG: cannot register button " + button_id + ".")

	def show_info(self, msg):
		self._info_box.value = msg

	def show_error(self, error_msg):
		self._error_box.value = error_msg

if __name__ == "__main__":
	pyforms.startApp(App)
