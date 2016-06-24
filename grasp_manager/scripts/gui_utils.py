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
workflow_gui_elements = {'new_trial_elements':["_good_bad", "_sub_num", "_obj_num", "_begin_trial"] }


