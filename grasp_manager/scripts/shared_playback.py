from shared_globals import *

extreme_bag_name = "grasp_extreme_snapshots.bag"
grasp_extreme_topic = "/grasp_snapshots"
grasp_data_directory = grasp_info_dir

def get_data_dir():
	obj = raw_input("Please input an object number: ")
	sub = raw_input("Please input a subject number: ")
	
	data_dir_name = "obj" + obj + "_sub" + sub
	data_dir_path = grasp_data_directory + "/good/" + data_dir_name
	return data_dir_path, obj, sub

def get_grasp_num(grasp_message):
	msg_bits = grasp_message.split()
	grasp_num_str = grasp_message[-1]
	try:
		return int(grasp_num_str)
	except ValueError:
		rospy.logerr("Cannot determine grasp str for string '" + grasp_message + "'.")
		return int(raw_input("Please insert the grasp number at the end of the string."))

def set_grasp_num(grasp_message, new_num):
	for idx, c in enumerate(grasp_message):
		if c.isdigit():
			grasp_message = grasp_message[:idx]

	return grasp_message + str(new_num)
