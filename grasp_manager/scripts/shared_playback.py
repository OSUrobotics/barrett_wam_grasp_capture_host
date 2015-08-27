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
