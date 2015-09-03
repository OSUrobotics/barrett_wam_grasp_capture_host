import rosbag
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
	grasp_num_str = msg_bits[-1]
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

# (Data dir, obj_num, sub_num)
def get_data_dirs(grasp_data_dir):
	good_dirs = get_data_subdirs(grasp_data_dir + "/" + "good")
	bad_dirs  = get_data_subdirs(grasp_data_dir + "/" + "bad" )
	good_dirs.extend(bad_dirs)
	print "Data directories to process: ", good_dirs

	return good_dirs

def get_data_subdirs(root):
	dir_list = os.listdir(root)
	dir_list = sorted(dir_list)

	# Put the defrag dirs in the list
	out_list = []
	for d in dir_list:
		if "defrag" in d:
			out_list.append(d)

	for d in dir_list:
		if not os.path.isdir(root + "/" + d):
			continue
		add_list = True
		for d2 in out_list:
			if d2.split("_defrag")[0] in d:
				add_list = False

		if add_list:
			out_list.append(d)

	# Make them absolute paths
	for idx, d in enumerate(out_list):
		name_bits = d.split("_")
		obj_num = int(name_bits[0][3:])
		sub_num = int(name_bits[1][3:])
		out_list[idx] = (root + "/" + d, obj_num, sub_num)

	return out_list

def try_bag_open(bag_path):
	try:
		b = rosbag.Bag(bag_path, "r")
		return b
	except rosbag.bag.ROSBagUnindexedException:
		print "unindexed bag... Trying to reindex."
		os.system("rosbag reindex " + bag_path)
		try:
			b = rosbag.Bag(bag_path, "r")
			return b
		except:
			print "Could not reindex and open "
			raise IOError



