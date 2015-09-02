#! /usr/bin/env python
# As a result of necessary data format changes during execution of the 
#	grasping study, we now (8/30/2015) have various and mixed data formats
#	in addition to distributed data due to kinect capture failures.
#
# This script is meant to make all data consistent with the most recent
#	revision of the grasp capture code (commit 692b713924d17ec52ff83fa7b9369175a32651e4). It currently consists of two processes: consolidating fragmented data, then translating annotations from older bag files to newer type separated bag files.
import rospy
import rosbag

from wam_msgs.msg import StampedString

from shared_globals import *
from shared_playback import *

import os
import yaml
import subprocess
import copy
from operator import itemgetter

# Description: Finds disparate bag files in the grasp data directory and 
#	joins them into single bag files in a single defrag folder. Then
# 	replaces originals with defrag directory.
def consolidate_fragments(data_directory):
	defrag_temp_suffix = "_defrag"
	frag_list = find_fragmented_subjects(data_directory, defrag_temp_suffix)

	for frag_set in frag_list:
		base_name = frag_set[0][0]
		merge_data(data_directory, base_name, defrag_temp_suffix, frag_set[1:])

# Returns a nested list of fragmented data directories with the first index
#	being the base file name.
# BAGS MUST BE IN CHRONOLOGICAL ORDER. First timestamps first.
def find_fragmented_subjects(data_dir, defrag_suffix):
	rospy.loginfo("Finding fragmented directories under: " + data_dir)
	data_dirs = os.listdir(data_dir)

	grasp_data_dict = {}
	for d in data_dirs:
		if defrag_suffix in d:
			continue
		dir_pieces = d.split("_")
		obj_num = sub_num = None
		try:
			obj_num = int(dir_pieces[0][3:])
			sub_num = int(dir_pieces[1][3:])
		except ValueError:
			rospy.logerr("Cannot pull object and subject number from grasp directory name: " + d)
			sys.exit(1)

		# Add to grasp_data_dict with timestamp for sorting
		if not obj_num in grasp_data_dict.keys():
			grasp_data_dict[obj_num] = {}
		
		relative_stamp = None
		try:
			relative_stamp = get_dir_relative_timestamp(data_dir + "/" + d)
		except:
			continue
		
		try:
			grasp_data_dict[obj_num][sub_num].append((d, relative_stamp))
		except KeyError:
			grasp_data_dict[obj_num][sub_num] = [(d, relative_stamp)]

	print "grasp_data_dict: ", grasp_data_dict
	#raw_input("Does that include every directory in the data directory with the start timestamp?")

	# Filter out directories that dont need concatenation
	frag_list = []
	for obj_num in grasp_data_dict:
		for sub_num in grasp_data_dict[obj_num]:
			if len(grasp_data_dict[obj_num][sub_num]) > 1:
				# This obj/sub is fragmented
				grasp_data_dict[obj_num][sub_num].insert(0, ("obj" + str(obj_num) + "_sub" + str(sub_num), 0))
				frag_list.append(grasp_data_dict[obj_num][sub_num])

	# Reorder based on timestamps
	for l in frag_list:
		l.sort(key=lambda tup: tup[1])

	print "frag list: ", frag_list
	#raw_input("How does that fragmented directory list look? Sorted chronologically?")

	return frag_list


def get_dir_relative_timestamp(base_dir):
	sample_bag = None
	sample_bag_path = base_dir + "/" + "human_grasp_annotations.bag"
	try:
		sample_bag = try_bag_open(sample_bag_path)
		bag_info_dict = yaml.load(sample_bag._get_yaml_info())
		sample_bag.close()
		return bag_info_dict['start']
	except:
		rospy.logwarn("Failed to get first timestamp.")
		
		sample_bag_path = base_dir + "/" + "robot_grasp_annotations.bag"
		try: 
			sample_bag = try_bag_open(sample_bag_path)
			bag_info_dict = yaml.load(sample_bag._get_yaml_info())
			sample_bag.close()
			return bag_info_dict['start']
		except:
			rospy.logerr("No time data available for " + base_dir + ". Must skip to avoid out of order issues.")
			raise IOError


def merge_data(grasp_data_directory, base_file_name, defrag_suffix, frag_dirs):
	defrag_dir = create_defrag_dir(grasp_data_directory, base_file_name, defrag_suffix)
	rospy.loginfo("Reformatting into " + defrag_dir)
	for idx, dir_name in enumerate(frag_dirs):
		frag_dirs[idx] = grasp_data_directory + "/" + dir_name[0]

	# Simple concatenation
	concatenate_bags(defrag_dir, frag_dirs, "kinect_robot_capture.bag")
	concatenate_bags(defrag_dir, frag_dirs, "kinect_hand_capturef.bag")
	concatenate_bags(defrag_dir, frag_dirs, "kinect_hand_capturel.bag")
	concatenate_bags(defrag_dir, frag_dirs, "wam_traj.bag")
	concatenate_bags(defrag_dir, frag_dirs, "hand_commands.bag")

	# Concatenation that requires more analysis
	concatenate_annotations(defrag_dir, frag_dirs, "human_grasp_annotations.bag", "Human grasp capture start", "Human grasp capture end")
	concatenate_annotations(defrag_dir, frag_dirs, "robot_grasp_annotations.bag", "Motion Capture Start", "Motion Capture End")

def create_defrag_dir(data_dir, base_name, suffix):
	defrag_path = data_dir + "/" + base_name + suffix
	if not os.path.exists(defrag_path):
		os.mkdir(defrag_path, 0755)	
	return defrag_path

# Always saves a bag, even if its empty
def concatenate_bags(result_dir, bag_dir_list, bag_name):
	result_bag_path = result_dir + "/" + bag_name
	if os.path.exists(result_bag_path):
		rospy.loginfo("Bag " + result_bag_path + " already exists.")
		return
	out_bag = rosbag.Bag(result_bag_path, "w")
	for bag_path in bag_dir_list:
		bag_path = bag_path + "/" + bag_name
	
		# Ignore if bag is missing.
		if not os.path.exists(result_bag_path):
			rospy.loginfo("Could not find bag " + bag_path)
			continue	
		
		# Skip bag if duration less than 10 sec
		try:
			in_bag = try_bag_open(bag_path)
			bag_info_dict = yaml.load(in_bag._get_yaml_info()) 
			if bag_info_dict['duration'] <= 10:
				in_bag.close()
				rospy.loginfo("Skipping " + bag_path  + " with duration " + str(bag_info_dict['duration']))
				continue
		except:
			# No bag present
			continue

		# Save messages
		for topic, msg, t in in_bag.read_messages():
			out_bag.write(topic, msg, t)

		in_bag.close()
	out_bag.close()

def concatenate_annotations(defrag_dir, frag_dirs, bag_name, start_msg, end_msg):
	output_bag_path = defrag_dir + "/" + bag_name
	output_bag = rosbag.Bag(output_bag_path, "w")
	grasp_offset = 0
	for frag_dir in frag_dirs:
		# Note if bag is missing
		bag_path = frag_dir + "/" + bag_name
		if not os.path.exists(bag_path):
			rospy.logerr("Directory " + frag_dir + " missing " + bag_name + " file.")
			continue
		cur_bag = None
		try:
			cur_bag = try_bag_open(bag_path)
		except:
			continue

		first_grasp_set_msg = True
		extreme_without_new = False
		for topic, msg, t in cur_bag.read_messages():
			# Record only first start annotation message and last end one
			if start_msg in msg.data and frag_dir == frag_dirs[0]:
				output_bag.write(topic, msg, t)
				continue
			elif end_msg in msg.data and frag_dir == frag_dirs[-1]:
				output_bag.write(topic, msg, t)
				continue
			elif start_msg in msg.data or end_msg in msg.data:
				continue

			if "Bag file closing. End final grasp set" in msg.data:
				# This message is unnecessary
				continue

			if "Optimal grasp" in msg.data and first_grasp_set_msg == True:
				# Someone didn't begin a new grasp set for this file.
				msg2 = copy.deepcopy(msg)
				msg2.stamp -= rospy.Duration(0.1)
				grasp_offset += 1
				msg.data = "Begin grasp set " + str(grasp_offset)
				output_bag.write(topic, msg2, t - rospy.Duration(0.1))
	
			if "Begin grasp set" in msg.data:
				first_grasp_set_msg = False
				grasp_offset += 1

			if "Grasp range extreme" in msg.data:
				extreme_without_new = True
			elif "Begin grasp set" in msg.data:
				extreme_without_new = False

			if "Optimal grasp" in msg.data and extreme_without_new:
				# Somebody forgot to add a new grasp between grasps
				rospy.logwarn("Found extremes between optimals. Repairing.")
				grasp_offset += 1
				msg2 = StampedString(t - rospy.Duration(0.1), "Begin grasp set " + str(grasp_offset))
				output_bag.write(topic, msg2, t - rospy.Duration(0.1))

			# Adjust the standard annotations grasp numbers
			if "Begin grasp set" in msg.data or "Optimal grasp" in msg.data or "Grasp range extreme" in msg.data or "There is rotational symmetry about" in msg.data:
				msg.data = set_grasp_num(msg.data, grasp_offset)
				
			# Just add the rest
			output_bag.write(topic, msg, t)

	output_bag.close()


def reformat_bags(grasp_data_directory):
	good_dirs = get_data_dirs(grasp_data_directory + "/" + "good")
	bad_dirs = get_data_dirs(grasp_data_directory + "/" + "bad")
	unformatted_dirs = good_dirs
	unformatted_dirs.extend(bad_dirs)
	
	print "Directories needing bag file formatting assistance: ", unformatted_dirs
	#raw_input("How does that list look?")

	for d in unformatted_dirs:
		rospy.loginfo("Splitting data bag file for directory: " + d)
		split_general_info(d)

	rospy.loginfo("Info bag splitting complete. Verify results.")

def get_data_dirs(grasp_data_directory):
	data_dirs = os.listdir(grasp_data_directory)
	unformatted_dirs = []
	for d in data_dirs:
		general_info_path = grasp_data_directory + "/" + d + "/" + "general_info.bag"
		if os.path.exists(general_info_path):
			unformatted_dirs.append(grasp_data_directory + "/" + d)

	return unformatted_dirs



def split_general_info(bag_dir):
	hand_bag = rosbag.Bag(bag_dir + "/" + "human_grasp_annotations.bag", "w")
	robot_bag = rosbag.Bag(bag_dir + "/" + "robot_grasp_annotations.bag", "w")
	general_info_bag = None
	try:
		general_info_bag = try_bag_open(bag_dir + "/" + "general_info.bag")
	except:
		return

	save_to_hand_bag = True
	bag_info_dict = yaml.load(general_info_bag._get_yaml_info())
	try:
		s_time = rospy.Time(bag_info_dict['start'])
	except KeyError:
		# No data in bag
		return

	for topic, msg, t in general_info_bag.read_messages():
		if "Motion Capture Start" in msg.data:
			hand_bag.write(topic, StampedString(t, "Human grasp capture end"), t + rospy.Duration(1))
			hand_bag.write(topic, StampedString(s_time, "Human grasp capture start"), s_time - rospy.Duration(1))
			save_to_hand_bag = False

		if save_to_hand_bag:
			if "Human grasp capture start" in msg.data or "Human grasp capture end" in msg.data:
				continue
			hand_bag.write(topic, msg, t)
		else:
			robot_bag.write(topic, msg, t)

	hand_bag.close()
	robot_bag.close()
	general_info_bag.close()

if __name__ == "__main__":
	rospy.init_node("data_format_enforcer")
	rospy.loginfo("Data format enforcer node online.")

	if "home" in grasp_data_directory:
		rospy.logerr("Using home directory, not harddrive.")

	reformat_bags(grasp_data_directory)

	raw_input("Press enter to consolidate fragments.")

	consolidate_fragments(grasp_data_directory + "/" + "good")
	consolidate_fragments(grasp_data_directory + "/" + "bad")

	rospy.loginfo("Data reformat complete.")
