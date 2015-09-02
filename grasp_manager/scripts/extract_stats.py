#! /usr/bin/env python
import rospy
import rosbag

# This file is meant to extract basic statistics from the snapshotted grasp data.
# 	it will create a small csv file containing an object/subject cross with the 
#	number of optimals and number of extremes
# The data is plopped down where you run this file.

import os
import csv

from shared_playback import *

class ObjLine:
	def __init__(self, obj_num):
		self.obj_num = obj_num
		self.task = task
		self.sub_dict = {}

	def get_csv_line(self, sub_list):
		csv_line = str(obj_num) + ","
		for sub_num in sub_list:
			csv_line += self.make_cell_entry(self, sub_num)

		return csv_line

	def make_cell_entry(self, sub_num):
		try:
			data = self.sub_dict[sub_num]
			return str(data.optimal_cnt) + "-" + str(data.extreme_cnt) + ","
		except:
			return "0-0,"


class GraspSet:
	def __init__(self, optimal_cnt, extreme_cnt):
		self.optimal_cnt = optimal_cnt
		self.extreme_cnt = extreme_cnt

class JointDataLine:
	def __init__(self, obj_num, sub_num, grasp_num, task, is_optimal, oe_num, jnts):
		self.obj_num = obj_num
		self.sub_num = sub_num
		self.grasp_num = grasp_num
		self.task = task
		self.is_optimal = is_optimal
		if self.is_optimal:
			self.optimal_extreme_str = "optimal"
		else:
			self.optimal_extreme_str = "extreme"
		self.oe_num = oe_num
		self.hand_jointstate = jnts

	def make_csv_line(self, field_list):
		csv_line = ""
		for f in field_list:
			if "obj" in f.lower():
				csv_line += str(self.obj_num) + ","
			elif "sub" in f.lower():
				csv_line += str(self.sub_num) + ","
			elif "grasp" in f.lower():
				csv_line += str(self.grasp_num) + ","
			elif "task" in f.lower():
				csv_line += self.task + ","
			elif "index" in f.lower():
				csv_line += str(self.oe_num) + ","
			elif "optimality" in f.lower():
				csv_line += self.optimal_extreme_str + ","
			else:
				# Ought to be a joint state name
				try:
					idx = self.hand_jointstate.name.index(f)
					csv_line += str(self.hand_jointstate.position[idx]) + ","
				except ValueError:
					rospy.logerr("Unexpected field name: " + f + ". skipping.")
		# Remove the trailing delimiter
		return csv_line[:-1] + "\n"

def get_obj_sub_num_lists(data_dir):
	dirs = os.listdir(data_dir)
	obj_num_list = set()
	sub_num_list = set()
	
	for d in dirs:
		name_bits = d.split("_")
		obj_num_list.add(int(name_bits[0][3:]))
		sub_num_list.add(int(name_bits[1][3:]))

	return (list(obj_num_list), list(sub_num_list))



def extract_nested_table(grasp_data_directory):
	(obj_num_list, sub_num_list) = get_obj_sub_num_lists(grasp_data_directory)
	objs = {}
	for obj_num in obj_num_list:
		objs[obj_num] = ObjLine(obj_num)

	# Harvest the necessary data
	for obj_num in obj_num_list:
		for sub_num in sub_num_list:
			data_dir = "obj" + str(obj_num) + "_sub" + str(sub_num)
			bag_path = grasp_data_directory + "/" + data_dir + "_defrag"
			# Determine the location of a bag
			if os.path.exists(bag_path):
				bag_path += "/grasp_extreme_snapshots.bag"

			elif os.path.exists(bag_path.split("_defrag")[0]):
				bag_path = bag_path[0] + "/grasp_extreme_snapshots.bag"
			else:
				continue
			
			# Open the bag
			try:
				in_bag = rosbag.Bag(bag_path)
			except:
				continue

			# Wrangle some data
			optimal_count = 0
			extreme_count = 0
			for topic, msg, t in in_bag.read_messages():
				if msg.is_optimal:
					optimal_count += 1
				elif msg.is_optimal == False:
					extreme_count += 1

			# Save the data
			d = GraspSet(optimal_count, extreme_count)
			obj[obj_num].sub_dict[sub_num] = d



def get_joint_value_table(grasp_data_dir):
	data_dirs = get_data_dirs(grasp_data_dir)
	field_names = ["Object", "Subject", "Grasp", "Optimality", "Index", "Task", "inner_f1", "inner_f2", "inner_f3", "outer_f1", "outer_f2", "outer_f3", "spread"]
	out_lines = []
	for (data_dir, obj_num, sub_num) in data_dirs:
		extreme_bag = None
		try:
			extreme_bag = get_extreme_bag(data_dir)
		except:
			rospy.logerr("Skipping data directory " + data_dir)
			continue

		for topic, msg, t in extreme_bag.read_messages():
			oe_num = 0
			if msg.is_optimal:
				oe_num = msg.optimal_num
			else:
				oe_num = msg.extreme_num
			out_lines.append(JointDataLine(obj_num, sub_num, msg.grasp_num, msg.task, msg.is_optimal, oe_num, msg.hand_joints))

		extreme_bag.close()

	write_csv_table("joint_angle_master.csv", field_names, out_lines)

def get_extreme_bag(data_dir):
	snapshot_path = data_dir + "/" + "grasp_extreme_snapshots.bag"
	if not os.path.exists(snapshot_path):
		rospy.logerr("Snapshot bag at " + grasp_data_dir + " does not exist.")
		raise IOError

	return try_bag_open(snapshot_path)
		

def write_csv_table(file_path, field_names, out_rows):
	# Write out to file
	if os.path.exists(file_path):
		rospy.logwarn(file_path + " file already exists.")
		user_input = raw_input("Overwrite? (y/n)")
		if user_input.lower() != "y":
			rospy.loginfo("Skipping.")
			return

	out_file = open(file_path, "w")
	csv_header = ",".join(field_names) + "\n"
	out_file.write(csv_header)
	for row in out_rows:
		out_file.write(row.make_csv_line(field_names))
	out_file.close()
	

if __name__ == "__main__":
	rospy.init_node("extract_stats")
	rospy.logerr("This file currently makes no separation between natural and pickup tasks.")

	#extract_nested_table(grasp_data_directory)

	# Get joint value statistics
	get_joint_value_table(grasp_data_directory)
