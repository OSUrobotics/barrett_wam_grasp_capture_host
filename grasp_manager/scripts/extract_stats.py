#! /usr/bin/env python
import rospy
import rosbag

# This file is meant to extract basic statistics from the snapshotted grasp data.
# 	it will create a small csv file containing an object/subject cross with the 
#	number of optimals and number of extremes
# The data is plopped down where you run this file.

import os
import csv
import copy

from grasp_manager.shared_playback import *

study_root_dir = os.path.expanduser("~") + "/grasping_user_study"

# Output stat file paths
average_grasps_per_subject_path = study_root_dir + "/processed_data/grasp_results" + "/" + "subject_grasps_averages_master.csv"
out_extreme_file_path = study_root_dir + "/processed_data/grasp_results" + "/" + "grasps_with_extremes_count.csv"
oe_stat_path = study_root_dir + "/processed_data/grasp_results/" + "optimal_extreme_count_master.csv"
oe_stat_agg_path = study_root_dir + "/processed_data/grasp_results/" + "optimal_extreme_agg_master.csv"

# Optimal/Extreme record used to remember peoples' optimal/extreme
#	counts of a per object basis
class OERecord:
	def __init__(self, obj_num, sub_num, good_optimal_cnt, good_extreme_cnt, bad_optimal_cnt, bad_extreme_cnt):
		self.obj_num = obj_num
		self.sub_num = sub_num
		self.good_optimal_cnt = good_optimal_cnt
		self.good_extreme_cnt = good_extreme_cnt
		self.bad_optimal_cnt = bad_optimal_cnt
		self.bad_extreme_cnt = bad_extreme_cnt
	
	def add_item(self, item):
		self.good_extreme_cnt += item.good_extreme_cnt
		self.good_optimal_cnt += item.good_optimal_cnt
		self.bad_extreme_cnt += item.bad_extreme_cnt
		self.bad_optimal_cnt += item.bad_optimal_cnt

	def make_csv_line(self, field_list):
		csv_line = ""
		for f in field_list:
			if "obj" in f.lower():
				csv_line += str(self.obj_num) + ","
			elif "sub" in f.lower():
				csv_line += str(self.sub_num) + ","
			elif "optimal" in f.lower():
				csv_line += str(self.good_optimal_cnt + self.bad_optimal_cnt) + ","
			elif "extreme" in f.lower():
				csv_line += str(self.good_extreme_cnt + self.bad_extreme_cnt) + ","
			elif "good" in f.lower():
				csv_line += str(self.good_optimal_cnt) + ","
			elif "bad" in f.lower():
				csv_line += str(self.bad_optimal_cnt) + ","
			else:
				rospy.logerr("Unknown field name in OERecord: " + f)

		return csv_line[:-1] + "\n"

class DataLine:
	def __init__(self, obj_num, sub_num, grasp_num, task, is_optimal, oe_num, jnts, stamp, is_bad):
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
		self.stamp = stamp
		self.is_bad = is_bad

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
			elif "stamp" in f.lower():
				csv_line += self.stamp + ","
			elif "good" in f.lower():
				if self.is_bad:
					csv_line += "bad,"
				else:
					csv_line += "good,"
			else:
				# Ought to be a joint state name
				try:
					idx = self.hand_jointstate.name.index(f)
					csv_line += str(self.hand_jointstate.position[idx]) + ","
				except ValueError:
					rospy.logerr("Unexpected field name: " + f + ". skipping.")
		
		# Remove the trailing delimiter and add a newline
		return csv_line[:-1] + "\n"

def get_obj_sub_num_lists(data_dir):
	(o,s) = get_obj_sub_sets(data_dir + "/" + "good")
	(o2,s2) = get_obj_sub_sets(data_dir + "/" + "bad")

	o.update(o2)
	s.update(s2)
	print "obj set: ", o
	print "sub set: ", s
	return (list(o), list(s))

def get_obj_sub_sets(data_dir):
	dirs = os.listdir(data_dir)
	obj_num_list = set()
	sub_num_list = set()
	
	for d in dirs:
		name_bits = d.split("_")
		print name_bits
		obj_num_list.add(int(name_bits[0][3:]))
		sub_num_list.add(int(name_bits[1][3:]))

	return (obj_num_list, sub_num_list)



def extract_optimal_extreme_count_table(grasp_data):
	oe_field_names = ['Object', 'Subject', 'Optimal Count', 'Extreme Count']
	oe_agg_field_names = ['Object', 'Total Optimal', 'Total Extreme', 'Total Good', 'Total Bad']
	(obj_num_list, sub_num_list) = get_obj_sub_num_lists(grasp_data_directory)
	oe_dict = {}
	for obj_num in obj_num_list:
		oe_dict[obj_num] = {}
		for sub_num in sub_num_list:
			oe_dict[obj_num][sub_num] = OERecord(obj_num, sub_num, 0,0,0,0)

	# Wrangle some data
	for d in grasp_data:
		if d.is_optimal:
			if d.is_bad:
				oe_dict[d.obj_num][d.sub_num].bad_optimal_cnt += 1
			else:
				oe_dict[d.obj_num][d.sub_num].good_optimal_cnt += 1
		else:
			if d.is_bad:
				oe_dict[d.obj_num][d.sub_num].bad_extreme_cnt += 1
			else:
				oe_dict[d.obj_num][d.sub_num].good_extreme_cnt += 1
	
	# Save the data
	out_list = []
	for o in oe_dict:
		for s in oe_dict[o]:
			if oe_dict[o][s].good_extreme_cnt > 0 or oe_dict[o][s].good_optimal_cnt > 0 or oe_dict[o][s].bad_extreme_cnt > 0 or oe_dict[o][s].bad_optimal_cnt > 0:
				out_list.append(oe_dict[o][s])
	write_csv_table(oe_stat_path, oe_field_names, out_list)

	# Aggregate some data
	agg_out_list = []
	for obj_num in obj_num_list:
		obj_agg = OERecord(obj_num, -1, 0, 0, 0, 0)
		for s in oe_dict[obj_num]:
			obj_agg.add_item(oe_dict[obj_num][s])

		agg_out_list.append(obj_agg)

	write_csv_table(oe_stat_agg_path, oe_agg_field_names, agg_out_list)

def get_joint_value_table(grasp_data):
	field_names = ["Good or Bad", "Object", "Subject", "Grasp", "Optimality", "Index", "Task", "inner_f1", "inner_f2", "inner_f3", "outer_f1", "outer_f2", "outer_f3", "spread"]
	
	out_path = os.path.expanduser("~") + "/grasping_user_study/processed_data/grasp_results/grasp_joints" + "_joint_angle_master.csv"
	write_csv_table(out_path, field_names, grasp_data)


def get_average_grasps_per_subject(grasp_data):
	global average_grasps_per_subject_path
	sub_dict = {} # Tuple of good grasp counts, bad grasp counts, and set of objects
	for d in grasp_data:
		if not d.is_optimal:
			continue

		s = d.sub_num
		if d.sub_num in sub_dict.keys():
			if d.is_bad:
				sub_dict[s][1] += 1
			else:
				sub_dict[s][0] += 1
			sub_dict[s][2].add(d.obj_num)
		else:
			if d.is_bad:
				sub_dict[s] = [0, 1, set()]
			else:
				sub_dict[s] = [1, 0, set()]
				
			sub_dict[s][2].add(d.obj_num)

	print "How does that look? ", sub_dict
	out_fields = ["subject", "good grasps", "bad grasps", "average good", "average bad"]
	out_dict = {"subject":0, "good grasps":0, "bad grasps":0, "average good":0, "average bad":0}
	out_rows = []
	for k in sorted(sub_dict.keys()):
		cur_dict = copy.deepcopy(out_dict)
		num_obj = float(len(sub_dict[k][2]))
		cur_dict["subject"] = k
		cur_dict["good grasps"] = sub_dict[k][0]
		cur_dict["bad grasps"] = sub_dict[k][1]
		cur_dict["average good"] = sub_dict[k][0] / num_obj
		cur_dict["average bad"] = sub_dict[k][1] / num_obj
		out_rows.append(cur_dict)

	# Write it to file
	with open(average_grasps_per_subject_path, "w") as out_file:
		out_csv = csv.DictWriter(out_file, out_fields)
		out_csv.writeheader()
		for l in out_rows:
			out_csv.writerow(l)

	rospy.loginfo("Wrote user average grasp tendencies to file")


def get_grasps_with_and_without_extremes(grasp_results):
	global out_extreme_file_path
	grasp_dict = {}
	
	# Break the data into sets of grasps
	for g in grasp_results:
		key = str(g.obj_num) + "-" + str(g.sub_num) + "_" + str(g.grasp_num)
		if key in grasp_dict:
			grasp_dict[key].add(g)
		else:
			grasp_dict[key] = set([g])

	# Check if they have extremes
	grasps_with_extremes = 0
	grasps_without_extremes = 0
	print "number of grasps: ", len(grasp_dict.keys())
	for k in grasp_dict:
		g_set = grasp_dict[k]
		found_extreme = False
		for g in g_set:
			if not g.is_optimal:
				found_extreme = True
				break

		if found_extreme:
			grasps_with_extremes += 1
		else:
			grasps_without_extremes += 1

	
	with open(out_extreme_file_path, "w") as out_file:
		out_file.write(str(grasps_with_extremes) + ", " + str(grasps_without_extremes))
		print "Grasps with extremes: ", grasps_with_extremes, " without: ", grasps_without_extremes
	

def get_all_data(grasp_data_dir):
	data_dirs = get_data_dirs(grasp_data_dir)
	out_lines = []
	for (data_dir, obj_num, sub_num) in data_dirs:
		extreme_bag = None
		try:
			extreme_bag = get_extreme_bag(data_dir)
		except:
			rospy.logerr("Skipping data directory " + data_dir)
			continue

		is_bad = "bad" in data_dir

		for topic, msg, t in extreme_bag.read_messages():
			oe_num = 0
			if msg.is_optimal:
				oe_num = msg.optimal_num
			else:
				oe_num = msg.extreme_num
			out_lines.append(DataLine(obj_num, sub_num, msg.grasp_num, msg.task, msg.is_optimal, oe_num, msg.hand_joints, msg.stamp, is_bad))
			if obj_num == 2 and sub_num == 4:
				print "For object 2 and subject 4, stored: ", msg.grasp_num, " oe_num: ", oe_num, " and is_optimal: ", msg.is_optimal

		extreme_bag.close()

	return out_lines

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

	grasp_data = get_all_data(grasp_data_directory)

	# Get basic optimal and extreme counts
	extract_optimal_extreme_count_table(grasp_data)

	# Get joint value statistics
	get_joint_value_table(grasp_data)

	get_average_grasps_per_subject(grasp_data)

	get_grasps_with_and_without_extremes(grasp_data)
