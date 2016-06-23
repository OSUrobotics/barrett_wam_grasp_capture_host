#! /usr/bin/env python
import rospy
import csv
import os
import sys

def agg_data(master_path, out_path):
	agg_data = {0:0, 1:0, 2:0, 3:0, 4:0, 5:0}


	with open(master_path,"r") as master_file:
		master_csv = csv.DictReader(master_file, delimiter=",")
		while True:
			batch = []
			for i in range(5):
				try:
					batch.append(master_csv.next())
				except StopIteration:
					pass


			if len(batch) == 0:
				print "No more batches."
				break
			elif len(batch) != 5:
				rospy.logerr("Input error! Not even batch of five inputs.")
				sys.exit(1)
			
			for i in range(1, 5):
				if batch[i]['object'] != batch[0]['object'] or batch[i]['subject'] != batch[0]['subject'] or batch[i]['grasp'] != batch[0]['grasp'] or batch[i]['idx'] != batch[0]['idx']:
					rospy.logerr("Grasps in same batch aren't the same!")
					print batch[i]

			# Else we have a data point
			#print "batch: ", batch
			successes = 0
			for l in batch:
				if int(l['success']) == 1:
					successes += 1

			agg_data[successes] += 1

	# Write the results
	field_names = []
	out_dict = {}
	for k in sorted(agg_data.keys()):
		f_name = str(k) + " success"
		field_names.append(f_name)
		out_dict[f_name] = agg_data[k]

	with open(out_path, "w") as out_file:
		out_csv = csv.DictWriter(out_file, field_names, delimiter=",")
		out_csv.writeheader()
		out_csv.writerow(out_dict)
	

if __name__ == "__main__":
	rospy.init_node("grasp_verification_agg")

	while True:
		master_path = raw_input("Where is the master verification file?:")
		if not os.path.exists(master_path):
			rospy.logerr("Master path given does not exist.")
			continue
		else:
			if master_path[-1] == '/':
				master_path = master_path[:-1]
			break

	out_path = raw_input("Out path: ")
	if os.path.exists(out_path):
		rospy.logwarn("Out path already exists.")
		overwrite = raw_input("Overwrite? (y/n): ")
		if "n" in overwrite.lower():
			sys.exit(1)
	
	agg_data(master_path, out_path)
	

