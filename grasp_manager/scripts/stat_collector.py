import csv
import os

base_data_dir = os.expanduser("~") + "/grasp_data_processed"
similar_description_dir = base_data_dir + "/grasp_similarities"
joint_data_dir = base_data_dir + "/grasp_joints"
contact_data_dir = base_data_dir + "/grasp_contacts"

output_dir = base_data_dir + "/results"

if __name__ == "__main__":
	print "Stat mesher online."
	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))
	grasp_set_id = int(raw_input("Similar Grasp Set ID: "))

	similar_path = similar_description_dir + "/" + "obj" + str(obj_num) + "_sub" + str(sub_num) + "_obj5_similar_grasp_" + str(grasp_set_id) + ".csv"
	out_path = output_dir + "/" + "obj" + str(obj_num) + "_sub" + str(sub_num) + "_master" + str(grasp_set_id)
	out_file = open(out_path, "w")
	out_csv = csv.writer
	with open(similar_path, 'r') as similar_file:
		similar_csv = csv.reader(similar_file, delimiter=",")


