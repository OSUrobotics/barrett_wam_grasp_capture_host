#! /usr/bin/env python
import os
from grasp_manager.shared_playback import *
import time

if __name__ == "__main__":
    dirs = get_data_dirs(grasp_data_directory)
    for (data_dir, obj_num, sub_num) in dirs:
            smi_robot_path = data_dir + "/" + "smi_robot"
            if not os.path.exists(smi_robot_path):
                print "No data directory in ", smi_robot_path
                continue
            list_dir = os.listdir(smi_robot_path)
            filename = ''
            filename_list = [s for s in list_dir if "recording.avi" in s]
            remove_filename = [s for s in list_dir if "recording.avi.rmf" in s]
            if len(filename_list) != 0 and len(remove_filename) != 0:
                filename_list.remove(remove_filename[0])
            if len(filename_list) != 0:
                filename = smi_robot_path + "/"+filename_list[0]
                outfile = smi_robot_path + "/Mp4_format_recording.mp4"
            print filename
            print outfile
            if len(filename) != 0:
                os.system("avconv -i "+filename+" -c:a copy "+outfile)
                time.sleep(10)

