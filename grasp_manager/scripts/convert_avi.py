#! /usr/bin/env python
import os
from shared_playback import *

if __name__ == "__main__":
    dirs = get_data_dirs(grasp_data_directory)
    for (data_dir, obj_num, sub_num) in dirs:
            smi_robot_path = data_dir + "/" + "smi_robot"
            if not os.exists(smi_robot_path):
                print "No data directory in ", smi_robot_path
                continue
            os.system("avconv *recording.avi -c:a Mp4_format_recording.mp4")

