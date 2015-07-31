#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty

import os
import time

def sound_cb(msg):
	duration = 1
	#freq = 22000
	freq = 5000
	os.system("play --no-show-progress --null --channels 1 synth %s sine %f" % (duration, freq))

if __name__ == "__main__":
	rospy.init_node("sounder")
	rospy.loginfo("Sounder online.")

	sound_sub = rospy.Subscriber("/make_beep", Empty, sound_cb)
	rospy.spin()
