#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty

import os
import time

def sound_cb(msg):
	duration = 1
	freq = 200
	os.system("say 'Hello Javier'")
	time.sleep(2)
	os.system("play --no-show-progress --null --channels 1 synth %s sine %f" % (duration, freq))

if __name__ == "__main__":
	rospy.init_node("sounder")
	rospy.loginfo("Sounder online.")

	sound_sub = rospy.Subscriber("/make_beep", Empty, sound_cb)
	rospy.spin()
