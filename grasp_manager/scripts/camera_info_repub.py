#! /usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

prefix = "kinect2/qhd/"
output_topic = prefix + "camera_info"
input_topic = prefix + "camera_info_raw"

def repub_info(msg):
	msg.header.stamp = rospy.Time.now()
	pubby.publish(msg)

if __name__ == "__main__":
	rospy.init_node("camera_info_repub")
	rospy.loginfo("Camera info republisher online")


	pubby = rospy.Publisher(output_topic, CameraInfo, queue_size="10")
	subby = rospy.Subscriber(input_topic, CameraInfo, repub_info)

	rospy.spin()
