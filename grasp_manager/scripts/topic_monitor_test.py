#! /usr/bin/env python
import rospy
from topic_monitor import TopicMonitor

def kinect_down():
    print "KINECT DOWN!"

if __name__ == "__main__":
    rospy.init_node("topic_monitor_test")

    kinect_data_topic = "/kinect2/qhd/image_color_rect"
    timeout = rospy.Duration(0.25)
    startup_delay = rospy.Duration(1)
    t = TopicMonitor('t1', kinect_data_topic, timeout, startup_delay, kinect_down)

    i = 1
    while not rospy.is_shutdown() and i < 3:
        raw_input("Press enter to pause the monitor")
        t.pause_monitor()

        raw_input("Press enter to resume the monitor")
        t.resume_monitor()
        i = i + 1
    t.kill_monitor()
    rospy.spin()
