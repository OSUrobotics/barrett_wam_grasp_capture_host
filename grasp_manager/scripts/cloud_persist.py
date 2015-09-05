#! /usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

## All this node does is publish the most current pointcloud it has at a 
## 	predictable rate until it recevies a new one

def cloud_cb(msg):
	global cloud
	cloud = msg

if __name__ == "__main__":
	rospy.init_node("ptcloud_repub")
	rospy.loginfo("repub online.")
	cloud = None

	pubby = rospy.Publisher("/cal_cloud", PointCloud2, queue_size=2)
	subby = rospy.Subscriber("/grasp_cloud", PointCloud2, cloud_cb)

	period = rospy.Duration(0.3)
	while not rospy.is_shutdown():
		if cloud != None:
			cloud.header.stamp = rospy.Time.now()
			pubby.publish(cloud)

		rospy.sleep(period)
