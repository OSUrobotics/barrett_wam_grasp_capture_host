import rospy

def grasp_hand():
	rospy.logdebug("Waiting for hand close service.")
	rospy.wait_for_service("/bhand/close_grasp")
	try:
		close_hand = rospy.ServiceProxy('/bhand/close_grasp', Empty)
		resp = close_hand()
	except:
		rospy.logerr("Closing barrett hand failed.")
