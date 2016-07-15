import rospy
from threading import Lock

from topic_monitor import TopicMonitor
from grasp_manager.shared_globals import kinect_data_topics

class KinectMonitor:
        # Parameters:
        #   topic_dict: a pairing of topic strings and associated callbacks
    def __init__(self, gm, gui):
        global kinect_data_topics
        self.gui = gui
        self.grasp_manager = gm
        self.acceptable_timeout = rospy.Duration(0.25)
        self.startup_delay = rospy.Duration(1)

        self.topic_dict = {}
        self.topic_dict[scene_capture_topics[0]] = self.kinect_rgb_down
        self.topic_dict[scene_capture_topics[1]] = self.kinect_depth_down
        self.topic_dict[scene_capture_topics[2]] = self.realsense1_down
        self.topic_dict[scene_capture_topics[3]] = self.realsense2_down

        self.video_need_reset_sync = Lock() # Ensures only one reset mechanism goes through the process of a reset
        self.video_reset_lock = Lock() # Prevents continuation of the grasp_manager until kinect reset occurs
        self.video_reset_lock.acquire()
        self.gui.register_button_cb("_video_reset", self.video_reset)
        self.gui.disable_element("_video_reset")

        self.monitor_dict = {}
        #print "topic_dict: ", self.topic_dict
        for t in self.topic_dict:
            self.monitor_dict[t] = TopicMonitor(t, t, self.acceptable_timeout, self.startup_delay, self.topic_dict[t])

        self.pause_monitors()

    def kill_monitors(self):
        for k in self.monitor_dict:
            self.monitor_dict[k].kill_monitor()

    def pause_monitors(self):
        for k in self.monitor_dict:
            self.monitor_dict[k].pause_monitor()

    def resume_monitors(self):
        for k in self.monitor_dict:
            self.monitor_dict[k].resume_monitor()

    def video_reset(self):
        self.video_reset_lock.release()

    def kinect_rgb_down(self):
        rospy.logerr("Kinect RGB stream down! The video capture software (or hardware) needs a reset before proceeding.")
        self.video_reset_common()

    def kinect_depth_down(self):
        rospy.logerr("Kinect DEPTH stream down! The video capture software (or hardware) needs a reset before proceeding.")
        self.video_reset_common()

    def video_reset_common(self):
        if not self.video_need_reset_sync.acquire(False):
            return

        self.gui.pickle_interface()
        self.gui.enable_element("_video_reset")
        self.video_reset_lock.acquire()
        self.gui.unpickle_interface()
        self.video_need_reset_sync.release()

    def realsense1_down(self):
        rospy.logerr("Realsense camera 1 down. Needs reset before proceeding.")
        video_reset_common()

    def realsense2_down(self):
        rospy.logerr("Realsense camera 2 down. Needs reset before proceeding.")
        video_reset_common()
