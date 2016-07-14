#! /usr/bin/env python
# File: topic_oneshot.py
# Description: This file instantiates a ROS node that will listen to a
#    given topic and buffer the last message received on that topic.
#    When the user commands (or the proper service is triggered), the 
#    node will publish the most recent message on the given output topic
# Parameters:
#    in_topic: The topic to buffer
#    out_topic: The topic to output on
import rospy
import rostopic
import threading

class atDict(dict):
        __getattr__= dict.__getitem__
        __setattr__= dict.__setitem__
        __delattr__= dict.__delitem__

class TopicOneShot:
    def __init__(self, in_topics):
        rospy.loginfo("Starting oneshot for topics: %s " % str(in_topics))
        self.in_topics = in_topics

        self.batch_lock = threading.Lock()

        self.out_topics = {}
        for t in in_topics:
            o = self.out_topics[t] = atDict()
            o.t = t
            o.ot = t + '/oneshot'
            o.cur_msg = None
            o.cur_msg_lock = threading.Lock()
            self.get_topic_info(o)
            o.pub = rospy.Publisher(o.ot, o.topic_class, queue_size=1, latch=True)
            o.sub = rospy.Subscriber(o.t, o.topic_class, self.sub_cb, o.t)
        print "out_topics: ", self.out_topics

    def get_topic_info(self, o):
        o.topic_class = rostopic.get_topic_class(o.t, blocking=False)[0]
        if o.topic_class == None:
            rospy.logerr("%s topic not instantiated. One-shot useless." % o.t)
            raise Exception("see ros log.")


    def sub_cb(self, msg, topic):
        # Prevent changing a message mid-batch
        self.batch_lock.acquire()
        self.batch_lock.release()

        o = self.out_topics[topic]
        o.cur_msg_lock.acquire()
        #print "Sub triggered!"
        o.cur_msg = msg
        o.cur_msg_lock.release()

    def publish(self):
        self.batch_lock.acquire()
        for k in self.out_topics:
            o = self.out_topics[k]
            o.cur_msg_lock.acquire()
            if o.cur_msg == None:
                rospy.logerr("No messages in buffer for %s to republish. Skipping." % o.t)
            else:
                if o.cur_msg.header != None:
                    pass
                    #o.cur_msg.header.stamp = rospy.Time.now()
                o.pub.publish(o.cur_msg)
            o.cur_msg_lock.release()
        self.batch_lock.release()


def get_args():
    in_topic=out_topic=None
    try:
        #in_topic_full_name = rospy.search_param('in_topics')
        in_topics = rospy.get_param('~in_topics')
    except KeyError as e:
        rospy.logerr("Could not get parameter in get_args(): ")

    return in_topics

if __name__ == "__main__":
    rospy.init_node('topic_oneshot', anonymous=True)

    in_topics = get_args()

    oneshot = TopicOneShot(in_topics)

    while not rospy.is_shutdown():
        raw_input("Press [Enter] to trigger the one shot")
        oneshot.publish()
