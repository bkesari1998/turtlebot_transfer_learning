#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

def episode_reset():
    pub = rospy.Publisher('/turtelbot_transfer_learning/episode_reset', Empty, queue_size=1)
    rospy.init_node('episode_reset', anonymous=True)
    rospy.loginfo("Episode reset pub up")
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        episode_reset()
    except rospy.ROSInterruptException:
        pass