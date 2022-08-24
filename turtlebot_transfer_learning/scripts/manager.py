#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from turtlebot_transfer_learning_srvs.srv import PrimitiveAction, PPO_Action

 # Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Manager(object):

    def __init__(self):

        """
        Initializes manager node.
        """

        # Initialize manager node 
        rospy.init_node("manager")
        rospy.on_shutdown(self.shutdown)

        # Subscribe to episode reset topic
        self.reset = rospy.Subscriber("turtlebot_transfer_learning/episode_reset", Empty, self.run_episode)

        # Create CvBridge 
        self.bridge = CvBridge()

        # Create service proxy for primitive moves
        self.get_action = rospy.ServiceProxy("ppo_action_srv", PPO_Action)
        self.move = rospy.ServiceProxy("/turtlebot_transfer_learning/primative_move_action", PrimitiveAction)

        try:
            self.time_steps = rospy.get_param("turtlebot_transfer_learning/time_steps")
        except (KeyError, rospy.ROSException):
            rospy.logwarn("Error retrieving 'time_steps' parameter, defaulting to 100")
            self.time_steps = 500
        
        self.action_list = ["clockwise", "counter_clockwise", "forward", "backward"]

        self.run_episode()

    def run_episode(self):

        for i in range(self.time_steps):
            
            try:
                rgb_img = rospy.wait_for_message("/camera/rgb/image_raw", Image, rospy.Duration(1))
                # depth_img = rospy.wait_for_message("/camera/depth_registered/image_raw", Image, rospy.Duration(1))
                cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_img, "rgb8")
                # cv_image_depth = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")

            except rospy.ROSException:
                rospy.logerr("Image not recieved")
                rospy.signal_shutdown()
            except CvBridgeError:
                rospy.logerr("Unable to convert image message to cv2 image")
                rospy.signal_shutdown()

            # # Check for done
            # if self.done(cv_image_rgb, cv_image_depth):
            #     rospy.loginfo("Turtlebot navigated to goal position")
            #     break

            resized_cv_image = cv2.resize(cv_image_rgb, (100, 100))
            action_num = self.get_action((resized_cv_image.flatten()).tolist())

            # Move the robot
            self.move(self.action_list[int(action_num.action)])

    def shutdown(self):
        pass

    def done(self, cv_image_rgb, cv_image_depth):

        # Convert to hsv colorspace
        hsv = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
        cv_image_rgb
        rgbd = cv2.cvtColor(cv_image_depth, cv2.COLOR_RGB)

        # Lower bound and upper bound for gray color
        lower_bound = np.array([0, 0, 71])   
        upper_bound = np.array([0, 0, 31])

        # find the colors within ghe boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # remove noise 
        kernel = np.ones((7,7),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        m = cv2.moments(mask)

        # object detected
        if m['m00'] != 0:
            cx = int(m['m10']/m['00'])
            cy = int(m['m01']/m['m00'])

            print(cv_image_depth[cx][cy])


if __name__ == '__main__':
    Manager()