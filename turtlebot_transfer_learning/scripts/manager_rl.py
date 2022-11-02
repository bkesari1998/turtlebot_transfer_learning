#!/usr/bin/env python

import rospy
import time
import math

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from turtlebot_transfer_learning_srvs.srv import PrimitiveAction, PPO_Action
from turtlebot_transfer_learning_srvs.msg import NetworkInfo

from apriltag_ros.msg import AprilTagDetectionArray

 # Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Manager(object):

    def __init__(self):

            # # Check for done
            # if self.done(cv_image_rgb, cv_image_depth):
            #     rospy.loginfo("Turtlebot navigated to goal position")
            #     break
        """
        Initializes manager node.
        """

        # Initialize manager node 
        rospy.init_node("manager")

        # Subscribe to episode reset topic
        self.reset = rospy.Subscriber("turtlebot_transfer_learning/episode_reset", Empty, self.run_episode)

        # Create CvBridge 
        self.bridge = CvBridge()

        self.net_info_pub = rospy.Publisher("net_info", NetworkInfo, queue_size=1)

        # Create service proxy for primitive moves
        rospy.wait_for_service("ppo_action_srv")
        self.get_action = rospy.ServiceProxy("ppo_action_srv", PPO_Action)
        self.move = rospy.ServiceProxy("/turtlebot_transfer_learning/primative_move_actions", PrimitiveAction)

        self.rate = rospy.Rate(10)

        try:
            self.time_steps = rospy.get_param("turtlebot_transfer_learning/time_steps")
        except (KeyError, rospy.ROSException):
            rospy.logwarn("Error retrieving 'time_steps' parameter, defaulting to 500")
            self.time_steps = 500
        
        self.action_list = ["clockwise", "counter_clockwise", "forward", "backward"]
        self.counter = 0

        while True:
            rospy.loginfo("Episode " + str(self.counter) + ":")
            self.run_episode()
            msg = ""
            while msg != "y":
                msg = raw_input("Ready for next episode? (y/n): ")
            self.counter += 1

    def run_episode(self):
        
        # Run experiment time_steps
        for i in range(100):
            # Get image from camera
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
            
            # Resize image and send to ppo network
            resized_cv_image = cv2.resize(cv_image_rgb, (100, 100))
            action_num = self.get_action((resized_cv_image.flatten()).tolist())

            # Move the robot
            rospy.loginfo("Timestep " + str(i) + ": " + self.action_list[int(action_num.action)])
            self.move(self.action_list[int(action_num.action)])
            
            tag_detections = rospy.wait_for_message("tag_detections", AprilTagDetectionArray)
            
            reward = -1
            end_episode = False
            reached_goal = False

            if len(tag_detections.detections) != 0:
                for detection in tag_detections.detections:
                    if detection.pose.pose.pose.position.z < 0.3:
                        reward = 1000
                        end_episode = True
                        reached_goal = True
                        break
                    else:
                        if i == 99:
                            end_episode = True
            else:
                if i == 99:
                    end_episode = True

            
            msg = NetworkInfo()
            msg.reward = reward
            msg.end_episode = end_episode
            msg.goal_reached = reached_goal
            self.net_info_pub.publish(msg)
            self.rate.sleep()

            if end_episode == True:
                return         

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
