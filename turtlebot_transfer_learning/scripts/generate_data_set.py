#!/usr/bin/env python

import rospy
import random

from std_srvs.srv import Empty as srv_empty
from std_msgs.msg import Empty
from turtlebot_transfer_learning_srvs.srv import PrimitiveAction


class GenerateData(object):
    def __init__(self):
        """
        Initializes ROS node which generates image data set from turtlebot.
        """

        # Initialize node
        rospy.init_node("generate_data")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("generate_data node active")

        # Wait for services
        # rospy.loginfo("waiting for camera_controller/save")
        rospy.wait_for_service("/camera_controller/save")
        rospy.wait_for_service("/turtlebot_transfer_learning/primative_velocity_actions")

        # Create service clients
        # self.image_saver = rospy.ServiceProxy("/camera_controller/save", srv_empty)
        self.velocity_action = rospy.ServiceProxy("/turtlebot_transfer_learning/primative_velocity_actions", PrimitiveAction)

        # Get data set params
        self.images_per_episode = 100
        self.total_episodes = 40

        try:
            images_per_episode = rospy.get_param(
                "/turtlebot_transfer_learning/images_per_episode"
            )
            total_episodes = rospy.get_param(
                "/turtlebot_transfer_learning/total_episodes"
            )

            if type(images_per_episode) != int and type(images_per_episode) != int:
                raise TypeError

            self.images_per_episode = images_per_episode
            self.total_episodes = total_episodes

        except rospy.ROSException:
            rospy.logwarn(
                "Parameter server reported an error. Resorting to in-node default data set parameters"
            )
        except KeyError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/images_per_episode' or '/turtlebot_transfer_learning/total_episodes' is not set and default not given. Resorting to in-node default data set parameters"
            )
        except TypeError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/images_per_episode' or '/turtlebot_transfer_learning/total_episodes' is not of type 'int'. Resorting to in-node default data set parameters"
            )

        # Generate data set
        self.generate_data_set()
    
    def generate_data_set(self):

        actions = ["forward", "backward", "clockwise", "counter_clockwise", "continue", "stop"]

        for epidsode in range(self.total_episodes):
            for i in range(self.images_per_episode):
                
                rospy.loginfo("Step: " + str(i))

                # Take random action
                random_action = random.choice(actions)
                move_response = self.velocity_action(random_action)
                rospy.loginfo(move_response.message)
                rospy.sleep(0.5)

                # Save photo
                # self.image_saver()

            rospy.loginfo("Episode %d complete, waiting for object reset" % epidsode)
            try: 
                rospy.wait_for_message("/turtlebot_transfer_learning/episode_reset", Empty, rospy.Duration(120))
                rospy.loginfo("Continuing")
            except rospy.ROSException:
                rospy.logwarn("Reset not recieved, continuing with data collection")
        
        rospy.loginfo("Data collection complete")

    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("generate_data node shutdown")

if __name__ == "__main__":
    GenerateData()