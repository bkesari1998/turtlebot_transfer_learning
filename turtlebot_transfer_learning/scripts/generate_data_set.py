# /usr/bin/env python

import rospy
import random

from std_srvs.srv import Empty
from std_msgs.msg import Empty
from turtlebot_transfer_learning_srvs.srv import PrimativeMoveAction


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
        rospy.wait_for_service("/image_saver/save")
        rospy.wait_for_service("/turtlebot_transfer_learning/primative_move_action")

        # Create service clients
        self.image_saver = rospy.ServiceProxy("/image_saver/save", Empty)
        self.move_action = rospy.ServiceProxy("/turtlebot_transfer_learning/primative_move_action", PrimativeMoveAction)

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

        actions = ["forward", "backward", "left", "right"]

        for epidsode in self.total_episodes:
            for image in self.images_per_episode:

                # Take random action
                random_action = random.choice(actions)
                move_req = PrimativeMoveAction()
                move_req.action = random_action
                move_response = self.move_action(move_req)
                rospy.loginfo(move_response.message)

                # Save photo
                self.image_saver(Empty())

            rospy.loginfo("Episode %d complete, waiting for object reset" % epidsode)
            try: 
                rospy.wait_for_message("/turtlebot_transfer_learning/episode_reset", Empty, rospy.Duration(120))
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