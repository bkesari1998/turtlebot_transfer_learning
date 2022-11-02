#!/usr/bin/env python

import rospy

from turtlebot_transfer_learning_srvs.srv import PrimitiveAction
from geometry_msgs.msg import Twist


class PrimativeVelocityAction(object):
    def __init__(self):
        """
        Initializes ROS node containing primitive move service.
        """

        # Initialize ROS node
        rospy.init_node("primative_velocity_actions")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("primitive_velocity_actions node active")

        # Primative move actions are forward, backward, counter_clockwise, clockwise
        # Get values for move actions from ros param

        self.max_velocity_values = {
            "linear": 0.5,
            "angular": 1
        }

        self.primative_action_values = {
            "forward": 0.05,
            "backward": -0.05,
            "counter_clockwise": 0.1,
            "clockwise": -0.1,
            "continue": 0,
            "stop": 0
        }

        try:
            param_primative_action_values = rospy.get_param(
                "/turtlebot_transfer_learning/primative_velocity_actions"
            )
            if type(param_primative_action_values) != dict:
                raise TypeError

            if (
                set(param_primative_action_values.keys())
                != set(self.primative_action_values.keys())
            ):
                raise ValueError

            self.primative_action_values = param_primative_action_values
        except rospy.ROSException:
            rospy.logwarn(
                "Parameter server reported an error. Resorting to in-node default primative action values"
            )
        except KeyError:
            rospy.logwarn(
                "Value not set and default not given for '/turtlebot_transfer_learning/primative_velocity_actions'. Resorting to in-node default primative action values"
            )
        except TypeError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/primative_velocity_actions' is not of type 'dict'. Resorting to in-node primative action values."
            )
        except ValueError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/primative_velocity_actions' contains unexpected keys. Resorting to in-node primative action values"
            )

        try:
            param_max_velocity_values = rospy.get_param(
                "/turtlebot_transfer_learning/max_velocity_values"
            )
            if type(param_max_velocity_values) != dict:
                raise TypeError
            if (
                set(param_primative_action_values.keys())
                != set(self.max_velocity_values.keys())
            ): 
                raise ValueError
            self.max_velocity_values = param_max_velocity_values
        except rospy.ROSException:
            rospy.logwarn(
                "Parameter server reported an error. Resorting to in-node default max velocity values"
            )
        except TypeError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/max_velocity_values' is not of type 'dict'. Resorting to in-node max velocity values."
            )
        except KeyError:
            rospy.logwarn(
                "Value not set and default not given for '/turtlebot_transfer_learning/max_velocity_values'. Resorting to in-node default max velocity values"
            )
        except ValueError:
            rospy.logwarn(
                "Value of param '/turtlebot_transfer_learning/max_velocity_values' contains unexpected keys. Resorting to in-node max_velocity_values"
            )

        # Initialize service
        self.primative_move_srv = rospy.Service(
            "/turtlebot_transfer_learning/primative_velocity_action",
            PrimitiveAction,
            self.velocity_action_srv_handler,
        )
        rospy.loginfo("/turtlebot_transfer_learning/primative_velocity_action service active")

        # Initialize publisher
        self.linear = 0
        self.angular = 0
        self.cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.spin()

    def velocity_action_srv_handler(self, req):
        print("In handler")
        move_cmd = Twist()
        if req.action == "continue":
            pass
        elif req.action == "forward":
            print("in forward")
            if (self.linear < self.max_velocity_values["linear"]):
                self.linear += self.primative_action_values["forward"]
        elif req.action == "backward":
            if (self.linear > -self.max_velocity_values["linear"]):
                self.linear -= self.primative_action_values["backward"]
        elif req.action == "counter_clockwise":
            if (self.angular < self.max_velocity_values["angular"]):
                self.angular += self.primative_action_values["forward"]
        elif req.action == "clockwise":
            if (self.angular > -self.max_velocity_values["angular"]):
                self.angular -= self.primative_action_values["forward"]
        else:
            self.linear = 0
            self.angular = 0
        
        move_cmd.linear.x = self.linear
        move_cmd.angular.z = self.angular
        self.cmd_vel.publish(move_cmd)
        print("published")
        self.rate.sleep()

        return True, "published velocity command"

    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("primitive_move_action node shutdown")

if __name__ == '__main__':
    PrimativeVelocityAction()