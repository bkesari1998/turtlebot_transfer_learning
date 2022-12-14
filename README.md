# turtlebot_transfer_learning

Contains ROS stack for zero-shot transfer experiment using cycle-GAN images for turtlebot navigation.

## Creating a python3 catkin workspace
We followed these instructions to build a catkin workspace in melodic compatible with python3: 
https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html
<br>
We used conda to install the python3 packages necessary to run the ppo network.

## Installing packages
Pull this repository into the /src folder of a catkin workspace. Run `catkin_make` in the root of the workspace to build the package. In the python3 catkin workspace, pull the repository at https://github.com/bkesari1998/turtlebot_transfer_learning_python3 into the /src folder of the workspace. Run `catkin_make` in the root of the python3 workspace to build the package.

## Generating a data set
In `config/dataset.yaml`, assign the ROS parameters for images per episode and total number of episodes. In `config/actions.yaml`, 
assign float values to each of the four primitive move actions. On the turtlebot's computer, launch both `image.launch` and
`turtlebot.launch`. Run `generate_dataset.py` using `rosrun` or a python interpreter.

## Running experiment
In `config/actions.yaml`, 
assign float values to each of the four primitive move action values and assign an integer to the timesteps_per_episode parameter. On the turtelbot's computer, on a ROS melodic catkin workspace built on python 2.7, launch `turtlebot.launch`. Then on a different catkin workspace with python3 support, either on a remote computer or the turtlebot's compute, run `ppo_manager.py` using python3 found in the turtlebot_transfer_learning_python3 package. Make sure to add the correct path to the weights file (i.e weights.pth) to `model_net.py` (line 67). Finally, on the original catkin workspace, run `manager.py`. Make sure to source the python3 catkin_ws in the new terminal.


