# turtlebot_transfer_learning

Contains ROS stack for zero-shot transfer experiment using cycle-GAN images for turtlebot navigation.

## Generating a data set
In `config/dataset.yaml`, assign the ROS parameters for images per episode and total number of episodes. In `config/actions.yaml`, 
assign values to each of the four primitive move actions. On the turtlebot's computer, launch both `image.launch` and
`turtlebot.launch`. Run `generate_dataset.py` using `rosrun` or a python interpreter.

## Running experiment
In `config/actions.yaml`, 
assign values to each of the four primitive move actions. On the turtelbot's computer, on a ROS melodic catkin workspace built on python 2.7, launch `turtlebot.launch` and `image.launch`. Then on a different catkin workspace with python3 support, either on a remote computer or the turtlebot's compute, run `ppo_manager.py` using python3. Finally, on the original catkin workspace, run `manager.py`. Make sure to source the python3 catkin_ws in the new terminal.
