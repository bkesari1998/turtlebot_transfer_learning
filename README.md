# turtlebot_transfer_learning

Contains ROS stack for zero-shot transfer experiment using cycle-GAN images for turtlebot navigation.

## Generating a data set
In `config/dataset.yaml`, assign the ROS parameters for images per episode and total number of episodes. In `config/actions.yaml`, 
assign values to each of the four primitive move actions. On the turtlebot's computer, launch both `image.launch` and
`turtlebot.launch`. Run `generate_dataset.py` using `rosrun` or a python interpreter.

## Running experiment
In `config/actions.yaml`, 
assign values to each of the four primitive move actions. On the turtlebot's computer, launch both `image.launch` and
`turtlebot.launch`. Run `manager.py` using `rosrun` or a python interpreter.
