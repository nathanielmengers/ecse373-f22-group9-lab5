# ecse373-f22-group9-lab5
## Introduction
This package is designed by ECSE373/473 Lab Group 9 from Case Western Reserve University. It serves as foundation to participate (for practice) in the Ariac 2019
competition. The main functions of this package include:
- Receiving generated orders from console
- Retrieve order information such as item type, bin location and desired part pose.
## Dependencies
- ecse_373_ariac
- rviz
- roscpp
- tf2
- actionlib

## Initiating the Competition
You can initiate the competition by calling the following line:

	roslaunch cwru_ecse_373_submission competition.launch

The launch file opens Gazebo and loads the competition environment by calling the launch file for the ecse_373_ariac package. Users may include any optional arguments specified in the ecse_373_ariac when launching the competition. See https://github.com/cwru-eecs-373/ecse_373_ariac/blob/noetic-devel/ecse_373_ariac/README.md for details.

Additionally, the competition file initializes a node which:

- Starts the competition. If the node cannot call the start_competition service, an error is printed. An additional warning informs whether or not the start_competition service returns successfully.
- Display incoming orders
- In a given order and shipment, uses logical cameras to identify which bins contain the requested parts. Also identifies the pose of the parts within the bin.
- Commands the UR10 robot to hover its end effector over each part
- Iterates through the shipments in an order.
- Iterates through each incoming order
