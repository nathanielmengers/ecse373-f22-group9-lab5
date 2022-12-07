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

Additionally, the competition file initializes a node to start the competition, manage orders, and control the UR10 manipulator. The specific functionality for each phase of the project is outlined below:

Phase I
- Starts the competition. If the node cannot call the start_competition service, an error is printed. An additional warning informs whether or not the start_competition service returns successfully.
- Display incoming orders
- In a given order and shipment, uses logical cameras to identify which bins contain the requested parts. Also identifies the pose of the parts within the bin.
- Commands the UR10 robot to hover its end effector over each part
- Iterates through the shipments in an order.
- Iterates through each incoming order

Phase II
- Commands the UR10 robot to pick up a single part from a specified order and shipment
- Moves the robot along the linear actuator and deposits the part on an AGV tray
- Submits the order by calling the shipping service

Phase III
- Display an incoming order, which contains two shipments with multiple parts each. Parts may originate in any bin.
- Commands the UR10 robot to grasp a part with the vacuum gripper, then transfer it to the specified AGV tray. Repeat for all parts in the order.

Phase IV (Final Project)
- The manipulator fulfills two orders, each of which contains two shipments with multiple parts each. 
- Parts may originate in any bin and must be deposited on the AGV tray specified for that part/shipment.
- Submits each order with shipping service 
