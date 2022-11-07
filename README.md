# ecse373-f22-group9-lab5
## Introduction
This package is designed by ECSE373/473 Lab Group 9 from Case Western Reserve Univeristy. It serves as foundation for participate (for practice) in the Ariac 2019
competition. The main funtions of this package include:
- Receiving generated orders from console
- Retrieve order information such as item type, bin location and desired part pose. 
## Dependencies
- ecse_373_ariac
- rviz
- roscpp
## How does this work
By launching this package, it will start the competition. The cpp file will subscribe to order published in given topic, and extract the information required 
then print them on the screen. The program will give proper info, warning, or error if the competion failed to start or was started unproperly. Also the program 
subscribed to the logic cameras built in the environment, which monitors the bins and the trays. This will enable the program to determine whether the part is 
posed correctly or if the part is there at all. 

