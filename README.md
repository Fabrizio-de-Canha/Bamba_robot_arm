# Bamba-robot-arm
A pick and place robot arm using Aruco markers and computer vision.

# URDF based on the following robot arm:
https://www.thingiverse.com/thing:34829

# What's needed
- Basic python knowledge
- 1 Arduino
- 1 breadboard
- +- 30 breadboard jumpers
- Raspberry pi with pi-cam (or any other camera)
- see robot construction details in thingiverse link

# Brief Guidelines
OS: its recomended to use Ubuntu 16.04 along with full desktop version of ROS Kinetic. Please see the following link for instructions on installing and using MoveIt and ROS http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

# General approach taken (Note there are multiple aproaches that can be taken)

- Creating the URDF based on the thingiverse robot. (created in SolidWorks)
- Flash Arduino sketch (Makes the Arduino a ROS subscriber node)
- Use the Moveit config package in Rviz to physically control robot.
- Use the camera and OpenCv (Python) to get the position of the object with attached Aruco marker.
- Solve for joints state angles neccessary to reach desired endeffector position from step above.
- Use a Python move group interface to publish joint states generated in a python script to the Arduino node that then moves the robot.
- Later improvement incorporated the use of a custom joint state publisher node written in Python instead of the Rviz joint state publisher (This allowed for smoother and faster motion that can be adjusted).

## side notes
- There may be a large deviation from desired position due to servo motor inaccuracy.
- The model on which this robot was done worked well on the negative x-positions but was unable to reach positions accurately with positive x-coordinates (this may be due to Camera distortion or the servo motor inaccuracy).
- The Configuration package has dissabled collisions between the top 2 links and the ground due to discrepancies between the model and the real robot. 
- The MoveIt IK was not used because it failed to find motion plans for endeffector positions, hence the IK is done in Python.

# Contributions
- JB Cloete
- S Harvey
- L Cremer
- R Jansen
- KG Hellberg
- FG de Canha
- DM Slabbert
- F van Wyk





