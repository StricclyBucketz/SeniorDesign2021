University of Nebraska Lincoln
Electrical Engineering Senior Design Spring 2021
Authors:
  Ezra Bailey-Kelly
  Dominic Paul
  Zachary Scott
  Christian White

Project: Walking Robot

The purpose of this project is to develop a bipedal walking robot using AX-12a servos,
3D printed parts, an IMU, and a custom PCB.  The robot has two modes, walking and
balancing to potentially be combined.  The mode is chosen via command line argument.
In walking mode, the desired number of steps to take is issued via the command line.
In balancing mode, no inputs are needed, the robot balances on its own.

The relevant packages for our project included in this repository are:
  
  #Produced and provided openly by ROBOTIS, INC at this Github link: https://github.com/ROBOTIS-GIT/dynamixel-workbench
  dynamixel-workbench
  
  NOTE: Python scripts located in this package in our project were produced by our software team.
  
  #Produced and provided by USC's Navigation Team for the Kaui Labs NavX-Micro IMU at this following Github link:
  #https://github.com/Navigation-Team/ros_navx_micro
  ros_navx_micro

Examples of functions to access different ROS topics and services using Python are provided in /catkin_ws/src/dynamixel-workbench/dynamixel_workbench_controllers/scripts/functions.py

The source libraries are written in C++ and have exmaples for many different functions throughout the packages.

Usage:
Mode selection: ... <balance/walk>
Walk: ... <integer_number_of _steps>

Guide to current implementation:
1. Upon cloning this repository, rename the directory to catkin_ws
2. Run source devel/setup.bash and catkin_make to build packages and dependencies
3. Open a terminal
4. Enter the following commands in order:
  cd catkin_ws
  source devel/setup.bash
  roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
5. Open a new terminal window and enter the following commands:
  cd catkin_ws
  source devel/setup.bash
  rosrun dynamixel_workbench_controllers walkLogic.py   #walkLogic.py can be replaced with any executable algorithm
6. To utilize the IMU, we must initialize the i2c port and start the controller, open a new terminal window
  cd catkin_ws
  source devel/setup.bash
  sudo chmod a+rw /dev/i2c-*
  rosrun navx_micro navx_micro
7. In order to capture runtime data during testing rosbag may implemented in the following manner:
  Open a new terminal
  before running your executable and after establishing your ROS services:
  rosbag record -a
  Ctrl-C when you would like the recording to end, the -a flag records all available topics, if you would like to record specific   topics you will need to use the "rosbag record <topic-names>" convention
8. To extract numerical data for a specific ROS topic from a .bag file, you can use "rostopic echo -b fileName.bag /topic >   fileName.txt"
9. To convert the .txt file to a .csv file for easier import into numerical analysis software, you can use the script provided in our base directory with "./txt2csv.bash fileName.txt fileName.csv"


  
