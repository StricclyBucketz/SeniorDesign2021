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

Usage:
Mode selection: ... <balance/walk>
Walk: ... <integer_number_of _steps>