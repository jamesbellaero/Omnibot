# Omnibot
Communicates between ROS on a computer and an Arduino-controlled robot with 4 servos for direction control.

Wireless communication is performed with 2 XBees using xbee-python and xbee-arduino.
To configure XBee radios, download and use XCTU. A decent tutorial is found here: https://learn.sparkfun.com/tutorials/exploring-xbees-and-xctu
If the two radios won't talk to each other, go to the settings tab and hit "Default" then "Write". 

vicon_bridge for ROS: https://github.com/ethz-asl/ros-drivers/tree/master/vicon_bridge
Instructions to install ROSSerial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
