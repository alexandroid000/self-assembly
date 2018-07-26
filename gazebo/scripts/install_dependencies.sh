#!/bin/bash

#################################################
#     Install Full ROS Kinetic for Ubuntu 16.04
#################################################

read -n1 -p "Do you need to install ROS? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     # Setup your sources.list
     sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

     # Setup your keys
     sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

     # Install Desktop-Full
     sudo apt-get update
     sudo apt-get install -y ros-kinetic-desktop-full

     # Initialize rosdep
     sudo rosdep init
     rosdep update

     # Setup bash Environment Loading
     echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     source /opt/ros/kinetic/setup.bash
fi

#################################
#     Additional Packages
################################
read -n1 -p "Do you need to install the additional ROS packages? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     # Joystick drivers for robot control
     sudo apt install ros-kinetic-joy ros-kinetic-joystick-drivers ros-kinetic-joy-teleop ros-kinetic-teleop-twist-joy
     # Drivers for robot navigation
     sudo apt install ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-hector-gazebo*

fi
echo

##############################
#   Get Other Dependencies
##############################
read -n1 -p "Do you need to install the additional dependencies for this repo (Ravelin)? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
       
	echo "deb http://ppa.launchpad.net/gwu-positronics/ravelin/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list
        echo "deb-src http://ppa.launchpad.net/gwu-positronics/ravelin/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list
	sudo apt-get update
	sudo apt-get install ravelin
fi
echo "Done!"
