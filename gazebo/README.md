Here are some high level scripts that I include for new users of ROS/Gazebo:

If you are new to ROS/Gazebo all of these scripts may be useful for you, if you are familiar with ROS/Gazebo then "install_dependencies.sh" should be all that you'd need.

install_dependencies.sh - This script install ROS, Gazebo, and other dependencies used by this repo

create_workspace.sh - When using ROS your files should be in a "catkin workspace". This creates the workspace and moves everything to where it is suppsed to be.

install_new_update.sh - Whenever you do a "git pull" or change the code you will need to update the compiled code. Run this to do so.

run_4T.sh - This script launches gazebo and runs the simulator with the weaselball structure in a T formation (using 4 weaselballs)

In other words if you've never used ROS/Gazebo before do the follow:

1) run install_dependencies.sh  
2) run create_workspace.sh   
3) run install_new_updates.sh  
4) run run_4T.sh

Dependencies:  
(See install_dependencies.sh for script I made that can be used to install everything. Available in the scripts directory)  
-Ubuntu 16.04 (This is what I use, you may be able to use a different version/distro but I don't guarantee stability of anything)  
-ROS/Gazebo     
-https://github.com/PositronicsLab/Ravelin
