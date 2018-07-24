Here are some high level scripts that I include for new users of ROS/Gazebo:

install_dependencies.sh - This script install ROS, Gazebo, and other dependencies used by this repo

create_workspace.sh - When using ROS your files should be in a "catkin workspace". This creates the workspace and moves everything to where it is suppsed to be. (TODO)

run_4T.sh - This script launches gazebo and runs the simulator with the weaselball structure in a T formation (using 4 weaselballs)

In other words if you've never used ROS/Gazebo before do the follow:

1) run install_dependencies.sh
2) run create_workspace.sh (TODO)
3) run run_4T.sh

Dependencies:
(A script I made that can be used to install everything is available in the scripts directory)
Ubuntu 16.04 (This is what I use, you may be able to use a different version/distro but I dont gaurantee stability of anything)
ROS/Gazebo : See script for auto isntall (COMING SOON)
https://github.com/PositronicsLab/Ravelin
