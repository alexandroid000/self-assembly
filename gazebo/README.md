Here are some high level scripts that I include for new users of ROS/Gazebo:

If you are new to ROS/Gazebo all of these scripts may be useful for you, if you are familiar with ROS/Gazebo then "install_dependencies.sh", "setupEnvironment.sh", and Robot_Config.txt should be all that you'd need.

install_dependencies.sh - This script install ROS, Gazebo, and other dependencies used by this repo

setupEnvironment.sh - This script will setup the code so that way it matches your local environment. It also is used to change the environment files whenever you change Robot_Config.txt

Robot_Config.txt - Allows the user to set parameters for how they'd like the system to function.

install_new_update.sh - Whenever you do a "git pull" or change the code you will need to update the compiled code. Run this to do so.

run_4T.sh - This script launches gazebo and runs the simulator with the weaselball structure in a T formation (using 4 weaselballs)

In other words to set up all of the code and the environment do the following:

1) run install_dependencies.sh     
2) run install_new_updates.sh  
3) run setupEnvironment.sh  
4) run run_4T.sh

Dependencies:  
(See install_dependencies.sh for script I made that can be used to install everything. Available in the scripts directory)  
-Ubuntu 16.04 (This is what I use, you may be able to use a different version/distro but I don't guarantee stability of anything)  
-ROS/Gazebo     
-https://github.com/PositronicsLab/Ravelin
