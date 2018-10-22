# Gazebo
This module simulates robot assemblies in 3D using a 3D physics engine.

### Getting Started
##### Dependencies and Executing Program
The code in this folder will install dependencies for ROS and Gazebo, and start
a simulation of our robotic platform.

To set up all of the code and the environment do the following:

1) run `install_dependencies.sh`
2) run `install_new_updates.sh`
3) run `setupEnvironment.sh`
4) run `run_4T.sh`

Code has been tested on Ubuntu 16.04, and Ubuntu is strongly recommended.

If you are new to ROS/Gazebo all of these scripts may be useful for you, if you
are familiar with ROS/Gazebo then `install_dependencies.sh`,
`setupEnvironment.sh`, and `Robot_Config.txt` should be all that you'd need.

Dependencies installed by `install_dependencies.sh`:
- ROS/Gazebo
- https://github.com/PositronicsLab/Ravelin

### Script Descriptions
-   `install_dependencies.sh`: This script install ROS, Gazebo, and other
    dependencies used by this repo
-   `setupEnvironment.sh`: sets up the code so that way it matches your local
    environment. It also is used to change the environment files whenever you
    change `Robot_Config.txt`.
-   `Robot_Config.txt`: Allows the user to set parameters for how they'd like
    the system to function.
-   `install_new_update.sh`: whenever you do a `git pull` or change the code
    you will need to update the compiled code. Run this to do so.
-   `run_4T.sh`: launches gazebo and runs the simulator with the weaselball
    structure in a T formation (using 4 weaselballs)
