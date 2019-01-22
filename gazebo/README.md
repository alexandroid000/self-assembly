# WeaselBall Gazebo Simulation
This module simulates weaselbot robot assemblies in 3D using a 3D physics engine through Gazebo. This allows for scalable data collection for analysis of non deterministic robot behavior. This module offers a streamlined way to collect data on weaselball behavior. Currently this simulator does not support multiple robot structure behavior.

## Getting Started
#### Dependencies and Executing Program
The code in this folder will install dependencies for ROS and Gazebo, and start
a simulation of our robotic platform.

To set up all of the code and the environment do the following:

1) run `install_dependencies.sh`
2) run `install_new_updates.sh`
3) run `setupEnvironment.sh`
4) run `run.sh`

Code in this folder has been tested on Ubuntu 16.04.


Dependencies installed by `install_dependencies.sh`:
- ROS/Gazebo
- https://github.com/PositronicsLab/Ravelin

Data from the simulation will be stored in self-assembly/gazebo/data/collections. A script exists in this folder for the `run.sh` script to upload the data to aws if needed. Testupload is a log that is used to show when new simulation data has been written to AWS.

## Script Descriptions
-   `install_dependencies.sh`: This script install ROS, Gazebo, and other
    dependencies used by this repo
-   `setupEnvironment.sh`: Changes the Gazebo environment files whenever you
    change `Robot_Config.txt`. 
-   `Robot_Config.txt`: Allows the user to set parameters for how they'd like
    the system to function.
-   `install_new_update.sh`: Whenever you do a `git pull` or change the code
    you will need to update the code. Run this to do so.
-   `run.sh`: Launches gazebo and runs the simulator with the weaselballs.

## Robot_Config.txt Settings
-   MOCAP_DATA_PATH, VIDEO_DATA_PATH, EXPERIMENTAL_SESSION, EXXPERIMENTAL_SESSIONS_ID, VICON_SAMPLE_RATE, BASE_VIDEO_LED_DURATION, and VIDEO_LED_TIME_FACTOR are global variables used in the simulation this simulator was based off of, and has no reason to be changed.

-   `UPLOAD_DATA`: Allows for AWS data file to be uploaded after simulation finishes.
-   `DELETE_AFTER_UPLOAD`: Deletes the local datalog file after running the simulation.
-   `ENABLE_GRAPHICS`: Selects whether the graphic mode of gazebo is ran. If "False", gazebo will be run without any graphics.
-   `DATA_RECORDING_TYPE`: Selects whether the verbose or mininal data is collected. See the data collection section for more information.
-   `ROBOT_TO_RUN`: Selects which robot configuration to run.
-   `NUMBER_OF_WEASELBALLS`: Tells the simulator how many weaselballs will be in the simulation (This will soon be phased out since it can be implied from "ROBOT_TO_RUN").
-   `RANDOMIZE_BALLS`: Randomly rotates the weaselballs
-   `RANDOMIZE_STRUCTURES`: Randomizes the placement and orientation of the weaselball structures.
-   `RUN_TRAILS`: Trials are a testing period where the simulator is reset. This allows for a more diverse data collection where the tested can have the weaselballs start off in random orientations and positions.
-   `NUMBER_OF_TRIALS_CYCLES`: Determines the amount of trials to be performed before turning off the simulator.
-   `INDIVIDUAL_TRIAL_TIME`: How long (in simulator secconds) we want each trial to run for. The simulator may run faster or slower than real life, so using simulator seconds is more consistant.
-   `RUNNING_ON_AWS`: Shuts down the computer after finishing.
-   `NUMBER_OF_STRUCTURES`: How many structures the simulator has to keep track of. Currently this shouldn't be changed as multi-weaselball structures are not fully tested in the simulator.

## Setting up connection with AWS
I am assuming that you already have an AWS account set up, and now just need to configure everything so that the data gets uploaded to the S3 buckets.
The only thing that needs to be done is set up `aws configure`. This will look like as follows

aws configure<br/>
AWS Access Key ID [None]: AKIAIOSFODNN7EXAMPLE<br/>
AWS Secret Access Key [None]: wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY<br/>
Default region name [None]: us-east-2<br/>
Default output format [None]:<br/>

If you need the Access Key ID or Secret Accesss Key here is a good tutorial on obtaining them - https://www.youtube.com/watch?v=H_YNNcM47Wk

The default region is wherever you chose to have your s3 data stored.

## Data Collection
This simulator offers 2 data collections, verbose and minimal. 

Verbose offers the following data - "Time,ID,Mount_X,Mount_Y,Mount_Yaw,Pos_x,Pos_y,Pos_z,Yaw,Pitch,Roll,Linear_Veloci    ty_X_World,Linear_Velocity_Y_World,Linear_Velocity_Z_World,Linear_Acceleration_X_World,Linear_Acceleration_Y_Wo    rld,Linear_Acceleration_Z_World,Rotational_Velocity_X_World,Rotational_Velocity_Y_World,Rotational_Velocity_Z_W    orld,Rotational_Acceleration_X_World,Rotational_Acceleration_Y_World,Rotational_Acceleration_Z_World,Linear_Vel    ocity_X_Relative,Linear_Velocity_Y_Relative,Linear_Velocity_Z_Relative,Linear_Acceleration_X_Relative,Linear_Ac    celeration_Y_Relative,Linear_Acceleration_Z_Relative_Relative,Rotational_Velocity_X_Relative,Rotational_Velocit    y_Y_Relative,Rotational_Velocity_Z_Relative,Rotational_Acceleration_X_Relative,Rotational_Acceleration_Y_Relati    ve,Rotational_Acceleration_Z_Relative,ResetID,checkCorrectness"

Minial offers the following data -
"Time,ID,X,Y,Yaw,ResetID,checkCorrectness,NumberOfWalls"

## Problems I have ran into while installing
There seems to be some weird interactions with Gazebo and Anaconda. I beleive this happens because rospkg (a python library) gets installed in the /usr/ space instead wherever your anaconda environment is.

### error: ROSPKG isnt installed
This happens when PYTHONPATH doesn't contain the path to the global python libraries. This can be fixed with export `PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages`
