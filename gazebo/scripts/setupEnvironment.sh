#!/usr/bin/env bash
#The purpose of this script is to generated the file that the code will use for magic numbers and strings as defined by the user in the "Robot_Config.txt" file.

#This file will also automatically generate directories for each system.
#__________Variables____________
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH="$(dirname "$SCRIPT_PATH")"
cd $SCRIPT_PATH
source Robot_Config.txt

FILE=$WORKSPACE_PATH/src/weaselball_gazebo/include/common.h

#Get longest weaselball structure (Or in this case number of weaselballs)
case $ROBOT_TO_RUN in
    1) NUMBER_OF_WEASELBALLS=1 ;;
    2) NUMBER_OF_WEASELBALLS=2 ;;
    3|4|5) NUMBER_OF_WEASELBALLS=3 ;;
    6|7|8|9|10) NUMBER_OF_WEASELBALLS=4 ;;
esac

#____________ECHO_TO_COMMON_H________


echo "#ifndef _COMMON_H_
#define _COMMON_H_" > $FILE

echo "//PLEASE READ ME!: This file was automatically generated by the scripts in scripts directory. Please edit Robot_Config.txt and then run ./setupEnvironment.sh to change this file!!!!
" >> $FILE

echo "#include<string>
using std::string;
" >> $FILE

echo "// Note: assumes build path is a subdirectory of the source directory and the
// data directory is in the same parent directory as the source directory
static const string DATA_ROOT_PATH= \"$WORKSPACE_PATH/data\";" >> $FILE 


echo "
static const double PI= 3.14159265359;

// path to the motion capture data files on this system
static const string MOCAP_DATA_PATH= \"$WORKSPACE_PATH/data/$MOCAP_DATA_PATH\";

// path to the motion capture data files on this system
static const string VIDEO_DATA_PATH= \"DATA_ROOT_PATH$VIDEO_DATA_PATH\";

// the total number of trials in the experimental sessions
static const int  EXPERIMENTAL_SESSIONS= $EXPERIMENTAL_SESSIONS;

// set on the interval [1,10] to switch between different vicon motion capture
// sessions.
static const int EXPERIMENTAL_SESSION_ID= $EXPERIMENTAL_SESSIONS_ID;

// set at data gathering time through the vicon system.  Constant over all 
// gathered data
static const double VICON_SAMPLE_RATE= $VICON_SAMPLE_RATE;

// led flashes are roughly a single frame, but can span at most two frames.  
// The duration of the signal is also velocity dependent.
static const double BASE_VIDEO_LED_DURATION= $BASE_VIDEO_LED_DURATION;
// A percentage scaling factor for an led event.  This is just an approximation
// factor to have led events occur for a long enough time that it would span 
// two frames.
static const double VIDEO_LED_TIME_FACTOR= $VIDEO_LED_TIME_FACTOR;
" >> $FILE

echo "//Definitions used in state recorder
static const bool RANDOMIZE_STRUCTURES= $RANDOMIZE_STRUCTURES;
static const bool RUN_TRIALS= $RUN_TRIALS;
static const int NUMBER_OF_TRIALS_CYCLES= $NUMBER_OF_TRIALS_CYCLES;
static const int INDIVIDUAL_TRIAL_TIME= $INDIVIDUAL_TRIAL_TIME;
static const bool RANDOMIZE_BALLS= $RANDOMIZE_BALLS;
static const string NAME_OF_WEASELBALLS= \"swarmbot\";
static const string NAME_OF_MOUNTS= \"mount\";
static const string SHELL_STRING= \"shell\";
static const int ROBOT_TO_RUN= $ROBOT_TO_RUN;
static const int LONGEST_WEASELBALL_SEQUENCE= $NUMBER_OF_WEASELBALLS;
static const double DIAMETER_OF_WEASELBALLS= 0.108;
static const int NUMBER_OF_STRUCTURES= 1; //I am keeping this as constant for now since we only want one robot running at a time.
static const int RECORDING_TYPE= $DATA_RECORDING_TYPE;
static const string COLLECTION_PATH= \"$WORKSPACE_PATH/data/collections/\";" >> $FILE
                      

echo "#endif // _COMMON_H_" >> $FILE

./install_new_updates.sh


#--------Create the run file and the corresponding models--------"
cd robot_builder_helper
./create_files.sh $ROBOT_TO_RUN $ENABLE_GRAPHICS
