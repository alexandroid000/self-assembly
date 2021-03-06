#ifndef _COMMON_H_
#define _COMMON_H_
//PLEASE READ ME!: This file was automatically generated by the scripts in scripts directory. Please edit Robot_Config.txt and then run ./setupEnvironment.sh to change this file!!!!

#include<string>
using std::string;

// Note: assumes build path is a subdirectory of the source directory and the
// data directory is in the same parent directory as the source directory
static const string DATA_ROOT_PATH= "/home/justin/Documents/self-assembly/gazebo/data";

static const double PI= 3.14159265359;

// path to the motion capture data files on this system
static const string MOCAP_DATA_PATH= "/home/justin/Documents/self-assembly/gazebo/data/raw_experiment/vicon/";

// path to the motion capture data files on this system
static const string VIDEO_DATA_PATH= "DATA_ROOT_PATHinterpreted/signals/";

// the total number of trials in the experimental sessions
static const int  EXPERIMENTAL_SESSIONS= 10;

// set on the interval [1,10] to switch between different vicon motion capture
// sessions.
static const int EXPERIMENTAL_SESSION_ID= 10;

// set at data gathering time through the vicon system.  Constant over all 
// gathered data
static const double VICON_SAMPLE_RATE= 100.0;

// led flashes are roughly a single frame, but can span at most two frames.  
// The duration of the signal is also velocity dependent.
static const double BASE_VIDEO_LED_DURATION= 1.0/30.0;
// A percentage scaling factor for an led event.  This is just an approximation
// factor to have led events occur for a long enough time that it would span 
// two frames.
static const double VIDEO_LED_TIME_FACTOR= 20.0;

//Definitions used in state recorder
static const bool RANDOMIZE_STRUCTURES= 1;
static const bool RUN_TRIALS= 1;
static const int NUMBER_OF_TRIALS_CYCLES= 25;
static const int INDIVIDUAL_TRIAL_TIME= 30;
static const bool RANDOMIZE_BALLS= 1;
static const string NAME_OF_WEASELBALLS= "swarmbot";
static const string NAME_OF_MOUNTS= "mount";
static const string NAME_OF_ENCLOSURE= "weazelenclosure";
static const string SHELL_STRING= "shell";
static const int ROBOT_TO_RUN= 1;
static const int LONGEST_WEASELBALL_SEQUENCE= 1;
static const double DIAMETER_OF_WEASELBALLS= 0.108;
static const int K_LARGE= 1;
static const bool LARGE_ROBOT_GENERATOR= 1;
static const bool ENABLE_ENCLOSURE=0;
static const int NUMBER_OF_STRUCTURES= 1; //I am keeping this as constant for now since we only want one robot running at a time.
static const int RECORDING_TYPE= 1;
static const string COLLECTION_PATH= "/home/justin/Documents/self-assembly/gazebo/data/collections/";
#endif // _COMMON_H_
