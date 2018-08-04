#ifndef _COMMON_H_
#define _COMMON_H_
//PLEASE READ ME!: This file was automatically generated by the scripts in scripts directory. Please edit Robot_Config.txt and then run ./setupEnvironment.sh to change this file!!!!

// Note: assumes build path is a subdirectory of the source directory and the
// data directory is in the same parent directory as the source directory
#define DATA_ROOT_PATH "/home/justin/Documents/bouncy/self-assembly/gazebo/data"

#define PI 3.14159265359

// path to the motion capture data files on this system
#define MOCAP_DATA_PATH DATA_ROOT_PATH "raw_experiment/vicon/" 

// path to the motion capture data files on this system
#define VIDEO_DATA_PATH DATA_ROOT_PATH "interpreted/signals/"

// the total number of trials in the experimental sessions
#define EXPERIMENTAL_SESSIONS 10

// set on the interval [1,10] to switch between different vicon motion capture
// sessions.
#define EXPERIMENTAL_SESSION_ID 10

// set at data gathering time through the vicon system.  Constant over all 
// gathered data
#define VICON_SAMPLE_RATE 100.0

// led flashes are roughly a single frame, but can span at most two frames.  
// The duration of the signal is also velocity dependent.
#define BASE_VIDEO_LED_DURATION 1.0/30.0
// A percentage scaling factor for an led event.  This is just an approximation
// factor to have led events occur for a long enough time that it would span 
// two frames.
#define VIDEO_LED_TIME_FACTOR 20.0

//Definitions used in state recorder
#define RANDOMIZE_STRUCTURES 1
#define RUN_TRIALS 1
#define NUMBER_OF_TRIALS_CYCLES -1
#define INDIVIDUAL_TRIAL_TIME 300
#define RANDOMIZE_BALLS 1
#define NUMBER_OF_WEASELBALLS 1
#define NUMBER_OF_STRUCTURES 1
#define NAME_OF_WEASELBALLS "swarmbot"
#define NAME_OF_MOUNTS "mount"
#define SHELL_STRING "shell"
#define LONGEST_WEASELBALL_SEQUENCE 1
#define DIAMETER_OF_WEASELBALLS 0.108
#define COLLECTION_PATH "/home/justin/Documents/bouncy/self-assembly/gazebo/data/collections/"
#endif // _COMMON_H_
