#!/usr/bin/env bash

killall gzserver
killall gzclient

#Get Workspace Path
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH="$(dirname "$SCRIPT_PATH")"

source $WORKSPACE_PATH/devel/setup.sh

export GAZEBO_MODEL_PATH=$WORKSPACE_PATH/src/weaselball_description/meshes:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$WORKSPACE_PATH/src/weaselball_gazebo/worlds:$GAZEBO_RESOURCE_PATH

#Not implemented yet
#read -n1 -p "Random initial condition (r) or Constant initial condition (c)?" doit


roslaunch weaselball_gazebo 4T.launch init_cond:=$doit


killall gzserver
killall gzclient
echo "4T Finished"


