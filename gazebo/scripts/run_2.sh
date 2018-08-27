#!/usr/bin/env bash

killall gzserver
killall gzclient

#Get Workspace Path
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH="$(dirname "$SCRIPT_PATH")"
cwd=$PWD
source $WORKSPACE_PATH/scripts/Robot_Config.txt

source $WORKSPACE_PATH/devel/setup.sh
#Upload the test file to make sure everything is connected fine
cd $WORKSPACE_PATH/data/collections
chmod +x upload.sh
sh ./upload.sh s3://vrmsl/2

cd $cwd
export GAZEBO_MODEL_PATH=$WORKSPACE_PATH/src/weaselball_description/meshes:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$WORKSPACE_PATH/src/weaselball_gazebo/worlds:$GAZEBO_RESOURCE_PATH

roslaunch weaselball_gazebo 2.launch init_cond:=$doit

if [ "$UPLOAD_DATA" -eq "1" ]; then
	cd $WORKSPACE_PATH/data/collections
	chmod +x upload.sh
	sh ./upload.sh s3://vrmsl/2
	if [ "$DELETE_AFTER_UPLOAD" -eq "1" ]; then
		rm *.csv
	fi
fi


killall gzserver
killall gzclient
echo "1 Finished"

if [ "$RUNNING_ON_AWS" -eq "1" ]; then
    sudo shutdown now
fi

