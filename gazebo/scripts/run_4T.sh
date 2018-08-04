#!/usr/bin/env bash
source Robot_Config.txt

killall gzserver
killall gzclient

#Get Workspace Path
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH="$(dirname "$SCRIPT_PATH")"

source $WORKSPACE_PATH/devel/setup.sh

export GAZEBO_MODEL_PATH=$WORKSPACE_PATH/src/weaselball_description/meshes:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$WORKSPACE_PATH/src/weaselball_gazebo/worlds:$GAZEBO_RESOURCE_PATH

roslaunch weaselball_gazebo 4T.launch init_cond:=$doit

if [ "$UPLOAD_DATA" -eq "1" ]; then
	cd $WORKSPACE_PATH/data/collections
	chmod +x upload.sh
	sh ./upload.sh s3://vrmsl/4T
fi


killall gzserver
killall gzclient
echo "4T Finished"


