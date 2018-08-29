#!/usr/bin/env bash


ROBOT_ID=$1
ENABLE_GRAPHICS=$2

DIR=../../src/weaselball_description/meshes/${ROBOT_ID}

re='^[0-9]+$'
if ! [[ $ROBOT_ID =~ $re ]] ; then
   echo "error: Not a number" >&2; exit 1
fi
python build_robot_files.py $ROBOT_ID $ENABLE_GRAPHICS
chmod +x run.sh

if [ ! -d $DIR ]; then
	mkdir $DIR 
fi


mv run.sh ../
mv model.sdf ../../src/weaselball_description/meshes/${ROBOT_ID}
mv model.config ../../src/weaselball_description/meshes/${ROBOT_ID}
mv one_${ROBOT_ID}_mount.launch ../../src/weaselball_gazebo/launch/include 
mv ${ROBOT_ID}.launch ../../src/weaselball_gazebo/launch 
