#!/usr/bin/env bash


ROBOT_ID=$1

python build_robot_files.py $ROBOT_ID
chmod +x run_$ROBOT_ID.sh
mv run_$ROBOT_ID.sh ../
mkdir ../../src/weaselball_description/meshes/${ROBOT_ID}
mv model.sdf ../../src/weaselball_description/meshes/${ROBOT_ID}
mv model.config ../../src/weaselball_description/meshes/${ROBOT_ID}
mv one_${ROBOT_ID}_mount.launch ../../src/weaselball_gazebo/launch/include 
mv ${ROBOT_ID}.launch ../../src/weaselball_gazebo/launch 
