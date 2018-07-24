#!/usr/bin/env bash

read -n1 -p "Please have this 'self-assembly' project located where you'd like to have the workspace installed. When you have done that press y" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
	cd ..
	cd src
	catkin_init_workspace
echo "Done creating the workspace"


