#!/usr/bin/env bash
#This script uploads all files in the data collection directory on aws
#Takes as input the directory to upload to
aws s3 sync . $1
