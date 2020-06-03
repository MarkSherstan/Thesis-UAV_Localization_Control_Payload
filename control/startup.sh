#!/bin/bash

echo "Starting MAV-SERVER in tmux session"
tmux new-session -d -s "MAV-SERVER" /home/odroid/Desktop/UAV-Sampling-Control-System/control/startMAVSDK.sh

echo "Entering virtual environment"
workon cv

cd /home/odroid/Desktop/UAV-Sampling-Control-System/control
echo "Enter python main.py to begin"