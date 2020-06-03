#!/bin/bash
# chmod +x startMAVSDK.sh
# . startMAVSDK.sh

cd /home/odroid/MAVSDK/build/default/src/backend/src
./mavsdk_server -p 50051 serial:///dev/ttyS1:921600
