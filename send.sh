#!/bin/bash

if [ $# -ne 1 ]; then
    echo "usage - ./send.sh <path>"
    exit 1
fi
if [ ! -d $1 ]; then
    echo "error - invalid path '$1'"
    exit 1
fi
# scp [other options] [source username@IP]:/[directory and file name] [destination username@IP]:/[destination directory]
scp ./$1/*.py pi@10.1.31.230:/home/pi/Robotics2024
#ssh pi@10.1.31.230 "cd /home/pi/Robotics2024 && python p2_base.py; python stop.py"
scp pi@10.1.31.230:/home/pi/Robotics2024/*.csv ./plotLOG
ssh pi@10.1.31.230 rm /home/pi/Robotics2024/*.csv
cd plotLOG
python plotLOG.py