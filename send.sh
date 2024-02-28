#!/bin/bash
# scp [other options] [source username@IP]:/[directory and file name] [destination username@IP]:/[destination directory]
scp ./src/* pi@10.1.31.230:/home/pi/Robotics2024
ssh pi@10.1.31.230 "cd /home/pi/Robotics2024 && python p2_base.py; python stop.py"
scp pi@10.1.31.230:/home/pi/Robotics2024/*.csv ./logs
