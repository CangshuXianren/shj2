#!/bin/zsh

gnome-terminal --tab -x zsh -c "\
sudo chmod 777 /dev/xsens; \
roslaunch agx_ws/driver4.launch; \
exec zsh"

gnome-terminal --tab -x zsh -c "\
roslaunch agx_ws/src/rslidar_sdk/rslidar_sdk/launch/start4.launch; \
exec zsh"