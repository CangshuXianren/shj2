#!/bin/zsh

gnome-terminal --tab -x zsh -c "\
sudo chmod 777 /dev/imu; \
roslaunch agx_ws/driver.launch; \
exec zsh"

gnome-terminal --tab -x zsh -c "\
roslaunch velodyne_pointcloud VLP16_points2.launch; \
exec zsh"