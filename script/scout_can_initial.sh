#!/bin/zsh

gnome-terminal --tab -x zsh -c "\
sudo modprobe gs_usb; \
sudo ip link set can0 up type can bitrate 500000; \
exec zsh"