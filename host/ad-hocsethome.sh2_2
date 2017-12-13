#!/bin/bash
#receive input from argument
if [ $# -eq 1 ]
then
sudo ip link set $1 down ; sudo iwconfig $1 mode ad-hoc
sudo iwconfig $1 essid 'rpi network'
sudo iwconfig $1 key 1234567890
sudo ip link set $1 up
sudo ifconfig $1 192.168.0.100/16
else
	echo "please input 1 argument"
	echo "which is device network name"
fi
