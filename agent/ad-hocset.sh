#!/bin/bash
sudo service ifplugd stop
sudo service networking restart
#sudo ip link set wlan0 down ; sudo iwconfig wlan0 mode ad-hoc
#sudo iwconfig wlan0 essid 'rpi network'
#sudo iwconfig wlan0 key 1234567890
#sudo ip link set wlan0 up
#sudo ifconfig wlan0 192.168.0.103/16
#sudo route add default gw 192.168.0.100
