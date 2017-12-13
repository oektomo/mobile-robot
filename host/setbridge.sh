#!/bin/bash
# trying to do 2 argument
if [ $# -eq 2 ]
then
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
ifconfig
else
echo "please insert 2 argument"
echo "example:"
echo "./setbridge.sh nic_source nic_destination"
echo "cheers"
fi
