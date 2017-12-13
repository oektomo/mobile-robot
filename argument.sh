#!/bin/bash
if [ $# -eq 0 ]
	then
	echo "no arguments supplied"
	echo "please input network device"
fi
if [ $# -eq 1 ]
	then
	echo $1
	ifconfig $1
fi
