#!/bin/bash

helpFunction(){
	echo ""
	echo "Usage: -a address -s script path"
	echo -e "\t-a Address of the port used to comunicate with an instance of PX4"
	echo -e "\t-h T/F"
	exit 1; #Exit after printing help
}

echo "Starting navigation script"

scriptpath=/home/incarsaron/PycharmProjects/Drone_Project/FollowMe_Implementation;

#Parse for inputs
while getopts "a:s:" flag
do
	case "${flag}" in
		a) address=${OPTARG};;
		s) scriptpath=${OPTARG};;
		?) helpFunction;;
	esac
done

echo "Address: $address";
echo "ScriptPath: $scriptpath";

python3 $scriptpath/main.py 
