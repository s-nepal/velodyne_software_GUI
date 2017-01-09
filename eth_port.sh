#!/bin/bash

#virtual ethernet ports
#eth10

create_vethernet_port()
{
	echo -n "Enter name of ethernet port (eg., eth0, lidar1) : "
	read port
	sudo /sbin/ip link add dummy0 type dummy 2> /dev/null
	if(sudo ip link set name $port dev dummy0 2> /dev/null); then
		sudo ip link set $port up
		#if(sudo ip link set $port up); then
		echo "Successfully created ethernet port $port !!!"
		#fi
	else
		if(ip link show $port 2> /dev/null 1> /dev/null);then
			echo "$port already exists"
		fi
	fi
}

delete_vethernet_port()
{
	echo -n "Enter name of ethernet port : "
	read port
	if(sudo ip link del $port); then
		echo "Successfully deleted ethernet port $port"
	fi
}

display_info()
{
	ip link show
}

userInput=
until [ "$userInput" = "0" ]; do
	echo "
PROGRAM MENU
	1 - create a virtual ethernet port
	2 - delete a virtual ethernet port
	0 - exit"
		echo -n "Enter number : "
		read num
		case $num in
			1 ) create_vethernet_port
				sudo sleep 1 
				;;
			2 ) delete_vethernet_port
				sudo sleep 1
				;;
			3 ) display_info
				sudo sleep 1
				;;
			0 ) exit
				;;
			* ) echo "Please enter proper number"
				sudo sleep 1
				;;
		esac
done
