
# Shell script that creates a virtual ethernet port named "eth10"

if(sudo ip link add eth10 type veth peer name eth20 2> /dev/null); then
	sudo ifconfig eth10 up
	echo "eth10 created"
fi

if(sudo ip link add eth11 type veth peer name eth21 2> /dev/null); then
	sudo ifconfig eth11 up
	echo "eth11 created"
fi

if(sudo ip link add eth12 type veth peer name eth22 2> /dev/null); then
	sudo ifconfig eth12 up
	echo "eth12 created"
fi
