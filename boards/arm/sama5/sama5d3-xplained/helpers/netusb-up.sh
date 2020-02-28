#!/bin/bash
set -x

# This script can be used to set up the USB Ethernet Gadget interfaces
# on Linux. Tested on Ubuntu 19.10, kernel 5.3.0-24-generic

# USB Ethernet Gadget interface
IF_USB=ens35u2
# external interface
IF_HOST=ens33

IP_NET="10.0.0.0/24"
IP_NETMASK="255.255.255.0"
IP_BROADCAST="10.0.0.255"
IP_HOST="10.0.0.1"
IP_NUTTX="10.0.0.2"

sudo ifconfig $IF_USB up
ifconfig -a
sudo ifconfig $IF_USB add $IP_HOST
sudo ifconfig $IF_USB:0 broadcast $IP_BROADCAST netmask $IP_NETMASK
sudo ip route delete $IP_NET
ip route add $IP_NET dev $IF_USB src $IP_HOST
sudo ip route add $IP_NET dev $IF_USB src $IP_HOST
sudo ip route add $IP_NUTTX/32 dev ens35u2 src $IP_HOST

# nat to allow NuttX to access the internet
sudo iptables -t nat -A POSTROUTING -o $IF_HOST -j MASQUERADE
sudo iptables -A FORWARD -i $IF_HOST -o $IF_USB -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i $IF_USB -o $IF_HOST -j ACCEPT

ip route show

# pinging the nuttx system should work now
#ping -c 1 $IP_NUTTX
