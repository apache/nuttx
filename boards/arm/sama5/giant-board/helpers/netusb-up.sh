#!/bin/bash
############################################################################
# boards/arm/sama5/giant-board/helpers/netusb-up.sh
#
#  Licensed to the Apache Software Foundation (ASF) under one or more
#  contributor license agreements.  See the NOTICE file distributed with
#  this work for additional information regarding copyright ownership.  The
#  ASF licenses this file to you under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License.  You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
#  License for the specific language governing permissions and limitations
#  under the License.
#
############################################################################/

set -x

# This script can be used to set up the USB Ethernet Gadget interfaces
# on Linux. Tested on Ubuntu 19.10, kernel 5.3.0-24-generic
#
# You may have to change the IF_HOST variable to match your network interface.

# USB Ethernet Gadget interface
IF_USB=usb0
# external interface
IF_HOST=wlp0s20f3

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
ping -c 1 $IP_NUTTX
