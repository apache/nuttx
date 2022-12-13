#!/bin/bash

#****************************************************************************
# tools/simhostroute.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
#****************************************************************************

# Helper script to set up host route to NuttX simulator
# and set up IP Tables to allow it to access the
# internet.
#
# This script needs to be run as root.
#
# Note that on Linux you may also have to set kernel capabilities
# on the nuttx executable to allow NuttX to access the tap device:
#
# sudo setcap cap_net_admin+ep ./nuttx

if [ $# != 2 ]; then
  echo "Usage: $0 <interface> <on|off>"
  exit 1
fi

IF_HOST=$1
STATUS=$2

IF_BRIDGE=nuttx0
IP_NET="10.0.1.0/24"
IP_NETMASK="255.255.255.0"
IP_BROADCAST="10.0.0.255"
IP_HOST="10.0.1.1"
IP_NUTTX="10.0.1.2"

if [ "$STATUS" == "on" ]; then
    ip link add $IF_BRIDGE type bridge
    ifconfig $IF_BRIDGE $IP_HOST
    ifconfig $IF_BRIDGE up
    ifconfig -a
    ip addr add $IP_HOST dev $IF_BRIDGE
    ifconfig $IF_BRIDGE netmask $IP_NETMASK
    ip route delete $IP_NET
    ip route add $IP_NET dev $IF_BRIDGE src $IP_HOST
    ip route add $IP_NUTTX/32 dev $IF_BRIDGE src $IP_HOST

    # nat to allow NuttX to access the internet
    iptables -t nat -A POSTROUTING -o $IF_HOST -j MASQUERADE
    iptables -A FORWARD -i $IF_HOST -o $IF_BRIDGE -m state --state RELATED,ESTABLISHED -j ACCEPT
    iptables -A FORWARD -i $IF_BRIDGE -o $IF_HOST -j ACCEPT

    # enable forward to make sure nat works
    sysctl -w net.ipv4.ip_forward=1

    ip route show
else
    ip route delete $IP_NET
    ip route delete $IP_NUTTX/32

    # delete nat rules to clean up
    iptables -t nat -D POSTROUTING -o $IF_HOST -j MASQUERADE
    iptables -D FORWARD -i $IF_HOST -o $IF_BRIDGE -m state --state RELATED,ESTABLISHED -j ACCEPT
    iptables -D FORWARD -i $IF_BRIDGE -o $IF_HOST -j ACCEPT

    ip link delete $IF_BRIDGE type bridge

    ip route show
fi

