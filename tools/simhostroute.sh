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
IPv4_HOST="10.0.1.1/24"
IPv6_HOST="fc00::1/112"
IPv6_ENABLE=true

call_all() {
  FUNC=$1

  IPTABLES="iptables"
  IP_HOST=$IPv4_HOST

  # call function
  $FUNC

  # enable forward to make sure nat works
  sysctl -w net.ipv4.ip_forward=1

  if [ "$IPv6_ENABLE" == "true" ]; then
    IPTABLES="ip6tables"
    IP_HOST=$IPv6_HOST

    # call function
    $FUNC

    # enable forward to make sure nat works
    sysctl -w net.ipv6.conf.all.forwarding=1
  fi
}

net_on() {
  # add address to the bridge, with CIDR specified, netmask/route will be automatically added.
  ip addr add $IP_HOST dev $IF_BRIDGE

  # nat to allow NuttX to access the internet
  $IPTABLES -t nat -A POSTROUTING -o $IF_HOST -j MASQUERADE
  $IPTABLES -A FORWARD -i $IF_HOST -o $IF_BRIDGE -m state --state RELATED,ESTABLISHED -j ACCEPT
  $IPTABLES -A FORWARD -i $IF_BRIDGE -o $IF_HOST -j ACCEPT
}

net_off() {
  ip addr del $IP_HOST dev $IF_BRIDGE

  # delete nat rules to clean up
  $IPTABLES -t nat -D POSTROUTING -o $IF_HOST -j MASQUERADE
  $IPTABLES -D FORWARD -i $IF_HOST -o $IF_BRIDGE -m state --state RELATED,ESTABLISHED -j ACCEPT
  $IPTABLES -D FORWARD -i $IF_BRIDGE -o $IF_HOST -j ACCEPT
}

# remove all configs first to avoid double configure
call_all net_off

if [ "$STATUS" == "on" ]; then
    ip link add $IF_BRIDGE type bridge
    ifconfig $IF_BRIDGE up
    ifconfig -a

    call_all net_on
else
    ip link delete $IF_BRIDGE type bridge
fi

ip route show
ip -6 route show
