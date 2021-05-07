#!/usr/bin/env python
############################################################################
# tools/discover.py
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
############################################################################

import array
from socket import AF_INET, SO_BROADCAST, SOCK_DGRAM, SOL_SOCKET, socket, timeout

PORT = 96

DISCOVER_PROTO_ID = 0x99
DISCOVER_ALL = 0xFF  # 0xff means all devices
DISCOVER_REQUEST = 0x01
DISCOVER_RESPONSE = 0x02
DISCOVER_REQUEST_SIZE = 4
DISCOVER_RESPONSE_SIZE = 35


def check_sum(data):
    chksum = 0
    for c in data[:-1]:
        chksum -= c
    return (chksum & 0xFF) == data[-1]


def send_discover(socket):
    cmd = array.array("B", [0] * DISCOVER_REQUEST_SIZE)
    cmd[0] = DISCOVER_PROTO_ID  # Tag for identification of the protocol
    cmd[1] = DISCOVER_REQUEST  # Request command
    cmd[2] = DISCOVER_ALL
    chksum = 0
    for c in cmd[:3]:
        chksum -= c
    cmd[3] = chksum & 0xFF

    socket.sendto(cmd, ("<broadcast>", PORT))


def read_responses(socket):
    res = []
    response = array.array("B", [0] * DISCOVER_RESPONSE_SIZE)
    try:
        while 1:
            size, src = socket.recvfrom_into(response)
            if (
                size == DISCOVER_RESPONSE_SIZE
                and response[0] == DISCOVER_PROTO_ID
                and response[1] == DISCOVER_RESPONSE
                and check_sum(response)
            ):

                dev = {}
                dev["addr"] = src[0]
                dev["descr"] = response[2:-1].tostring().rstrip("\0")
                res.append(dev)

    except timeout:
        return res


if __name__ == "__main__":
    print("Sending discover...")

    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(("0.0.0.0", PORT))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    s.settimeout(1.0)
    send_discover(s)
    devices = read_responses(s)
    socket.close(s)

    print(devices)
