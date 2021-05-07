#!/usr/bin/env bash
#############################################################################
# boards/arm/lpc214x/mcu123-lpc214x/lpc21isp.sh
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
#############################################################################
#set -x

# The path to the built lpc21isp binary

lpc21isp=../lpc2148/lpc21isp/lpc21isp

# lpc21ips options

options="-bin -control -verify"

# The path to the NuttX raw binary format binary

hxfile=nuttx.bin

# The TTY to use for the download

tty=/dev/ttyS0

# The BAUD rate supported by the lpc214x board

baud=38400

# The LPC214X crystal frequency in KHz

osckhz=12000

# Do it!

sudo $lpc21isp $options $hxfile $tty $baud $osckhz
