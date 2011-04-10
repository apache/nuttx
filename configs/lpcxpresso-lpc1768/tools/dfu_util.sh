#!/bin/bash
####################################################################################
# dfu_util.sh
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
####################################################################################
# On Linux, the program dfu_utils is included in the Code Red installation:
#
# dfu-util - (C) 2007-2008 by OpenMoko Inc.
# This program is Free Software and has ABSOLUTELY NO WARRANTY
# 
# You need to specify one of -D or -U
# Usage: dfu-util [options] ...
#   -h --help                     Print this help message
#   -V --version                  Print the version number
#   -l --list                     List the currently attached DFU capable USB devices
#   -d --device vendor:product    Specify Vendor/Product ID of DFU device
#   -p --path bus-port. ... .port Specify path to DFU device
#   -c --cfg config_nr            Specify the Configuration of DFU device
#   -i --intf intf_nr             Specify the DFU Interface number
#   -a --alt alt                  Specify the Altsetting of the DFU Interface
#                                 by name or by number
#   -t --transfer-size            Specify the number of bytes per USB Transfer
#   -U --upload file              Read firmware from device into <file>
#   -D --download file            Write firmware from <file> into device
#   -R --reset                    Issue USB Reset signalling once we're finished
#
# Example:
#
# /usr/local/LPCXpresso/bin/Flash$ dfu-util -l
# dfu-util - (C) 2007-2008 by OpenMoko Inc.
# This program is Free Software and has ABSOLUTELY NO WARRANTY
#
# Found Runtime: [0x0471:0xdf55] devnum=3, cfg=0, intf=0, alt=0, name="UNDEFINED"
#
# dmesg:
# [    1.472016] usb 1-3: new high speed USB device using ehci_hcd and address 3
# [    1.604784] usb 1-3: configuration #1 chosen from 1 choice
#
####################################################################################
# In the windows installation, a program called DFUAPP.exe is provided.
#
# DFUAPP.exe /s gui : Will only the DFU app in GUI mode
####################################################################################

# This is the default install location for dfu_util on Linux
DFU_UTIL=/usr/local/LPCXpresso/bin/dfu-util

# This is the default install location for DFUAPP.exe on Windows (note that this
# path could change with the Code Red version number
DFUAPP=/cygdrive/c/nxp/lpcxpresso_3.6/bin/DFUAPP.exe

# The binary to download:
NUTTX=/home/patacongo/projects/nuttx/nuttx/trunk/nuttx/nuttx

#${DFU_UTIL} -d nxp:lpc1768 -p 1-3 -c 0 -i 0 -a 0 -D ${NUTTX} -R
${DFU_UTIL} -d nxp:lpc1768 -D ${NUTTX} -R


