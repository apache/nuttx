#!/bin/bash
####################################################################################
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
####################################################################################
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

DFU_UTIL=/usr/local/LPCXpresso/bin/dfu-util
NUTTX=/home/patacongo/projects/nuttx/nuttx/trunk/nuttx/nuttx
#${DFU_UTIL} -d nxp:lpc1768 -p 1-3 -c 0 -i 0 -a 0 -D ${NUTTX} -R
${DFU_UTIL} -d nxp:lpc1768 -D ${NUTTX} -R


