#!/bin/bash

# This script must be executed in the NuttX top-level directory

TOPDIR=`pwd`
if [ ! -f .config ]; then
	echo "There is no configured version of NuttX in this directory."
	echo "  Is '$TOPDIR' the NuttX top level directory?"
	echo "  Has NuttX been configured?"
	exit 1
fi
if [! -f nuttx ]; then
	echo "The NuttX ELF file (nuttx) does not exist in this directory."
	echo "  Has the NuttX binary been built?"
	exit 1
fi

DEVICE=at32uc3b0256
HARDWARE=usb
OPERATION="erase f memory flash blankcheck loadbuffer nuttx program verify start reset 0"

batchisp -device $DEVICE -hardware $HARDWAR -operation $OPERATION
