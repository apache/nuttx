#!/usr/bin/env bash
#set -x

# This script must be executed in the NuttX top-level directory.  We'll check..
# the .config file should be located there.

# Note: dfu-programmer needs 'sudo' for its operation. Execute
# this script by prefixing 'sudo' during its invocation.

TOPDIR=`pwd`
if [ ! -f .config ]; then
	echo "There is no configured version of NuttX in this directory."
	echo "  Is '$TOPDIR' the NuttX top level directory?"
	echo "  Has NuttX been configured?"
	exit 1
fi

# The NuttX build system creates a nuttx ELF file, but the batchisp tools
# expects the file to have a .elf extension

if [ ! -f nuttx.elf ]; then
	if [ ! -f nuttx ]; then
		echo "The NuttX ELF file (nuttx or nuttx.elf) does not exist in this directory."
		echo "  Has the NuttX binary been built?"
		exit 1
	fi
	echo "Re-naming nuttx to nuttx.elf"
	mv nuttx nuttx.elf || { echo "mv failed"; exit 1; }
else
	if [ -f nuttx ]; then
		echo "Replacing the old nuttx.elf with the new nuttx file."
		mv nuttx nuttx.elf || { echo "mv failed"; exit 1; }
	fi
fi

DEVICE=at32uc3a0512
ERASE="erase"
FLASH="flash nuttx.hex"
START="start"

# Invoke dfu-programmer

dfu-programmer $DEVICE $ERASE
dfu-programmer $DEVICE $FLASH
dfu-programmer $DEVICE $START
