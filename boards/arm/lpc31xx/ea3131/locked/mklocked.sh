#!/usr/bin/env bash
###########################################################################
# boards/arm/lpc31xx/ea3131/locked/mklocked.sh
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

#set -x

############################################################################
# Arguments
############################################################################

USAGE="$0 <nuttx-dir>"

TOPDIR="$1"
CONFIG="$TOPDIR/.config"
if [ -z "$TOPDIR" ]; then
	echo "Missing Argument"
	echo $USAGE
	exit 1
fi
if [ ! -d "$TOPDIR" ]; then
	echo "NuttX directory does not exist: $TOPDIR"
	echo $USAGE
	exit 1
fi
if [ ! -f "$CONFIG" ]; then
	echo "Configuration file not found: $CONFIG"
	exit 1
fi

############################################################################
# Functions
############################################################################

function checkconfig () {
	CONFIGLINE=`cat "$CONFIG" | grep "$1="`
	if [ -z "$CONFIGLINE" ]; then
		echo "n"
	fi
	if [ "X${CONFIGLINE}" = "X${1}=y" ]; then
		echo "y"
	else
		echo "n"
	fi
}

function checkzero () {
	CONFIGLINE=`cat "$CONFIG" | grep "$1="`
	if [ -z "$CONFIGLINE" ]; then
		echo "y"
	fi
	if [ "X${CONFIGLINE}" = "X${1}=0" ]; then
		echo "y"
	else
		echo "n"
	fi
}

############################################################################
# Interrupt Handlers
############################################################################
#
# All interrupt handlers must be forced to lie in the locked .text region
#
# These are the vector entry points (only one is really needed since they
# are all in the same file). These should drag in all of the vector
# dispatching logic.
#

rm -f ld-locked.inc
echo "EXTERN(arm_vectorswi)" >>ld-locked.inc
echo "EXTERN(arm_vectordata)" >>ld-locked.inc
echo "EXTERN(arm_vectorprefetch)" >>ld-locked.inc
echo "EXTERN(arm_vectorundefinsn)" >>ld-locked.inc
echo "EXTERN(arm_vectorfiq)" >>ld-locked.inc
echo "EXTERN(arm_vectorirq)" >>ld-locked.inc
echo "EXTERN(arm_vectoraddrexcptn)" >>ld-locked.inc

#
# These are the initialization entry points of all device drivers that
# handle interrupts.  We really want to include as little as possible --
# ideally just the interrupt handler itself, but that is not usually
# possible.
#
# Of course, this list must be extended as interrupt handlers are added.

echo "EXTERN(up_timer_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_LPC31_UART)
if [ "$answer" = y ]; then
	echo "EXTERN(arm_earlyserialinit)" >>ld-locked.inc
fi

# xyz_i2cbus_initialize -- Not conditioned on anything

answer=$(checkconfig CONFIG_USBDEV)
if [ "$answer" = y ]; then
	echo "EXTERN(arm_usbinitialize)" >>ld-locked.inc
fi

############################################################################
# Initialization logic
############################################################################
# All initialization logic must be in memory because it must execute before
# the page fill worker thread is started.  Ideally this would be in some
# region that is mapped initially, but then unmapped after initialization
# is complete -- effectively freeing the memory used for the 1-time
# initialization code.  That optimization has not yet been made and, as
# consequence, the 1-time initialization code takes up precious memory
# in the locked memory region.
#
# arm_boot is a low-level initialization function called by __start:

echo "EXTERN(arm_boot)" >>ld-locked.inc

# All of the initialization functions that are called by nx_start up to
# the point where the page fill worker thread is started must also be
# included in the locked text section (at least for now)

answer=$(checkzero CONFIG_TASK_NAME_SIZE)
if [ "$answer" = n ]; then
	echo "EXTERN(arm_boot)" >>ld-locked.inc
fi

echo "EXTERN(dq_addfirst)" >>ld-locked.inc
echo "EXTERN(up_initial_state)" >>ld-locked.inc
echo "EXTERN(up_allocate_heap)" >>ld-locked.inc
echo "EXTERN(mm_initialize)" >>ld-locked.inc
echo "EXTERN(irq_initialize)" >>ld-locked.inc
echo "EXTERN(wd_initialize)" >>ld-locked.inc
echo "EXTERN(clock_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_DISABLE_POSIX_TIMERS)
if [ "$answer" = n ]; then
	echo "EXTERN(timer_initialize)" >>ld-locked.inc
fi

echo "EXTERN(nxsig_initialize)" >>ld-locked.inc
echo "EXTERN(sem_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_DISABLE_MQUEUE)
if [ "$answer" = n ]; then
	echo "EXTERN(nxmq_initialize)" >>ld-locked.inc
fi

answer=$(checkconfig CONFIG_DISABLE_PTHREAD)
if [ "$answer" = n ]; then
	echo "EXTERN(pthread_initialize)" >>ld-locked.inc
fi

echo "EXTERN(fs_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_NET)
if [ "$answer" = y ]; then
	echo "EXTERN(net_initialize)" >>ld-locked.inc
fi

echo "EXTERN(up_initialize)" >>ld-locked.inc
echo "EXTERN(sched_setupidlefiles)" >>ld-locked.inc
echo "EXTERN(task_create)" >>ld-locked.inc

############################################################################
# Idle Loop
############################################################################
#
# The IDLE loop must be forced to lie in the locked .text region.

echo "EXTERN(nx_start)" >>ld-locked.inc
echo "EXTERN(up_idle)" >>ld-locked.inc

############################################################################
# PG Fill Worker Thread
############################################################################
#
# All of the page fill worker thread must be in the locked .text region.

echo "EXTERN(pg_worker)" >>ld-locked.inc
