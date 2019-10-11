#!/usr/bin/env bash
# refresh.sh
#
#   Copyright (C) 2014, 2016-2017, 2019 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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

WD=`test -d ${0%/*} && cd ${0%/*}; pwd`
TOPDIR="${WD}/.."

USAGE="USAGE: $0 [options] <board>:<config>"
ADVICE="Try '$0 --help' for more information"

unset CONFIG
silent=n
defaults=n
prompt=y

while [ ! -z "$1" ]; do
    case $1 in
    --debug )
        set -x
        ;;
    --silent )
        silent=y
        defaults=y
        prompt=n
        ;;
    --prompt )
        prompt=y
        ;;
    --defaults )
        defaults=y
        ;;
    --help )
        echo "$0 is a tool for refreshing board configurations"
        echo ""
        echo $USAGE
        echo ""
        echo "Where [options] include:"
        echo "  --debug"
        echo "     Enable script debug"
        echo "  --silent"
        echo "     Update board configuration without interaction.  Implies --defaults."
        echo "     Assumes no prompt for save.  Use --silent --prompt to prompt before saving."
        echo "  --prompt"
        echo "     Prompt before updating and overwriting the defconfig file.  Default is to"
        echo "     prompt unless --silent"
        echo "  --defaults"
        echo "     Do not prompt for new default selections; accept all recommended default values"
        echo "  --help"
        echo "     Show this help message and exit"
        echo "  <board>"
        echo "     The board directory under nuttx/boards"
        echo "  <config>"
        echo "     The board configuration directory under nuttx/boards/<board>/configs"
        exit 0
        ;;
    * )
        CONFIG=$1
        break
        ;;
    esac
    shift
done

# Get the board configuration

if [ -z "${CONFIG}" ]; then
  echo "ERROR: No configuration provided"
  echo $USAGE
  echo $ADVICE
  exit 1
fi

CONFIGSUBDIR=`echo ${CONFIG} | cut -s -d':' -f2`
if [ -z "${CONFIGSUBDIR}" ]; then
  CONFIGSUBDIR=`echo ${CONFIG} | cut -s -d'/' -f2`
  if [ -z "${CONFIGSUBDIR}" ]; then
    echo "ERROR: Malformed configuration: ${CONFIG}"
    echo $USAGE
    echo $ADVICE
    exit 1
  else
    BOARDSUBDIR=`echo ${CONFIG} | cut -d'/' -f1`
  fi
else
  BOARDSUBDIR=`echo ${CONFIG} | cut -d':' -f1`
fi

# Detect the architecture of this board.

ARCHLIST="arm avr hc mips misoc or1k renesas risc-v sim x86 xtensa z16 z80"
ARCHLIST="arm avr hc mips misoc or1k renesas risc-v sim x86 xtensa z16 z80"
CHIPLIST="a1x am335x c5471 cxd56xx dm320 efm32 imx6 imxrt kinetis kl lc823450
  lpc17xx_40xx lpc214x lpc2378 lpc31xx lpc43xx lpc54xx max326xx moxart nrf52
  nuc1xx rx65n s32k1xx sam34 sama5 samd2l2 samd5e5 samv7 stm32 stm32f0l0g0 stm32f7 stm32h7
  stm32l4 str71x tiva tms570 xmc4 at32uc3 at90usb atmega mcs92s12ne64 pic32mx
  pic32mz lm32 mor1kx m32262f8 sh7032 gap8 nr5m100 sim qemu esp32 z16f2811
  ez80 z180 z8 z80"

for ARCH in ${ARCHLIST}; do
  for CHIP in ${CHIPLIST}; do
    if [ -f ${TOPDIR}/boards/${ARCH}/${CHIP}/${BOARDSUBDIR}/Kconfig ]; then
      ARCHSUBDIR=${ARCH}
      CHIPSUBDIR=${CHIP}
      echo "  Detected ${ARCHSUBDIR} Architecture"
      echo "  Detected ${CHIPSUBDIR} Chip"
    fi
  done
done

for ARCH in ${ARCHLIST}; do
  if [ -f boards/${ARCH}/${BOARDSUBDIR}/Kconfig ]; then
    ARCHSUBDIR=${ARCH}
  fi
done

if [ -z "${ARCHSUBDIR}" ]; then
  echo "ERROR:  Architecture of ${BOARDSUBDIR} not found"
  exit 1
fi

# Where are we

MYNAME=`basename $0`

if [ -x ./${MYNAME} ] ; then
   cd .. || { echo "ERROR: cd .. failed" ; exit 1 ; }
fi

if [ ! -x tools/${MYNAME} ] ; then
   echo "ERROR:  This file must be executed from the top-level NuttX directory: $PWD"
   exit 1
fi

# Set up the environment

BOARDDIR=boards/$ARCHSUBDIR/$CHIPSUBDIR/$BOARDSUBDIR
SCRIPTSDIR=$BOARDDIR/scripts
MAKEDEFS1=$SCRIPTSDIR/Make.defs

CONFIGDIR=$BOARDDIR/configs/$CONFIGSUBDIR
DEFCONFIG=$CONFIGDIR/defconfig
MAKEDEFS2=$CONFIGDIR/Make.defs

CMPCONFIG_TARGET=cmpconfig
CMPCONFIG1=tools/cmpconfig
CMPCONFIG2=tools/cmpconfig.exe
CMPCONFIGMAKEFILE=Makefile.host
CMPCONFIGMAKEDIR=tools

# Check the board configuration directory

if [ ! -d "$BOARDDIR" ]; then
  echo "No board directory found at $BOARDDIR"
  exit 1
fi

if [ ! -d "$CONFIGDIR" ]; then
  echo "No configuration directory found at $CONFIGDIR"
  exit 1
fi

if [ ! -r "$DEFCONFIG" ]; then
  echo "No readable defconfig file at $DEFCONFIG"
  exit 1
fi

if [ -r "$MAKEDEFS1" ]; then
  MAKEDEFS=$MAKEDEFS1
else
  if [ -r "$MAKEDEFS2" ]; then
    MAKEDEFS=$MAKEDEFS2
  else
    echo "No readable Make.defs file at $MAKEDEFS1 or $MAKEDEFS2"
    exit 1
  fi
fi

# If the cmpconfig executable does not exist, then build it

if [ -x ${CMPCONFIG1} ]; then
  CMPCONFIG=${CMPCONFIG1}
else
  if [ -x ${CMPCONFIG2} ]; then
    CMPCONFIG=${CMPCONFIG2}
  else
    make -C ${CMPCONFIGMAKEDIR} -f ${CMPCONFIGMAKEFILE} ${CMPCONFIG_TARGET} || \
      { echo "ERROR: make ${CMPCONFIG1} failed" ; exit 1 ; }
  fi
fi

if [ -x ${CMPCONFIG1} ]; then
  CMPCONFIG=${CMPCONFIG1}
else
  if [ -x ${CMPCONFIG2} ]; then
    CMPCONFIG=${CMPCONFIG2}
  else
    echo "ERROR: Failed to create  ${CMPCONFIG1}"
    exit 1
  fi
fi

# Copy the .config and Make.defs to the toplevel directory

rm -f SAVEconfig
if [ -e .config ]; then
  mv .config SAVEconfig || \
    { echo "ERROR: Failed to move .config to SAVEconfig"; exit 1; }
fi

cp -a $DEFCONFIG .config || \
  { echo "ERROR: Failed to copy $DEFCONFIG to .config"; exit 1; }

rm -f SAVEMake.defs
if [ -e Make.defs ]; then
  mv Make.defs SAVEMake.defs || \
    { echo "ERROR: Failed to move Make.defs to SAVEMake.defs"; exit 1; }
fi

cp -a $MAKEDEFS Make.defs || \
  { echo "ERROR: Failed to copy $MAKEDEFS to Make.defs"; exit 1; }

# Then run oldconfig or oldefconfig

if [ "X${defaults}" == "Xy" ]; then
  make olddefconfig
else
  make oldconfig
fi

# Run savedefconfig to create the new defconfig file

make savedefconfig

# Show differences

# sed -i -e "s/^CONFIG_APPS_DIR/# CONFIG_APPS_DIR/g" defconfig
$CMPCONFIG $DEFCONFIG defconfig

# Save the refreshed configuration

if [ "X${prompt}" == "Xy" ]; then
  read -p "Save the new configuration (y/n)?" -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]
  then
    echo "Saving the new configuration file"
    mv defconfig $DEFCONFIG || \
        { echo "ERROR: Failed to move defconfig to $DEFCONFIG"; exit 1; }
    chmod 644 $DEFCONFIG
  fi
else
  echo "Saving the new configuration file"
  mv defconfig $DEFCONFIG || \
      { echo "ERROR: Failed to move defconfig to $DEFCONFIG"; exit 1; }
  chmod 644 $DEFCONFIG
fi

# Restore any previous .config and Make.defs files

if [ -e SAVEconfig ]; then
  mv SAVEconfig .config || \
    { echo "ERROR: Failed to move SAVEconfig to .config"; exit 1; }
fi

if [ -e SAVEMake.defs ]; then
  mv SAVEMake.defs Make.defs || \
    { echo "ERROR: Failed to move SAVEMake.defs to Make.defs"; exit 1; }
fi
