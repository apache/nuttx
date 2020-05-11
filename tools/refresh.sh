#!/usr/bin/env bash
# tools/refresh.sh
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

USAGE="USAGE: $0 [options] <board>:<config>+"
ADVICE="Try '$0 --help' for more information"

unset CONFIGS
diff=0
debug=n
defaults=n
prompt=y
nocopy=n

while [ ! -z "$1" ]; do
  case $1 in
  --debug )
    debug=y
    ;;
  --silent )
    defaults=y
    prompt=n
    ;;
  --prompt )
    prompt=y
    ;;
  --defaults )
    defaults=y
    ;;
  --nocopy )
    nocopy=y
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
    echo "  --nocopy"
    echo "     Do not copy defconfig from nuttx/boards/<board>/configs to nuttx/.config"
    echo "  --help"
    echo "     Show this help message and exit"
    echo "  <board>"
    echo "     The board directory under nuttx/boards"
    echo "  <config>"
    echo "     The board configuration directory under nuttx/boards/<board>/configs"
    echo "  Note: all configuration is refreshed if <board>:<config> equals all."
    exit 0
    ;;
  * )
    CONFIGS=$*
    break
    ;;
  esac
  shift
done

# Where are we

MYNAME=`basename $0`

cd $WD
if [ -x ./${MYNAME} ] ; then
  cd .. || { echo "ERROR: cd .. failed" ; exit 1 ; }
fi

if [ ! -x tools/${MYNAME} ] ; then
  echo "ERROR:  This file must be executed from the top-level NuttX directory: $PWD"
  exit 1
fi

# Get the board configuration

if [ -z "${CONFIGS}" ]; then
  echo "ERROR: No configuration provided"
  echo $USAGE
  echo $ADVICE
  exit 1
fi

if [ "X${CONFIGS}" == "Xall" ]; then
  CONFIGS=`find boards -name defconfig | cut -d'/' -f4,6`
fi

for CONFIG in ${CONFIGS}; do
  echo "  Normalize ${CONFIG}"

  # Set up the environment

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

  BOARDDIR=boards/*/*/$BOARDSUBDIR
  SCRIPTSDIR=$BOARDDIR/scripts
  MAKEDEFS1=$SCRIPTSDIR/Make.defs

  CONFIGDIR=$BOARDDIR/configs/$CONFIGSUBDIR
  DEFCONFIG=$CONFIGDIR/defconfig
  MAKEDEFS2=$CONFIGDIR/Make.defs

  # Check the board configuration directory

  if [ ! -d $BOARDDIR ]; then
    echo "No board directory found at $BOARDDIR"
    exit 1
  fi

  if [ ! -d $CONFIGDIR ]; then
    echo "No configuration directory found at $CONFIGDIR"
    exit 1
  fi

  if [ ! -r $DEFCONFIG ]; then
    echo "No readable defconfig file at $DEFCONFIG"
    exit 1
  fi

  if [ -r $MAKEDEFS2 ]; then
    MAKEDEFS=$MAKEDEFS2
  else
    if [ -r $MAKEDEFS1 ]; then
      MAKEDEFS=$MAKEDEFS1
    else
      echo "No readable Make.defs file at $MAKEDEFS1 or $MAKEDEFS2"
      exit 1
    fi
  fi

  # Copy the .config and Make.defs to the toplevel directory

  rm -f SAVEconfig
  rm -f SAVEMake.defs

  if [ "X${nocopy}" != "Xy" ]; then
    if [ -e .config ]; then
      mv .config SAVEconfig || \
        { echo "ERROR: Failed to move .config to SAVEconfig"; exit 1; }
    fi

    cp -a $DEFCONFIG .config || \
      { echo "ERROR: Failed to copy $DEFCONFIG to .config"; exit 1; }

    if [ -e Make.defs ]; then
      mv Make.defs SAVEMake.defs || \
        { echo "ERROR: Failed to move Make.defs to SAVEMake.defs"; exit 1; }
    fi

    cp -a $MAKEDEFS Make.defs || \
      { echo "ERROR: Failed to copy $MAKEDEFS to Make.defs"; exit 1; }

    # Then run oldconfig or oldefconfig

    if [ "X${defaults}" == "Xy" ]; then
      if [ "X${debug}" == "Xy" ]; then
        make olddefconfig V=1
      else
        make olddefconfig 1>/dev/null
      fi
    else
      if [ "X${debug}" == "Xy" ]; then
        make oldconfig V=1
      else
        make oldconfig
      fi
    fi
  fi

  # Run savedefconfig to create the new defconfig file

  if [ "X${debug}" == "Xy" ]; then
    make savedefconfig V=1
  else
    make savedefconfig 1>/dev/null
  fi

  # Show differences

  if ! diff $DEFCONFIG defconfig; then

    # Save the refreshed configuration

    if [ "X${prompt}" == "Xy" ]; then

      read -p "Save the new configuration (y/n)?" -n 1 -r
      echo
      if [[ $REPLY =~ ^[Yy]$ ]]; then
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

    diff=1
  fi

  # Restore any previous .config and Make.defs files

  if [ -e SAVEMake.defs ]; then
    mv SAVEMake.defs Make.defs || \
      { echo "ERROR: Failed to move SAVEMake.defs to Make.defs"; exit 1; }
  fi

  if [ -e SAVEconfig ]; then
    mv SAVEconfig .config || \
      { echo "ERROR: Failed to move SAVEconfig to .config"; exit 1; }

    if [ "X${debug}" == "Xy" ]; then
      ./tools/sethost.sh V=1
    else
      ./tools/sethost.sh 1>/dev/null
    fi
  fi
done

exit $diff
