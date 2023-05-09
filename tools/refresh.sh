#!/usr/bin/env bash
# tools/refresh.sh
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
    echo "     The board directory under nuttx/boards/arch/chip/"
    echo "  <config>"
    echo "     The board configuration directory under nuttx/boards/arch/chip/<board>/configs"
    echo "  <archname>"
    echo "     The architecture directory under nuttx/boards/"
    echo "  <chipname>"
    echo "     The chip family directory under nuttx/boards/<arch>/"
    echo "  Note1: all configuration is refreshed if <board>:<config> equals all."
    echo "  Note2: all configuration of arch XYZ is refreshed if \"arch:<namearch>\" is passed"
    echo "  Note3: all configuration of chip XYZ is refreshed if \"chip:<chipname>\" is passed"
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
  echo "Normalizing all boards!"
  CONFIGS=`find boards -name defconfig | cut -d'/' -f4,6`
else
  if [[ "X${CONFIGS}" == "Xarch:"* ]]; then
    IFS=: read -r atype archname <<< "${CONFIGS}"
    ARCH=$archname
    echo "Normalizing all boards in arch: ${ARCH} !"
    CONFIGS=`find boards/${ARCH} -name defconfig | cut -d'/' -f4,6`
  else
    if [[ "X${CONFIGS}" == "Xchip:"* ]]; then
      IFS=: read -r atype chipname <<< "${CONFIGS}"
      CHIP=$chipname
      echo "Normalizing all boards in chip: ${CHIP} !"
      CONFIGS=`find boards/*/${CHIP} -name defconfig | cut -d'/' -f4,6`
    fi
  fi
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
