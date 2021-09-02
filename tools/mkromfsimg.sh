#!/usr/bin/env bash
############################################################################
# tools/mkromfsimg.sh
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

# Environmental stuff

wd=`pwd`
workingdir=$wd/img
rcsysinitfile=rc.sysinit
rcsysinittemplate=$rcsysinitfile.template
rcsfile=rcS
rcstemplate=$rcsfile.template
romfsimg=romfs.img
headerfile=nsh_romfsimg.h

# Get the input parameters

nofat=$1
usefat=true
topdir=$2
rcsysinit_fname=$3
rcs_fname=$4
usage="USAGE: $0 [-nofat] <topdir> [rcsysinitfile] [<rcsfile>]"

# Verify if we have the optional "-nofat"

if [ "$nofat" == "-nofat" ]; then
  echo "We will not mount a FAT/RAMDISK!"
  usefat=false
else
  topdir=$1
  rcsysinit_fname=$2
  rcs_fname=$3
fi

if [ -z "$topdir" -o ! -d "$topdir" ]; then
  echo "The full path to the NuttX base directory must be provided on the command line"
  echo $usage
  exit 1
fi

# Verify if we have the optional "rcsysinit_fname" and "rcs_fname"

if [ ! -z "$rcsysinit_fname" ]; then
  rcsysinittemplate=$rcsysinit_fname
  echo "Target template is $rcsysinittemplate"
fi

if [ ! -z "$rcs_fname" ]; then
  rcstemplate=$rcs_fname
  echo "Target template is $rcstemplate"
fi

# Extract all values from the .config in the $topdir that contains all of the NuttX
# configuration settings.  The .config file was intended to be include-able by makefiles
# and source-able by scripts.  Unfortunately,there are too many syntactic difference
# to make that practical

if [ ! -r $topdir/.config ]; then
  echo "No readable file at $topdir/.config"
  echo "Has NuttX been configured?"
  exit 1
fi

romfsetc=`grep CONFIG_NSH_ROMFSETC= $topdir/.config | cut -d'=' -f2`
disablempt=`grep CONFIG_DISABLE_MOUNTPOINT= $topdir/.config | cut -d'=' -f2`
disablescript=`grep CONFIG_NSH_DISABLESCRIPT= $topdir/.config | cut -d'=' -f2`
devconsole=`grep CONFIG_DEV_CONSOLE= $topdir/.config | cut -d'=' -f2`
romfs=`grep CONFIG_FS_ROMFS= $topdir/.config | cut -d'=' -f2`
romfsmpt=`grep CONFIG_NSH_ROMFSMOUNTPT= $topdir/.config | cut -d'=' -f2`
initscript=`grep CONFIG_NSH_INITSCRIPT= $topdir/.config | cut -d'=' -f2`
sysinitscript=`grep CONFIG_NSH_SYSINITSCRIPT= $topdir/.config | cut -d'=' -f2`
romfsdevno=`grep CONFIG_NSH_ROMFSDEVNO= $topdir/.config | cut -d'=' -f2`
romfssectsize=`grep CONFIG_NSH_ROMFSSECTSIZE= $topdir/.config | cut -d'=' -f2`

# If we disabled FAT FS requirement, we don't need to check it

if [ "$usefat" = true ]; then
  fatfs=`grep CONFIG_FS_FAT= $topdir/.config | cut -d'=' -f2`
  fatdevno=`grep CONFIG_NSH_FATDEVNO= $topdir/.config | cut -d'=' -f2`
  fatsectsize=`grep CONFIG_NSH_FATSECTSIZE= $topdir/.config | cut -d'=' -f2`
  fatnsectors=`grep CONFIG_NSH_FATNSECTORS= $topdir/.config | cut -d'=' -f2`
  fatmpt=`grep CONFIG_NSH_FATMOUNTPT= $topdir/.config | cut -d'=' -f2`
fi

# The following settings are required for general ROMFS support
#
# Mountpoint support must be enabled

if [ "X$disablempt" = "Xy" ]; then
  echo "Mountpoint support is required for this feature"
  echo "Set CONFIG_DISABLE_MOUNTPOINT=n to continue"
  exit 1
fi

# Scripting support must be enabled

if [ "X$disablescript" = "Xy" ]; then
  echo "NSH scripting support is required for this feature"
  echo "Set CONFIG_NSH_DISABLESCRIPT=n to continue"
  exit 1
fi

# ROMFS support is required, of course

if [ "X$romfs" != "Xy" ]; then
  echo "ROMFS support is disabled in the NuttX configuration"
  echo "Set CONFIG_FS_ROMFS=y to continue"
  exit 0
fi

# If it is the default rcS.template, then it also requires FAT FS support

if [ "$usefat" = true -a "X$fatfs" != "Xy" ]; then
  echo "FAT FS support is disabled in the NuttX configuration"
  echo "Set CONFIG_FS_FAT=y to continue"
  exit 0
fi

# Verify that genromfs has been installed

genromfs -h 1>/dev/null 2>&1 || { \
  echo "Host executable genromfs not available in PATH"; \
  echo "You may need to download in from http://romfs.sourceforge.net/"; \
  exit 1; \
}

# Supply defaults for all un-defined ROMFS settings

if [ -z "$romfsmpt" ]; then
  romfsmpt=\"/etc\"
fi
if [ -z "$initscript" ]; then
  initscript=\"init.d/rcS\"
fi
if [ -z "$sysinitscript" ]; then
  sysinitscript=\"init.d/rc.sysinit\"
fi
if [ -z "$romfsdevno" ]; then
  romfsdevno=0
fi
if [ -z "$romfssectsize" ]; then
  romfssectsize=64
fi

# If FAT FS is a requirement

if [ "$usefat" = true ]; then

  # Supply defaults for all un-defined FAT FS settings

  if [ -z "$fatdevno" ]; then
    fatdevno=1
  fi
  if [ -z "$fatsectsize" ]; then
    fatsectsize=512
  fi
  if [ -z "$fatnsectors" ]; then
    fatnsectors=1024
  fi
  if [ -z "$fatmpt" ]; then
   fatmpt=\"/tmp\"
  fi
fi

# Verify the mountpoint.  Verify that it is an absolute path but not /, /dev,
# /., /./*, /.., or /../*

if [ ${romfsmpt:0:1} != "\"" ]; then
  echo "CONFIG_NSH_ROMFSMOUNTPT must be a string"
  echo "Change it so that it is enclosed in quotes."
  exit 1
fi

uromfsmpt=`echo $romfsmpt | sed -e "s/\"//g"`

if [ ${uromfsmpt:0:1} != "/" ]; then
  echo "CONFIG_NSH_ROMFSMOUNTPT must be an absolute path in the target FS"
  echo "Change it so that it begins with the character '/'.  Eg. /etc"
  exit 1
fi

tmpdir=$uromfsmpt
while [ ${tmpdir:0:1} == "/" ]; do
  tmpdir=${tmpdir:1}
done

if [ -z "$tmpdir" -o "X$tmpdir" = "Xdev" -o "X$tmpdir" = "." -o \
     ${tmpdir:0:2} = "./" -o "X$tmpdir" = ".." -o ${tmpdir:0:3} = "../" ]; then
  echo "Invalid CONFIG_NSH_ROMFSMOUNTPT selection."
  exit 1
fi

# Verify that the path to the init file is a relative path and not ., ./*, .., or ../*

if [ ${initscript:0:1} != "\"" ]; then
  echo "CONFIG_NSH_INITSCRIPT must be a string"
  echo "Change it so that it is enclosed in quotes."
  exit 1
fi

uinitscript=`echo $initscript | sed -e "s/\"//g"`

if [ ${uinitscript:0:1} == "/" ]; then
  echo "CONFIG_NSH_INITSCRIPT must be an relative path in under $romfsmpt"
  echo "Change it so that it begins with the character '/'.  Eg. init.d/rcS. "
  exit 1
fi

if [ "X$uinitscript" = "."  -o ${uinitscript:0:2} = "./" -o \
     "X$uinitscript" = ".." -o ${uinitscript:0:3} = "../" ]; then
  echo "Invalid CONFIG_NSH_INITSCRIPT selection.  Must not begin with . or .."
  exit 1
fi

if [ ${sysinitscript:0:1} != "\"" ]; then
  echo "CONFIG_NSH_SYSINITSCRIPT must be a string"
  echo "Change it so that it is enclosed in quotes."
  exit 1
fi

usysinitscript=`echo $sysinitscript | sed -e "s/\"//g"`

if [ ${usysinitscript:0:1} == "/" ]; then
  echo "CONFIG_NSH_SYSINITSCRIPT must be an relative path in under $romfsmpt"
  echo "Change it so that it begins with the character '/'.  Eg. init.d/rc.sysinit. "
  exit 1
fi

if [ "X$usysinitscript" = "."  -o ${usysinitscript:0:2} = "./" -o \
     "X$usysinitscript" = ".." -o ${usysinitscript:0:3} = "../" ]; then
  echo "Invalid CONFIG_NSH_SYSINITSCRIPT selection.  Must not begin with . or .."
  exit 1
fi

# Create a working directory

rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }
mkdir -p $workingdir || { echo "Failed to created the new $workingdir"; exit 1; }

# Create the rc.sysinit file from the rc.sysinit.template

if [ ! -r $rcsysinittemplate ]; then
  echo "$rcsysinittemplate does not exist"
  rmdir $workingdir
  exit 1
fi

# If we are using FAT FS with RAMDISK we need to setup it

if [ "$usefat" = true ]; then
  cat $rcsysinittemplate | \
      sed -e "s,XXXMKRDMINORXXX,$fatdevno,g" | \
      sed -e "s,XXMKRDSECTORSIZEXXX,$fatsectsize,g" | \
      sed -e "s,XXMKRDBLOCKSXXX,$fatnsectors,g" | \
      sed -e "s,XXXRDMOUNTPOINTXXX,$fatmpt,g" >$rcsysinitfile
else
  cp $rcsysinittemplate $rcsysinitfile
fi

# Create the rcS file from the rcS.template

if [ ! -r $rcstemplate ]; then
  echo "$rcstemplate does not exist"
  rmdir $workingdir
  exit 1
fi

cp $rcstemplate $rcsfile

# And install it at the specified relative location

# Fix for BSD install without -D option
mkdir -p $workingdir/$uinitscript
rmdir $workingdir/$uinitscript

install -m 0755 $rcsysinitfile $workingdir/$usysinitscript || \
    { echo "Failed to install $rcsysinitfile at $workingdir/$usysinitscript"; rm -f $rcsysinitfile; exit 1; }
rm -f $rcsysinitfile

install -m 0755 $rcsfile $workingdir/$uinitscript || \
    { echo "Failed to install $rcsfile at $workingdir/$uinitscript"; rm -f $rcsfile; exit 1; }
rm -f $rcsfile

# Now we are ready to make the ROMFS image

genromfs -f $romfsimg -d $workingdir -V "NSHInitVol" || { echo "genromfs failed" ; exit 1 ; }
rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }

# And, finally, create the header file

echo '#include <nuttx/compiler.h>' >${headerfile}
xxd -i ${romfsimg} | sed 's/^unsigned char/const unsigned char aligned_data(4)/g' >>${headerfile} || \
  { echo "ERROR: xxd of $< failed" ; rm -f $romfsimg; exit 1 ; }
rm -f $romfsimg
