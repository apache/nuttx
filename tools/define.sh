#!/usr/bin/env bash
# tools/define.sh
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

#
# Handle command line options
#

progname=$0
wintool=n
usage="USAGE: $progname [-w] [-d] [-h] <compiler-path> <def1>[=val1] [<def2>[=val2] [<def3>[=val3] ...]]"
advice="Try '$progname -h' for more information"

while [ ! -z "$1" ]; do
  case $1 in
  -d )
    set -x
    ;;
  -w )
    wintool=y
    ;;
  -h )
    echo "$progname is a tool for flexible generation of command line pre-processor"
    echo "definitions arguments for a variety of diffent compilers in a variety of"
    echo "compilation environments"
    echo ""
    echo $usage
    echo ""
    echo "Where:"
    echo "  <compiler-path>"
    echo "    The full path to your compiler"
    echo "  <def1> <def2> [<def3> ..."
    echo "    A list of pre-preprocesser variable names to be defined."
    echo "  [=val1] [=val2] [=val3]"
    echo "    optional values to be assigned to each pre-processor variable."
    echo "    If not supplied, the variable will be defined with no explicit value."
    echo "  -w"
    echo "    The compiler is a Windows native tool and requires Windows"
    echo "    style pathnames like C:\\Program Files"
    echo "  -d"
    echo "    Enable script debug"
    ;;
  * )
    break
    ;;
  esac
  shift
done

ccpath=$1
shift
varlist=$@

if [ -z "$ccpath" ]; then
  echo "Missing compiler path"
  echo $usage
  echo $advice
  exit 1
fi

if [ -z "$varlist" ]; then
  echo "Missing definition list"
  echo $usage
  echo $advice
  exit 1
fi

#
# Most compilers support CFLAG options like '-D<defn>' to add pre-processor
# variable definitions.  Some (like the Zilog tools), do not.  This script
# makes the selection of pre-processor definitions compiler independent.
#
# Below are all known compiler names (as found in the config/*/*/Make.defs
# files).  If a new compiler is used that has some unusual syntax, then
# additional logic needs to be added to this file.
#
#   NAME                        Syntax
#   $(CROSSDEV)gcc              -D<def1> -D<def2> -D<def3> ...
#   sdcc                        -D<def2> -D<def2> -D<def3> ...
#   $(ZDSBINDIR)/ez8cc.exe      -define:<def1> -define:<def2> -define:<def3> ...
#   $(ZDSBINDIR)/zneocc.exe     -define:<def1> -define:<def2> -define:<def3> ...
#   $(ZDSBINDIR)/ez80cc.exe     -define:<def1> -define:<def2> -define:<def3> ...
#
os=`uname -o 2>/dev/null || echo "Other"`

#
# Let's assume that all GCC compiler paths contain the string gcc and
# no non-GCC compiler paths include this substring
#
gcc=`echo $ccpath | grep gcc`
sdcc=`echo $ccpath | grep sdcc`

if [ "X$os" = "XCygwin" ]; then
  #
  # We can treat Cygwin native toolchains just like Linux native
  # toolchains in the Linux.  Let's assume:
  # 1. GCC or SDCC are the only possible Cygwin native compilers
  # 2. If this is a Window native GCC version, then -w provided
  #    on the command line (wintool=y)

  if [ -z "$gcc" -a -z "$sdcc" ]; then
    #
    # Not GCC or SDCC, must be Windows native
    #
    compiler=`cygpath -u "$ccpath"`
  else
    if [ "X$wintool" == "Xy" ]; then
      #
      # It is a native GCC or SDCC compiler
      #
      compiler=`cygpath -u "$ccpath"`
    else
      #
      # GCC or SDCC and not for Windows
      #
      compiler="$ccpath"
    fi
  fi
else
  #
  # Otherwise, we must be in a Linux environment where there are
  # only Linux native toolchains
  #
  compiler="$ccpath"
fi
exefile=`basename "$compiler"`

# Check for some well known, non-GCC Windows native tools that require
# a special output format as well as special paths

if [ "X$exefile" = "Xez8cc.exe" -o "X$exefile" = "Xzneocc.exe" -o "X$exefile" = "Xez80cc.exe" ]; then
  fmt=define
else
  fmt=std
fi

# Now process each definition in the definition list

unset response
for vardef in $varlist; do

  varname=`echo $vardef | cut -d'=' -f1`
  if [ "X$varname" != "X$vardef" ]; then
    varvalue=`echo $vardef | cut -d'=' -f2`
  else
    unset varvalue
  fi

  # Handle the output depending on if there is a value for the variable or not

  if [ -z "$varvalue" ]; then

    # Handle the output using the selected format

    if [ "X$fmt" = "Xdefine" ]; then
      # Treat the first definition differently

      if [ -z "$response" ]; then
        response="-define:"$varname
      else
        response=$response" -define:$varname"
      fi
    else
      # Treat the first definition differently

      if [ -z "$response" ]; then
        response=-D$varname
      else
        response=$response" -D$varname"
      fi
    fi
  else

    # Handle the output using the selected format

    if [ "X$fmt" = "Xdefine" ]; then
      # Treat the first definition differently

      if [ -z "$response" ]; then
        response="-define:"$varname=$varvalue
      else
        response=$response" -define:$varname=$varvalue"
      fi
    else
      # Treat the first definition differently

      if [ -z "$response" ]; then
        response=-D$varname=$varvalue
      else
        response=$response" -D$varname=$varvalue"
      fi
    fi
  fi
done

echo $response
