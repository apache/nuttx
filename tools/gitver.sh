#!/bin/bash

#****************************************************************************
# tools/gitver.sh
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
#****************************************************************************

# Example usage:
# $ ./gitver.sh 0.1
#   0.1.2

if [ $# -ne 1 ]
then
    echo "Missing version argument" 1>&2
    exit -1
fi 

NUTTX_VERSION=$1

# Closest Tag of this Version
VER_TAG=`git describe --abbrev=0 --match "nuttx-$NUTTX_VERSION" --match "nuttx-$NUTTX_VERSION-*" --match "nuttx-$NUTTX_VERSION.[0-9]*"  2>/dev/null`
if [ $? -ne 0 ]
then
    VER_TAG=nuttx-$NUTTX_VERSION
fi


# Remove nuttx and possible RC
VER_TAG=`echo $VER_TAG | cut -d'-' --fields=2`

# If version does not have a patch add one
if [ `expr match $VER_TAG '.'` -eq 1 ]
then
    VER_TAG=${VER_TAG}.0
fi

SEMVER_REGEX="([0-9]+).([0-9]+).([0-9]+)"
if [[ $VER_TAG =~ $SEMVER_REGEX ]]
then
    major="${BASH_REMATCH[1]}"
    minor="${BASH_REMATCH[2]}"
    patch="${BASH_REMATCH[3]}"
else
    echo "Unexpected Version format: $VER_TAG"  1>&2
    exit -1
fi

echo ${major}.${minor}.${patch}
