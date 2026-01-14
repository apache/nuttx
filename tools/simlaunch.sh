#!/usr/bin/env bash
# tools/simlaunch.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

if ! command -v patchelf &> /dev/null; then
    echo "patchelf does not installed:" 1>&2
    echo "sudo apt-get update" 1>&2
    echo "sudo apt-get install patchelf" 1>&2
    exit 1
fi

CWD=`pwd`

if [ ! -d ${CWD}/libs ]; then
    echo "Directory for nuttx libs does not exist in ${CWD}." 1>&2
    exit 1
fi

if [ ! -e ${CWD}/nuttx ]; then
    echo "Nuttx elf does not exist in ${CWD}." 1>&2
    exit 1
fi

INTERPRETER=$(find "${CWD}" -maxdepth 1 -type f -name '*ld-linux*')

if [ ! -n "${INTERPRETER}" ]; then
    echo "No dynamic interpreter exist in ${CWD}." 1>&2
    exit 1
fi

CURRENT_INTERPRETER=$(readelf -l nuttx |grep "program interpreter" | awk -F':' '{print $2}'| cut -d"]" -f1 | awk '{$1=$1};1')

if [ "$INTERPRETER" != "$CURRENT_INTERPRETER" ]; then
    echo "Patch nuttx so interpreter to ${INTERPRETER}"
    patchelf --set-interpreter "$INTERPRETER" nuttx
fi

export LD_LIBRARY_PATH=${CWD}/libs
"${CWD}/nuttx"
