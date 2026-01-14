#!/usr/bin/env bash
# tools/releasesdownloader.sh
#
# SPDX-License-Identifier: Apache-2.0
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
# This simple script uses wget to download all git tags release artifacts.
# At time of 12.10.0 release all rtos and apps downloads take ~4GB in size.
# Non-existent artifacts (HTTP/404 status) will exist but have zero size.
# Useful for testing and comparison between releases.

VERSION="20251216"
TS=`date +%s`
ARGC=$#
TAGS=`git tag -l`
REL_URL_RTOS="https://github.com/apache/nuttx/archive/refs/tags/"
REL_URL_APPS="https://github.com/apache/nuttx-apps/archive/refs/tags/"
REL_EXT=".tar.gz"
REL_OUT="../../nuttx-packages"
WRKDIR=`pwd`
WGET_FLAGS="-c -nv --show-progress -a wget-$TS.log --retry-on-http-error=503\
  --wait=1 --waitretry=60 --keep-badhash"

if [ "$1" != "" ]; then REL_OUT=$1; fi
if [ $# -ge 2 ]; then
  TAGS=""
  while shift && [ -n "$1" ]; do TAGS="$TAGS $1"; done;
fi

printf "====================================================================\n"
printf " NuttX RTOS and Apps release packages fetch script version $VERSION\n"
printf "====================================================================\n\n"
printf "  Usage: $0 [download_path] [tags_list_space_separated]\n\n"
printf "  Destination : $REL_OUT.\n"
printf "     Git tags : `echo $TAGS|tr -d '\n'`.\n\n"

wget --version &>/dev/null || {
    echo "ERROR: Please install wget to use this script!"; exit 1
}

if [ $ARGC -lt 2 ]; then printf "NOTE: Using all tags from current nuttx repo clone.\n"; fi
printf "NOTE: Zero package size = no corresponding tag.\n"
printf "WARNING: This tool may fetch several GB of data!\n\n"
printf "Press Return to continue, Ctrl+C to abort.\n"
read x

if [ ! -d $REL_OUT ]; then printf "Creating: $REL_OUT.\n\n"; mkdir $REL_OUT; fi
cd $REL_OUT
    
set +e
for TAG in $TAGS; do
    wget $WGET_FLAGS -O $TAG$REL_EXT $REL_URL_RTOS$TAG$REL_EXT 2>&1
    wget $WGET_FLAGS -O ${TAG%%-*}-apps-${TAG#*-}$REL_EXT $REL_URL_APPS$TAG$REL_EXT 2>&1
done

printf "\nDone! Log is at: $REL_OUT/wget-$TS.log.\n"

cd $WKDIR
