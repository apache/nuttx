#!/usr/bin/env bash
############################################################################
# tools/showsize.sh
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

# set -x

# Host nm should always work
# vs. NM=arm-none-eabi-nm

NM=nm

# This should be executed from the top-level NuttX directory

if [ ! -x "tools/showsize.sh" ]; then
  echo "This script must executed from the top-level NuttX directory"
  exit 1
fi

# On the cywin simulation, the executable will be nuttx.exe

if [ -f "nuttx" ]; then
  NUTTX=nuttx
else
  if [ -x "nuttx.exe" ]; then
    NUTTX=nuttx.exe
  else
    echo "Cannot find the NuttX executable"
    exit 1
  fi
fi

# Show what we were asked for

echo "TOP 10 BIG DATA"
$NM --print-size --size-sort --radix dec -C $NUTTX | grep ' [DdBb] ' | tail -20

echo "TOP 10 BIG CODE"
$NM --print-size --size-sort --radix dec -C $NUTTX | grep ' [TtWw] ' | tail -20
