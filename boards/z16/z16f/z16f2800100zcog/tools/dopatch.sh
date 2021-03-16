#!/usr/bin/env bash
############################################################################
# boards/z16/z16f/z16f2800100zcog/tools/dopatch.sh
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

USAGE="${0} [-R] \$PWD"
WD=`pwd`
TOOLDIR=${WD}/boards/z16f2800100zcog/tools
ME=${TOOLDIR}/dopatch.sh
PATCH=${TOOLDIR}/zneo-zdsii-5_0_1-variadic-func-fix.patch
ARGS=${1}

if [ ! -x ${ME} ]; then
  echo "ERROR:  This script must be executed from the top-level NuttX directory"
  echo ${USAGE}
  exit 1
fi

if [ ! -r ${PATCH} ]; then
  echo "ERROR: Readable patch not found at ${PATCH}"
  echo ${USAGE}
  exit 1
fi

cd .. || \
  { echo "ERROR: failed to CD to the parent directory"; exit 1; }

cat ${PATCH} | patch ${ARGS} -p1 || \
  { echo "ERROR: patch failed" ; exit 1; }
