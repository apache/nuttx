#!/usr/bin/env bash
# tools/mkwindeps.sh
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

# Uncomment to enable debug options
# set -x
# DEBUG=--dep-debug

# Make sure that we know where we are

TOOLDIR=$(dirname $0)

if [ ! -x ${TOOLDIR}/mkwindeps.sh ]; then
  echo "# ERROR: tools/ directory not found"
  exit 1
fi

# Make sure that executables are ready

MKDEPS=${TOOLDIR}/mkdeps.exe
CNVWINDEPS=${TOOLDIR}/cnvwindeps.exe

if [ ! -x ${MKDEPS} ]; then
  echo "# ERROR: tools/mkdeps.exe does not exist"
  exit 1
fi

if [ ! -x ${CNVWINDEPS} ]; then
  echo "# ERROR: tools/cnvwindeps.exe does not exist"
  exit 1
fi

# Run the mkdeps.exe program to generate a Windows dependency file

TMPFILE=$(mktemp)
${MKDEPS} ${DEBUG} --winpath $* > ${TMPFILE} || { echo "# ERROR: mkdeps.exe failed"; exit 1; }

# Then convert this to a POSIX dependency file (on stdout)

${CNVWINDEPS} ${TMPFILE}
rm -f ${TMPFILE}
