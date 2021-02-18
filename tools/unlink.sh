#!/usr/bin/env bash
############################################################################
# tools/unlink.sh
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

link=$1

# Verify that arguments were provided

if [ -z "${link}" ]; then
  echo "Missing link argument"
  exit 1
fi

# Check if something already exists at the link path

if [ -e "${link}" ]; then

  # Yes, is it a symbolic link?  If so, then remove it

  if [ -h "${link}" ]; then
    rm -f "${link}"
  else

    # If the path is a directory and contains the "fake link" mark, then
    # treat it like a soft link (i.e., remove the directory)

    if [ -d "${link}" -a -f "${link}/.fakelnk" ]; then
      rm -rf "${link}"
    else

      # It is something else (like a file) or directory that does
      # not contain the "fake link" mark

      echo "${link} already exists but is not a symbolic link"
      exit 1
    fi
  fi
fi
